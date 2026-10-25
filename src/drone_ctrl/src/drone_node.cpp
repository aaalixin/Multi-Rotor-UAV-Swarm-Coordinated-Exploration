#include <ros/ros.h>
#include <geometry_msgs/PoseStamped.h>
#include <mavros_msgs/State.h>
#include <mavros_msgs/CommandBool.h>
#include <mavros_msgs/SetMode.h>
#include <mavros_msgs/PositionTarget.h>
#include <sensor_msgs/Imu.h>
#include <cmath>

constexpr double TAKEOFF_HEIGHT = 4.0;
enum State { INIT, OFFBOARD, ARM, TAKEOFF, FLY };

class DroneNode {
public:
    DroneNode() : nh_("~") {
        int id; nh_.param("drone_id", id, 0);
        drone_id_ = id;
        std::string ns = "/typhoon_h480_" + std::to_string(id);
        ros::NodeHandle g;

        state_sub_ = g.subscribe<mavros_msgs::State>(ns+"/mavros/state",10,
            &DroneNode::cbState, this);
        pose_sub_  = g.subscribe<geometry_msgs::PoseStamped>(ns+"/mavros/local_position/pose",10,
            &DroneNode::cbPose, this);
        setpoint_pub_ = g.advertise<mavros_msgs::PositionTarget>(ns+"/mavros/setpoint_raw/local",10);
        arm_client_   = g.serviceClient<mavros_msgs::CommandBool>(ns+"/mavros/cmd/arming");
        mode_client_  = g.serviceClient<mavros_msgs::SetMode>(ns+"/mavros/set_mode");
    }

    void run() {
        ros::Rate rate(30);
        ros::Time state_start = ros::Time::now();

        while (ros::ok()) {
            if (state_ == INIT) {
                if (connected_) { state_ = OFFBOARD; state_start = ros::Time::now(); }
            }
            else if (state_ == OFFBOARD) {
                heartbeat();
                if (mode_=="OFFBOARD") {
                    if ((ros::Time::now()-state_start).toSec()>2.0)
                        { state_ = ARM; state_start=ros::Time::now(); }
                } else {
                    mavros_msgs::SetMode sm; sm.request.custom_mode="OFFBOARD";
                    mode_client_.call(sm);
                }
            }
            else if (state_ == ARM) {
                heartbeat();
                if (armed_) { state_=TAKEOFF; state_start=ros::Time::now(); }
                else {
                    mavros_msgs::CommandBool arm; arm.request.value=true;
                    arm_client_.call(arm);
                }
            }
            else if (state_ == TAKEOFF) {
                if (wz_ >= TAKEOFF_HEIGHT-0.3) {
                    state_=FLY;
                    ROS_INFO("[drone%d] At %.1fm, flying forward",drone_id_,wz_);
                } else {
                    flyTraj(wx_,wy_,TAKEOFF_HEIGHT,0,0,0);
                }
            }
            else if (state_ == FLY) {
                // Simply fly forward at 1.5 m/s
                flyTraj(wx_,wy_,TAKEOFF_HEIGHT,8.0,0,0);
            }

            ros::spinOnce();
            rate.sleep();
        }
    }

private:
    ros::NodeHandle nh_;
    int drone_id_;
    State state_ = INIT;

    ros::Subscriber state_sub_, pose_sub_;
    ros::Publisher setpoint_pub_;
    ros::ServiceClient arm_client_, mode_client_;

    bool connected_=false, armed_=false; std::string mode_;
    double wx_=0,wy_=0,wz_=0;

    void cbState(const mavros_msgs::State::ConstPtr& m){connected_=m->connected;armed_=m->armed;mode_=m->mode;}
    void cbPose(const geometry_msgs::PoseStamped::ConstPtr& m){wx_=m->pose.position.x;wy_=m->pose.position.y;wz_=m->pose.position.z;}

    void heartbeat(){mavros_msgs::PositionTarget pt;pt.coordinate_frame=1;pt.type_mask=255;setpoint_pub_.publish(pt);}
    void flyTraj(double px,double py,double pz,double vx,double vy,double yaw){
        mavros_msgs::PositionTarget pt;
        pt.coordinate_frame=mavros_msgs::PositionTarget::FRAME_LOCAL_NED;
        pt.type_mask=mavros_msgs::PositionTarget::IGNORE_AFX|mavros_msgs::PositionTarget::IGNORE_AFY|mavros_msgs::PositionTarget::IGNORE_AFZ|mavros_msgs::PositionTarget::IGNORE_YAW_RATE|mavros_msgs::PositionTarget::IGNORE_PZ;
        pt.position.x=px;pt.position.y=py;
        pt.velocity.x=vx;pt.velocity.y=vy;
        pt.velocity.z=(pz-wz_)*1.0;
        pt.yaw=yaw;
        setpoint_pub_.publish(pt);
    }
};

int main(int argc,char**argv){ros::init(argc,argv,"drone_node");DroneNode().run();return 0;}
