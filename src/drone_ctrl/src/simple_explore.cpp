#include <ros/ros.h>
#include <geometry_msgs/PoseStamped.h>
#include <geometry_msgs/Twist.h>
#include <mavros_msgs/State.h>
#include <mavros_msgs/CommandBool.h>
#include <mavros_msgs/SetMode.h>
#include <mavros_msgs/PositionTarget.h>
#include <std_msgs/Bool.h>
#include <sensor_msgs/LaserScan.h>
#include <cmath>
#include <vector>

constexpr double TAKEOFF_H=3.5, SPEEDS[6]={15,15,15,15,15,15};

struct WpConfig {std::vector<std::pair<double,double>> wps;double sox,soy;double spd;};

WpConfig cfgs[7][6]={
    { // route 0 (contX.py)
        {{{-14,-3},{-14,-22},{-14,-40},{2,-40},{21,-40},{42.5,-40},{42.5,-22},{42.5,-3},{60,-3},{75,-3},{90,-3},{105,-3},{105,-22},{105,-40},{90,-40},{75,-40},{60,-40},{42.5,-40},{42.5,-22},{42.5,-3},{21,-3},{2,-3}},-17,-3,15},
        {{{-11,-4.5},{0,-4.5},{15,-4.5},{30,-4.5},{40,-4.5},{40,-21.5},{40,-38},{55,-38},{70,-38},{85,-38},{100,-38},{114.5,-38},{114.5,-21.5},{114.5,-4.5},{114.5,15},{114.5,30},{114.5,45},{100,45},{85,45},{70,45},{55,45},{40,45},{25,45},{5,45},{-11,45},{-11,30},{-11,15}},-14,-3,15},
        {{{-16,-1.5},{-30,-1.5},{-45,-1.5},{-45,-21.5},{-45,-40},{-25,-40},{-10.5,-40},{-10.5,-20},{-10.5,-1.5},{5,-1.5},{20,-1.5},{35,-1.5},{48,-1.5},{48,-20},{48,-42.5},{35,-42.5},{20,-42.5},{5,-42.5},{-10.5,-42.5},{-10.5,-20},{-10.5,-5}},-17,0,15},
        {{{-11,4.5},{0,4.5},{15,4.5},{30,4.5},{40,4.5},{40,21.5},{40,38},{55,38},{70,38},{85,38},{100,38},{112.5,38},{112.5,21.5},{112.5,4.5},{112.5,-15},{112.5,-30},{112.5,-45},{100,-45},{85,-45},{70,-45},{55,-45},{40,-45},{25,-45},{5,-45},{-11,-45},{-11,-30},{-11,-15}},-14,0,15},
        {{{-16,1.5},{-30,1.5},{-45,1.5},{-45,21.5},{-45,40},{-25,40},{-10.5,40},{-10.5,15},{-10.5,1.5},{5,1.5},{20,1.5},{35,1.5},{48,1.5},{48,15},{48,42.5},{35,42.5},{20,42.5},{5,42.5},{-10.5,42.5},{-10.5,15},{-10.5,5}},-17,3,15},
        {{{-14,3},{-14,22},{-14,40},{2,40},{21,40},{42.5,40},{42.5,22},{42.5,3},{60,3},{75,3},{90,3},{105,3},{105,22},{105,40},{90,40},{75,40},{60,40},{42.5,40},{42.5,22},{42.5,3},{21,3},{2,3}},-14,3,15},
    },
    { // route 1 (contX_1.py)
        {{{-14,-12.5},{-14,-24.5},{-14,-40},{2,-40},{21,-40},{42.5,-40},{42.5,-24.5},{42.5,-12.5},{60,-12.5},{75,-12.5},{90,-12.5},{105,-12.5},{105,-22},{105,-40},{90,-40},{75,-40},{60,-40},{42.5,-40},{42.5,-22},{42.5,-12.5},{21,-12.5},{2,-12.5}},-17,-3,15},
        {{{-11,0},{-11,-14},{-14,-14},{0,-14},{15,-14},{30,-14},{40,-14},{40,-21.5},{40,-38},{55,-38},{70,-38},{85,-38},{100,-38},{114.5,-38},{114.5,-21.5},{114.5,-14},{114.5,0},{114.5,15},{114.5,30},{114.5,45},{100,45},{85,45},{70,45},{55,45},{40,45},{25,45},{5,45},{-11,45},{-11,30},{-11,15}},-14,-3,15},
        {{{-17,-11},{-30,-11},{-45,-11},{-45,-21.5},{-45,-40},{-25,-40},{-10.5,-40},{-10.5,-20},{-10.5,-11},{5,-11},{20,-11},{35,-11},{48,-11},{48,-20},{48,-42.5},{35,-42.5},{20,-42.5},{5,-42.5},{-10.5,-42.5},{-10.5,-20},{-10.5,-14.5},{-16,-11}},-17,0,15},
        {{{-14,-5},{0,-5},{15,-5},{30,-5},{40,-5},{40,21.5},{40,38},{55,38},{70,38},{85,38},{100,38},{112.5,38},{112.5,21.5},{112.5,4.5},{112.5,-15},{112.5,-30},{112.5,-45},{100,-45},{85,-45},{70,-45},{55,-45},{40,-45},{25,-45},{5,-45},{-11,-45},{-11,-30},{-11,-15},{-11,-5}},-14,0,15},
        {{{-10.5,-4.5},{-16,-8},{-17,-8},{-30,-8},{-45,-8},{-45,10},{-45,25},{-45,40},{-25,40},{-10.5,40},{-10.5,15},{-10.5,-8},{5,-8},{20,-8},{35,-8},{48,-8},{48,15},{48,42.5},{35,42.5},{20,42.5},{5,42.5},{-10.5,42.5},{-10.5,15}},-17,3,15},
        {{{-14,10},{-14,26},{-14,40},{2,40},{21,40},{42.5,40},{42.5,26},{42.5,10},{42.5,-6.5},{60,-6.5},{75,-6.5},{90,-6.5},{105,-6.5},{105,10},{105,26},{105,40},{90,40},{75,40},{60,40},{42.5,40},{42.5,26},{42.5,10},{42.5,-6.5},{21,-6.5},{2,-6.5},{-14,-6.5}},-14,3,15},
    },
    { // route 2 (contX_2.py)
        {{{-14,-10},{-14,-25},{-14,-40},{2,-40},{21,-40},{42.5,-40},{42.5,-25},{42.5,-10},{42.5,7.8},{60,7.8},{75,7.8},{90,7.8},{105,7.8},{105,-10},{105,-25},{105,-40},{90,-40},{75,-40},{60,-40},{42.5,-40},{42.5,-25},{42.5,-10},{42.5,7.8},{21,7.8},{2,7.8},{-14,7.8}},-17,-3,15},
        {{{-14,6.3},{0,6.3},{15,6.3},{30,6.3},{40,6.3},{40,-11.7},{40,-21.5},{40,-38},{55,-38},{70,-38},{85,-38},{100,-38},{114.5,-38},{114.5,-21.5},{114.5,-11.7},{114.5,6.3},{114.5,15},{114.5,30},{114.5,45},{100,45},{85,45},{70,45},{55,45},{40,45},{25,45},{5,45},{-11,45},{-11,30},{-11,15},{-11,6.3}},-14,-3,15},
        {{{-10.5,-5},{-10.5,9.3},{-16,9.3},{-17,9.3},{-30,9.3},{-45,9.3},{-45,-10},{-45,-21.5},{-45,-40},{-25,-40},{-10.5,-40},{-10.5,-20},{-10.5,-5},{-10.5,9.3},{5,9.3},{20,9.3},{35,9.3},{48,9.3},{48,-5},{48,-20},{48,-42.5},{35,-42.5},{20,-42.5},{5,-42.5},{-10.5,-42.5},{-10.5,-20}},-17,0,15},
        {{{-11,0},{-11,15.3},{-14,15.3},{0,15.3},{15,15.3},{30,15.3},{40,15.3},{40,25},{40,38},{55,38},{70,38},{85,38},{100,38},{112.5,38},{112.5,25},{112.5,15.3},{112.5,0},{112.5,-15},{112.5,-30},{112.5,-45},{100,-45},{85,-45},{70,-45},{55,-45},{40,-45},{25,-45},{5,-45},{-11,-45},{-11,-30},{-11,-15}},-14,0,15},
        {{{-17,12.3},{-30,12.3},{-45,12.3},{-45,25},{-45,40},{-25,40},{-10.5,40},{-10.5,25},{-10.5,12.3},{5,12.3},{20,12.3},{35,12.3},{48,12.3},{48,25},{48,42.5},{35,42.5},{20,42.5},{5,42.5},{-10.5,42.5},{-10.5,25},{-10.5,15.8},{-16,12.3}},-17,3,15},
        {{{-14,13.8},{-14,26},{-14,40},{2,40},{21,40},{42.5,40},{42.5,26},{42.5,13.8},{60,13.8},{75,13.8},{90,13.8},{105,13.8},{105,26},{105,40},{90,40},{75,40},{60,40},{42.5,40},{42.5,26},{42.5,13.8},{21,13.8},{2,13.8}},-14,3,15},
    },
    { // route 3 (contX_3.py)
        {{{-14,-12.5},{-14,-24.5},{-14,-40},{2,-40},{21,-40},{42.5,-40},{60,-40},{60,-24.5},{60,-12.5},{75,-12.5},{90,-12.5},{105,-12.5},{105,-22},{105,-40},{90,-40},{75,-40},{60,-40},{60,-22},{60,-12.5},{40,-12.5},{21,-12.5},{2,-12.5}},-17,-3,15},
        {{{-11,0},{-11,-14},{-14,-14},{0,-14},{15,-14},{30,-14},{57,-14},{57,-21.5},{57,-38},{70,-38},{85,-38},{100,-38},{114.5,-38},{114.5,-21.5},{114.5,-14},{114.5,0},{114.5,15},{114.5,30},{114.5,45},{100,45},{85,45},{70,45},{55,45},{40,45},{25,45},{5,45},{-11,45},{-11,30},{-11,15}},-14,-3,15},
        {{{-17,-11},{-30,-11},{-40,-11},{-40,-21.5},{-45,-21.5},{-45,-40},{-25,-40},{-10.5,-40},{-10.5,-20},{-10.5,-11},{5,-11},{20,-11},{35,-11},{48,-11},{62,-11},{62,-20},{62,-42.5},{48,-42.5},{35,-42.5},{20,-42.5},{5,-42.5},{-10.5,-42.5},{-10.5,-20},{-10.5,-14.5},{-16,-11}},-17,0,15},
        {{{-14,-5},{0,-5},{15,-5},{32,-5},{57,-5},{57,21.5},{57,38},{70,38},{85,38},{100,38},{112.5,38},{112.5,21.5},{112.5,4.5},{112.5,-15},{112.5,-30},{112.5,-45},{100,-45},{85,-45},{70,-45},{55,-45},{40,-45},{25,-45},{5,-45},{-11,-45},{-11,-30},{-11,-15},{-11,-5}},-14,0,15},
        {{{-10.5,-4.5},{-16,-8},{-17,-8},{-30,-8},{-45,-8},{-45,10},{-45,25},{-45,40},{-25,40},{-10.5,40},{-10.5,15},{-10.5,-8},{5,-8},{20,-8},{35,-8},{48,-8},{62,-8},{62,15},{62,42.5},{40,42.5},{35,42.5},{20,42.5},{5,42.5},{-10.5,42.5},{-10.5,15}},-17,3,15},
        {{{-14,10},{-14,26},{-14,40},{2,40},{21,40},{42.5,40},{60,40},{60,26},{60,10},{60,-6.5},{75,-6.5},{90,-6.5},{105,-6.5},{105,10},{105,26},{105,40},{90,40},{75,40},{60,40},{60,26},{60,10},{60,-6.5},{42,-6.5},{21,-6.5},{2,-6.5},{-14,-6.5}},-14,3,15},
    },
    { // route 4 (contX_4.py)
        {{{-14,-10},{-14,-25},{-14,-40},{2,-40},{21,-40},{42.5,-40},{60,-40},{60,-25},{60,-10},{60,7.8},{75,7.8},{90,7.8},{105,7.8},{105,-10},{105,-25},{105,-40},{90,-40},{75,-40},{60,-40},{60,-25},{60,-10},{60,7.8},{21,7.8},{2,7.8},{-14,7.8}},-17,-3,15},
        {{{-14,6.3},{0,6.3},{15,6.3},{30,6.3},{57,6.3},{57,-11.7},{57,-21.5},{57,-38},{70,-38},{85,-38},{100,-38},{114.5,-38},{114.5,-21.5},{114.5,-11.7},{114.5,6.3},{114.5,15},{114.5,30},{114.5,45},{100,45},{85,45},{70,45},{55,45},{40,45},{25,45},{5,45},{-11,45},{-11,30},{-11,15},{-11,6.3}},-14,-3,15},
        {{{-10.5,-5},{-10.5,9.3},{-16,9.3},{-17,9.3},{-30,9.3},{-45,9.3},{-45,-10},{-45,-21.5},{-45,-40},{-25,-40},{-10.5,-40},{-10.5,-20},{-10.5,-5},{-10.5,9.3},{5,9.3},{20,9.3},{35,9.3},{48,9.3},{62,9.3},{62,-5},{62,-20},{62,-42.5},{48,-42.5},{35,-42.5},{20,-42.5},{5,-42.5},{-10.5,-42.5},{-10.5,-20}},-17,0,15},
        {{{-11,0},{-11,15.3},{-14,15.3},{0,15.3},{15,15.3},{30,15.3},{57,15.3},{57,25},{57,38},{70,38},{85,38},{100,38},{112.5,38},{112.5,25},{112.5,15.3},{112.5,0},{112.5,-15},{112.5,-30},{112.5,-45},{100,-45},{85,-45},{70,-45},{55,-45},{40,-45},{25,-45},{5,-45},{-11,-45},{-11,-30},{-11,-15}},-14,0,15},
        {{{-17,12.3},{-30,12.3},{-45,12.3},{-45,25},{-45,40},{-25,40},{-10.5,40},{-10.5,25},{-10.5,12.3},{5,12.3},{20,12.3},{35,12.3},{48,12.3},{62,12.3},{62,25},{62,42.5},{48,42.5},{35,42.5},{20,42.5},{5,42.5},{-10.5,42.5},{-10.5,25},{-10.5,15.8},{-16,12.3}},-17,3,15},
        {{{-14,13.8},{-14,26},{-14,40},{2,40},{21,40},{42.5,40},{60,40},{60,26},{60,13.8},{75,13.8},{90,13.8},{105,13.8},{105,26},{105,40},{90,40},{75,40},{60,40},{60,26},{60,13.8},{40,13.8},{21,13.8},{2,13.8}},-14,3,15},
    },
    { // route 5 (contX_6.py)
        {{{-14,-3},{-14,-22},{-14,-40},{2,-40},{15,-40},{28,-40},{28,-22},{28,-3},{40,-3},{60,-3},{75,-3},{90,-3},{105,-3},{105,-22},{105,-40},{90,-40},{75,-40},{60,-40},{42.5,-40},{28,-40},{28,-22},{28,-3},{2,-3}},-17,-3,15},
        {{{-11,-4.5},{0,-4.5},{10,-4.5},{25,-4.5},{25,-21.5},{25,-38},{40,-38},{55,-38},{70,-38},{85,-38},{100,-38},{114.5,-38},{114.5,-21.5},{114.5,-4.5},{114.5,15},{114.5,30},{114.5,45},{100,45},{85,45},{70,45},{55,45},{40,45},{25,45},{5,45},{-11,45},{-11,30},{-11,15}},-14,-3,15},
        {{{-16,-1.5},{-16,-20},{-10.5,-20},{-10.5,-5},{-30,-1.5},{-45,-1.5},{-45,-21.5},{-45,-40},{-25,-40},{-10.5,-40},{-10.5,-20},{-10.5,-1.5},{5,-1.5},{20,-1.5},{35,-1.5},{35,-20},{35,-42.5},{20,-42.5},{5,-42.5},{-10.5,-42.5},{-10.5,-1.5}},-17,0,15},
        {{{-14,4.5},{0,4.5},{15,4.5},{30,4.5},{37,4.5},{37,21.5},{37,38},{55,38},{70,38},{85,38},{100,38},{112.5,38},{112.5,21.5},{112.5,4.5},{112.5,-15},{112.5,-30},{112.5,-45},{100,-45},{85,-45},{70,-45},{55,-45},{40,-45},{25,-45},{5,-45},{-11,-45},{-11,-30},{-11,-15},{-11,4.5}},-14,0,15},
        {{{-16,1.5},{-16,15},{-10.5,15},{-10.5,5},{-30,1.5},{-45,1.5},{-45,21.5},{-45,40},{-25,40},{-10.5,40},{-10.5,15},{-10.5,1.5},{5,1.5},{20,1.5},{35,1.5},{35,15},{35,42.5},{20,42.5},{5,42.5},{-10.5,42.5},{-10.5,1.5}},-17,3,15},
        {{{-14,3},{-14,22},{-14,40},{2,40},{27,40},{27,22},{27,0},{40,0},{60,3},{75,3},{90,3},{105,3},{105,22},{105,40},{90,40},{75,40},{60,40},{42.5,40},{27,40},{27,22},{27,3},{2,3}},-14,3,15},
    },
    { // route 6 (contX_7.py)
        {{{-14,-3},{-14,-22},{-14,-40},{2,-40},{21,-40},{42.5,-40},{60,-40},{60,-22},{60,-3},{75,-3},{90,-3},{105,-3},{105,-22},{105,-40},{90,-40},{75,-40},{60,-40},{60,-22},{60,-3},{42.5,-3},{21,-3},{2,-3}},-17,-3,15},
        {{{-11,-4.5},{0,-4.5},{15,-4.5},{30,-4.5},{40,-4.5},{57,-4.5},{57,-21.5},{57,-38},{70,-38},{85,-38},{100,-38},{114.5,-38},{114.5,-21.5},{114.5,-4.5},{114.5,15},{114.5,30},{114.5,45},{100,45},{85,45},{70,45},{57,45},{40,45},{25,45},{5,45},{-11,45},{-11,30},{-11,15}},-14,-3,15},
        {{{-16,-1.5},{-30,-1.5},{-45,-1.5},{-45,-21.5},{-45,-40},{-25,-40},{-10.5,-40},{-10.5,-20},{-10.5,-1.5},{5,-1.5},{20,-1.5},{35,-1.5},{48,-1.5},{62,-1.5},{62,-20},{62,-42.5},{42,-42.5},{35,-42.5},{20,-42.5},{5,-42.5},{-10.5,-42.5},{-10.5,-20},{-10.5,-5}},-17,0,15},
        {{{-14,4.5},{0,4.5},{15,4.5},{30,4.5},{40,4.5},{57,4.5},{57,21.5},{57,38},{70,38},{85,38},{100,38},{112.5,38},{112.5,21.5},{112.5,4.5},{112.5,-15},{112.5,-30},{112.5,-45},{100,-45},{85,-45},{70,-45},{57,-45},{40,-45},{25,-45},{5,-45},{-11,-45},{-11,-30},{-11,-15},{-11,4.5}},-14,0,15},
        {{{-16,1.5},{-30,1.5},{-45,1.5},{-45,21.5},{-45,40},{-25,40},{-10.5,40},{-10.5,15},{-10.5,1.5},{5,1.5},{20,1.5},{35,1.5},{48,1.5},{62,1.5},{62,15},{62,42.5},{42,42.5},{35,42.5},{20,42.5},{5,42.5},{-10.5,42.5},{-10.5,15},{-10.5,5}},-17,3,15},
        {{{-14,3},{-14,22},{-14,40},{2,40},{21,40},{42.5,40},{60,40},{60,22},{60,3},{75,3},{90,3},{105,3},{105,22},{105,40},{90,40},{75,40},{60,40},{60,22},{60,3},{42.5,3},{21,3},{2,3}},-14,3,15},
    },
};

std::vector<std::pair<double,double>> densify(const std::vector<std::pair<double,double>>& wps, double step){
    std::vector<std::pair<double,double>> out;
    for(size_t i=0;i<wps.size();i++){
        auto a=wps[i], b=wps[(i+1)%wps.size()];
        double d=std::sqrt((b.first-a.first)*(b.first-a.first)+(b.second-a.second)*(b.second-a.second));
        int n=std::max(1,(int)std::ceil(d/step));
        for(int k=0;k<n;k++){
            double t=(double)k/n;
            out.push_back({a.first+(b.first-a.first)*t, a.second+(b.second-a.second)*t});
        }
    }
    return out;
}

struct DroneNode{
    ros::NodeHandle nh;
    DroneNode():nh("~"){
        int id;nh.param("drone_id",id,0);
        int rid;nh.param("route_id",rid,0);if(rid<0)rid=0;if(rid>6)rid=6;
        auto& cfg=cfgs[rid][id];waypoints=densify(cfg.wps,3.0);off_x=cfg.sox;off_y=cfg.soy;speed=cfg.spd;drone_id=id;
        wp_idx=0;target=waypoints[0];
        speed = 12;
        // 标记是否已经完成起飞阶段（第一个航点）
        takeoff_complete = false;
        ROS_INFO("[drone%d] route=%d %zu waypoints off(%d,%d) spd=%.0f",drone_id,rid,waypoints.size(),(int)off_x,(int)off_y,speed);
    }
    int drone_id;double off_x,off_y,speed;
    std::vector<std::pair<double,double>> waypoints;
    std::pair<double,double> target;
    int wp_idx,state=0;
    bool takeoff_complete = false;  // 新增：标记起飞阶段是否完成

    double wx(){return (lx-ox)+off_x;}double wy(){return (ly-oy)+off_y;}
    double wz_w(){return wz-oz;}
    double lx=0,ly=0,wz=0,yaw=0;
    double ox=0,oy=0,oz=0;bool origin_ok=false;int origin_cnt=0;double sx=0,sy=0,sz=0;
    bool con=false,armed=false;std::string mode;
    ros::Subscriber s_s,s_p,s_track,s_scan;
    ros::Publisher p_set,p_track_enable;
    ros::ServiceClient ca,cm;
    bool tracking=false;
    bool track_enable=false;  // 起飞高度≥2m后置true, 之后一直发true
    ros::Time last_track_msg;

    // 激光雷达避障
    bool lidar_avoid_active = false;
    double lidar_avoid_vx = 0.0;
    double lidar_avoid_vy = 0.0;
    double odom_vx = 0.0;
    double odom_vy = 0.0;
    bool odom_vel_ready = false;

    void initROS(){
        std::string ns="/typhoon_h480_"+std::to_string(drone_id);ros::NodeHandle g;
        s_s=g.subscribe<mavros_msgs::State>(ns+"/mavros/state",10,&DroneNode::cbS,this);
        s_p=g.subscribe<geometry_msgs::PoseStamped>(ns+"/mavros/local_position/pose",10,&DroneNode::cbP,this);
        s_track=g.subscribe<std_msgs::Bool>("/drone_"+std::to_string(drone_id)+"/tracking",1,&DroneNode::cbTrack,this);
        s_scan=g.subscribe<sensor_msgs::LaserScan>(ns+"/scan",10,&DroneNode::cbScan,this);
        p_set=g.advertise<geometry_msgs::Twist>("/xtdrone/typhoon_h480_"+std::to_string(drone_id)+"/cmd_vel_flu",10);
        p_track_enable=g.advertise<std_msgs::Bool>("/drone_"+std::to_string(drone_id)+"/track_enable",1);
        ca=g.serviceClient<mavros_msgs::CommandBool>(ns+"/mavros/cmd/arming");
        cm=g.serviceClient<mavros_msgs::SetMode>(ns+"/mavros/set_mode");
    }
    void cbS(const mavros_msgs::State::ConstPtr& m){con=m->connected;armed=m->armed;mode=m->mode;}
    void cbP(const geometry_msgs::PoseStamped::ConstPtr& m){
        lx=m->pose.position.x;ly=m->pose.position.y;wz=m->pose.position.z;
        double qx=m->pose.orientation.x,qy=m->pose.orientation.y,qz=m->pose.orientation.z,qw=m->pose.orientation.w;
        yaw=std::atan2(2.0*(qw*qz+qx*qy),1.0-2.0*(qy*qy+qz*qz));
        if(!origin_ok){
            sx+=lx;sy+=ly;sz+=wz;origin_cnt++;
            if(origin_cnt>=10){ox=sx/origin_cnt;oy=sy/origin_cnt;oz=sz/origin_cnt;origin_ok=true;
                ROS_INFO("[drone%d] origin=(%.2f,%.2f,%.2f)",drone_id,ox,oy,oz);}
        }
        // 用位置差分算速度
        static double last_lx=0, last_ly=0;
        static ros::Time last_time = ros::Time::now();
        double dt = (ros::Time::now() - last_time).toSec();
        if(dt > 0.01 && dt < 0.1){
            odom_vx = (lx - last_lx) / dt;
            odom_vy = (ly - last_ly) / dt;
            odom_vel_ready = true;
        }
        last_lx = lx; last_ly = ly;
        last_time = ros::Time::now();
    }

    // 激光雷达避障回调 - 起飞阶段跳过避障
    void cbScan(const sensor_msgs::LaserScan::ConstPtr& msg){
        // ===== 起飞阶段（第一个航点）不进行避障 =====
        if(!takeoff_complete){
            lidar_avoid_active = false;
            return;
        }
        
        if(wz_w() < 1.0 || !odom_vel_ready){
            lidar_avoid_active = false;
            return;
        }

        // ===== 固定避障参数 =====
        double obstacle_dist = 4.5;
        double avoid_speed = 4.5;

        // ===== 获取机体系下的速度方向 =====
        double cos_yaw = std::cos(yaw);
        double sin_yaw = std::sin(yaw);
        double vx_body = cos_yaw * odom_vx + sin_yaw * odom_vy;
        double vy_body = -sin_yaw * odom_vx + cos_yaw * odom_vy;
        double vel_mag = std::sqrt(vx_body*vx_body + vy_body*vy_body);

        // 速度太小不触发
        if(vel_mag < 0.3){
            lidar_avoid_active = false;
            return;
        }

        // ===== 运动方向角度（机体系） =====
        double move_angle = std::atan2(vy_body, vx_body);
        const double FOV_HALF = 35.0 * M_PI / 180.0;  // ±35°

        double min_dist = 100.0;
        double min_angle = 0.0;
        bool found = false;

        for(size_t i = 0; i < msg->ranges.size(); ++i){
            double d = msg->ranges[i];
            if(d < obstacle_dist && d > 0.1){
                double angle = msg->angle_min + i * msg->angle_increment;
                double diff = angle - move_angle;
                while(diff > M_PI) diff -= 2.0 * M_PI;
                while(diff < -M_PI) diff += 2.0 * M_PI;
                if(std::abs(diff) <= FOV_HALF){
                    if(d < min_dist){
                        min_dist = d;
                        min_angle = angle;
                        found = true;
                    }
                }
            }
        }

        if(found && min_dist < obstacle_dist){
            double vx = std::cos(min_angle) * avoid_speed;
            double vy = std::sin(min_angle) * avoid_speed;
            double vy_extra = 0.0;
            if(std::abs(min_angle) < 0.25){
                vy_extra = 4.5;
                if(rand() % 2 == 0) vy_extra = -vy_extra;
            } else if(min_angle > 0){
                vy_extra = -avoid_speed * 4.5;
            } else {
                vy_extra = avoid_speed * 4.5;
            }
            lidar_avoid_vx = -vx;
            lidar_avoid_vy = -vy + vy_extra;
            lidar_avoid_active = true;
        } else {
            lidar_avoid_active = false;
        }
    }

    void resumePatrol(){
        double best=1e9;
        for(size_t i=0;i<waypoints.size();i++){
            double d=std::sqrt((waypoints[i].first-wx())*(waypoints[i].first-wx())+
                               (waypoints[i].second-wy())*(waypoints[i].second-wy()));
            if(d<best){best=d;wp_idx=i;}
        }
        target=waypoints[wp_idx];
    }
    void cbTrack(const std_msgs::Bool::ConstPtr& m){
        last_track_msg=ros::Time::now();
        if(tracking && !m->data){
            resumePatrol();
            ROS_INFO("[drone%d] tracking lost, resume wp%d",drone_id,wp_idx);
        }
        tracking=m->data;
    }

    double wrapPi(double a){while(a>M_PI)a-=2*M_PI;while(a<-M_PI)a+=2*M_PI;return a;}
    void hb(){geometry_msgs::Twist tw;p_set.publish(tw);}
    
    void ft(double dx, double dy, double d, double vxy, double vz){
        geometry_msgs::Twist tw;

        // ===== 激光雷达避障优先（起飞阶段跳过） =====
        if(takeoff_complete && lidar_avoid_active){
            tw.linear.x = lidar_avoid_vx;
            tw.linear.y = lidar_avoid_vy;
            tw.linear.z = vz;
            tw.angular.z = 0.0;
            p_set.publish(tw);
            return;
        }

        // ===== 正常追踪 =====
        double target_yaw = std::atan2(dy, dx);
        double yaw_err = wrapPi(target_yaw - yaw);
        
        // 世界系速度
        double wx_body = (d > 0.3) ? dx/d * vxy : 0.0;
        double wy_body = (d > 0.3) ? dy/d * vxy : 0.0;
        
        // 转到机体系 (旋转矩阵)
        double cos_yaw = std::cos(yaw);
        double sin_yaw = std::sin(yaw);
        tw.linear.x = cos_yaw * wx_body + sin_yaw * wy_body;   // 机体系前
        tw.linear.y = -sin_yaw * wx_body + cos_yaw * wy_body;  // 机体系右
        tw.linear.z = vz;
        
        static double last_yaw_err = 0.0;
        double dt = 0.033;
        double yaw_err_dot = (yaw_err - last_yaw_err) / dt;
        last_yaw_err = yaw_err;
        
        double yr = yaw_err * 1.8 + yaw_err_dot * 0.5;
        if(yr > 1.8) yr = 1.8;
        if(yr < -1.8) yr = -1.8;
        tw.angular.z = yr;
        
        p_set.publish(tw);
    }

    void run(){
        ros::Rate rate(30);
        ros::Time t0;
        initROS();
        
        // 等待连接
        while(ros::ok() && !con){ ros::spinOnce(); rate.sleep(); }
        ROS_INFO("[drone%d] Connected", drone_id);
        
        state = 1;
        t0 = ros::Time::now();
        
        while(ros::ok()){
            switch(state){
                case 1: {
                    // ===== OFFBOARD切换 =====
                    // 1. 先发0速度，让PX4知道有控制信号
                    geometry_msgs::Twist zero_twist;
                    zero_twist.linear.x = 0;
                    zero_twist.linear.y = 0;
                    zero_twist.linear.z = 0;
                    zero_twist.angular.z = 0;
                    p_set.publish(zero_twist);
                    
                    // 2. 检查是否已经在OFFBOARD
                    if(mode == "OFFBOARD"){
                        ROS_INFO("[drone%d] Already in OFFBOARD mode", drone_id);
                        state = 2;
                        t0 = ros::Time::now();
                        break;
                    }
                    
                    // 3. 请求OFFBOARD，带重试
                    bool offboard_success = false;
                    for(int i = 0; i < 20; i++){
                        // 每次重试前发0速度
                        p_set.publish(zero_twist);
                        
                        mavros_msgs::SetMode sm;
                        sm.request.custom_mode = "OFFBOARD";
                        
                        if(cm.call(sm) && sm.response.mode_sent){
                            offboard_success = true;
                            ROS_INFO("[drone%d] OFFBOARD set success (try %d)", drone_id, i + 1);
                            break;
                        }
                        
                        // 检查是否已经切换成功（可能mode还没更新）
                        if(mode == "OFFBOARD"){
                            offboard_success = true;
                            ROS_INFO("[drone%d] OFFBOARD already set (mode updated)", drone_id);
                            break;
                        }
                        
                        ROS_WARN("[drone%d] OFFBOARD set failed (try %d/%d), retrying...", 
                                 drone_id, i + 1, 20);
                        ros::Duration(0.3).sleep();
                        ros::spinOnce();
                    }
                    
                    if(!offboard_success){
                        ROS_ERROR("[drone%d] OFFBOARD set failed after 20 retries!", drone_id);
                        ros::Duration(0.5).sleep();
                        break;
                    }
                    
                    ros::Duration(0.3).sleep();
                    state = 2;
                    t0 = ros::Time::now();
                    break;
                }
                
                case 2: {
                    // ===== 解锁 =====
                    if(armed){
                        ROS_INFO("[drone%d] Already armed", drone_id);
                        state = 3;
                        t0 = ros::Time::now();
                        break;
                    }
                    
                    if(mode != "OFFBOARD"){
                        ROS_WARN("[drone%d] Lost OFFBOARD, go back to state 1", drone_id);
                        state = 1;
                        break;
                    }
                    
                    geometry_msgs::Twist zero_twist;
                    zero_twist.linear.x = 0;
                    zero_twist.linear.y = 0;
                    zero_twist.linear.z = 0;
                    zero_twist.angular.z = 0;
                    p_set.publish(zero_twist);
                    
                    bool arm_success = false;
                    for(int i = 0; i < 20; i++){
                        mavros_msgs::CommandBool a;
                        a.request.value = true;
                        
                        if(ca.call(a) && a.response.success){
                            arm_success = true;
                            ROS_INFO("[drone%d] Armed success (try %d)", drone_id, i + 1);
                            break;
                        }
                        
                        if(armed){
                            arm_success = true;
                            ROS_INFO("[drone%d] Already armed", drone_id);
                            break;
                        }
                        
                        ROS_WARN("[drone%d] Armed failed (try %d/%d), retrying...", 
                                 drone_id, i + 1, 20);
                        ros::Duration(0.3).sleep();
                        ros::spinOnce();
                    }
                    
                    if(!arm_success){
                        ROS_ERROR("[drone%d] Armed failed after 20 retries!", drone_id);
                        ros::Duration(0.5).sleep();
                        break;
                    }
                    
                    state = 3;
                    t0 = ros::Time::now();
                    break;
                }
                
                case 3: {
                    // ===== 起飞（第一个航点，不避障） =====
                    bool done = wz_w() >= TAKEOFF_H - 0.3;
                    static int dc = 0;
                    if(++dc % 60 == 0) ROS_INFO("[drone%d] takeoff wz=%.1f (no obstacle avoidance)", drone_id, wz_w());
                    if(done){
                        // 起飞完成，标记takeoff_complete = true
                        takeoff_complete = true;
                        ROS_INFO("[drone%d] Takeoff complete! Obstacle avoidance enabled from next waypoint", drone_id);
                        state = 4;
                        ROS_INFO("[drone%d] Patrol", drone_id);
                    }
                    double dx = target.first - wx();
                    double dy = target.second - wy();
                    double d = std::sqrt(dx*dx + dy*dy);
                    double vz = (wz_w() < TAKEOFF_H * 0.7) ? 3.0 : (TAKEOFF_H - wz_w()) * 0.5;
                    ft(dx, dy, d, speed, vz);
                    break;
                }
                
                case 4: {
                    // ===== 巡逻（从第二个航点开始，恢复避障） =====
                    static ros::Time last_offb(0);
                    bool need_offb = (mode != "OFFBOARD" && (ros::Time::now() - last_offb).toSec() > 2.0);
                    if(need_offb){
                        geometry_msgs::Twist zero_twist;
                        zero_twist.linear.x = 0; 
                        zero_twist.linear.y = 0; 
                        zero_twist.linear.z = 0; 
                        zero_twist.angular.z = 0;
                        p_set.publish(zero_twist);
                        ros::Duration(0.1).sleep();
                        
                        mavros_msgs::SetMode sm;
                        sm.request.custom_mode = "OFFBOARD";
                        cm.call(sm);
                        last_offb = ros::Time::now();
                        ROS_INFO("[drone%d] re-request OFFBOARD (mode=%s)", drone_id, mode.c_str());
                    }
                    
                    if(tracking && (ros::Time::now() - last_track_msg).toSec() > 3.0){
                        ROS_WARN("[drone%d] vision node heartbeat lost, resume patrol", drone_id);
                        tracking = false;
                        resumePatrol();
                    }
                    
                    if(tracking){
                        // Python视觉追踪接管
                    } else {
                        double dx = target.first - wx();
                        double dy = target.second - wy();
                        double d = std::sqrt(dx*dx + dy*dy);
                        if(d < 1.5){
                            wp_idx = (wp_idx + 1) % waypoints.size();
                            target = waypoints[wp_idx];
                            ROS_INFO("[drone%d] reached waypoint %d, next waypoint", drone_id, wp_idx);
                        }
                        dx = target.first - wx();
                        dy = target.second - wy();
                        d = std::sqrt(dx*dx + dy*dy);
                        double vz = (TAKEOFF_H - wz_w()) * 0.5;
                        ft(dx, dy, d, speed, vz);
                    }
                    break;
                }
            }
            if(!track_enable && wz_w() >= 2.0){
                track_enable = true;
                ROS_INFO("[drone%d] track_enable=true (wz=%.1f)", drone_id, wz_w());
            }
            std_msgs::Bool te; te.data = track_enable;
            p_track_enable.publish(te);
            ros::spinOnce();
            rate.sleep();
        }
    }
};

int main(int argc, char** argv){
    ros::init(argc, argv, "se");
    DroneNode node;
    node.run();
    return 0;
}
