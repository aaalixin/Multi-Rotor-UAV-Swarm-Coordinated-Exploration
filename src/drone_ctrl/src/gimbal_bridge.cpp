#include <ros/ros.h>
#include <std_msgs/Float64.h>
#include <gazebo/transport/transport.hh>
#include <gazebo/msgs/msgs.hh>
#include <vector>

// ROS Float64 -> gazebo transport Any(double), 转发给自定义gimbal插件
// /typhoon_h480_X/gimbal_pitch_cmd (rad) -> /gazebo/<world>/typhoon_h480_X/gimbal_pitch_cmd
int main(int argc, char** argv)
{
    ros::init(argc, argv, "gimbal_bridge");
    ros::NodeHandle nh("~");
    std::string world_name;
    nh.param("world_name", world_name, std::string("default"));

    gazebo::transport::NodePtr gznode(new gazebo::transport::Node());
    gznode->Init();

    std::vector<ros::Subscriber> subs(6);
    std::vector<gazebo::transport::PublisherPtr> pubs(6);

    for (int i = 0; i < 6; i++) {
        std::string gz_topic = "/gazebo/" + world_name + "/typhoon_h480_" + std::to_string(i) + "/gimbal_pitch_cmd";
        pubs[i] = gznode->Advertise<gazebo::msgs::Any>(gz_topic);
        subs[i] = nh.subscribe<std_msgs::Float64>("/typhoon_h480_" + std::to_string(i) + "/gimbal_pitch_cmd", 1,
            [i, pubs](const std_msgs::Float64::ConstPtr& m) {
                gazebo::msgs::Any msg;
                msg.set_type(gazebo::msgs::Any_ValueType_DOUBLE);
                msg.set_double_value(m->data);
                pubs[i]->Publish(msg);
            });
        ROS_INFO("gimbal_bridge drone%d -> %s", i, gz_topic.c_str());
    }
    ros::spin();
    return 0;
}
