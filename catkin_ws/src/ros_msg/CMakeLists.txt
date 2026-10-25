cmake_minimum_required(VERSION 3.0.2)
project(ros_msg)

find_package(catkin REQUIRED COMPONENTS
  rospy
  std_msgs
  geometry_msgs
  #gazebo_ros_actor_cmd_plugin_msgs
  message_generation
)

add_message_files(
  FILES
  LockReport.msg
  TargetRaw.msg
  LockMap.msg
)

generate_messages(
  DEPENDENCIES
  std_msgs
  geometry_msgs
)

catkin_package(
  CATKIN_DEPENDS rospy std_msgs geometry_msgs gazebo_ros_actor_cmd_plugin_msgs message_runtime
)
