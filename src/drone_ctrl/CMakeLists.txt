cmake_minimum_required(VERSION 3.0.2)
project(drone_ctrl)
find_package(catkin REQUIRED COMPONENTS roscpp geometry_msgs mavros_msgs sensor_msgs std_msgs nav_msgs pcl_ros pcl_conversions)
find_package(PCL REQUIRED)
find_package(Eigen3 REQUIRED)
find_package(gazebo REQUIRED)
include_directories(include ${catkin_INCLUDE_DIRS} ${PCL_INCLUDE_DIRS} ${EIGEN3_INCLUDE_DIR} ${GAZEBO_INCLUDE_DIRS})
catkin_package()

add_executable(simple_explore src/simple_explore.cpp)
target_link_libraries(simple_explore ${catkin_LIBRARIES} ${PCL_LIBRARIES})

add_executable(gimbal_bridge src/gimbal_bridge.cpp)
target_link_libraries(gimbal_bridge ${catkin_LIBRARIES} ${GAZEBO_LIBRARIES})

install(TARGETS simple_explore RUNTIME DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION})
install(DIRECTORY scripts DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION} USE_SOURCE_PERMISSIONS)
