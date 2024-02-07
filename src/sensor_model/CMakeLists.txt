cmake_minimum_required(VERSION 3.0.2)
project(sensor_model)

find_package(catkin REQUIRED COMPONENTS
  roscpp
  rospy
  std_msgs
  tf2_ros
)

catkin_package()

include_directories(${catkin_INCLUDE_DIRS})

add_executable(sensor_model_node src/sensor_model_node.cpp)
target_link_libraries(sensor_model_node ${catkin_LIBRARIES})
