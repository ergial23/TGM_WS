	cmake_minimum_required(VERSION 3.0.2)
project(sensor_model)

find_package(catkin REQUIRED COMPONENTS
  roscpp
  rospy
  std_msgs
  tf2_ros
  sensor_msgs
  nav_msgs
  pcl_ros
)
catkin_package()

find_package(PCL REQUIRED)

include_directories(
  ${catkin_INCLUDE_DIRS}
  ${PCL_INCLUDE_DIRS}
  include
)
add_library(preprocessing_library
  src/preprocessing_library.cpp
)

link_directories(${PCL_LIBRARY_DIRS})
add_definitions(${PCL_DEFINITIONS})

add_executable(sensor_model_node src/sensor_model_node.cpp)

target_include_directories(sensor_model_node PUBLIC ${catkin_INCLUDE_DIRS} ${PCL_INCLUDE_DIRS})
target_link_libraries(sensor_model_node ${catkin_LIBRARIES} ${PCL_LIBRARIES} preprocessing_library)
