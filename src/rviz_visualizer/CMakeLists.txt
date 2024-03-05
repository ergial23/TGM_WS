cmake_minimum_required(VERSION 3.0.2)
project(rviz_visualizer)

find_package(catkin REQUIRED COMPONENTS
  roscpp
  rospy
  std_msgs
  visualization_msgs
 
)
catkin_package()



include_directories(
  ${catkin_INCLUDE_DIRS}

)



add_executable(car_publisher src/car_publisher.cpp)

target_include_directories(car_publisher PUBLIC ${catkin_INCLUDE_DIRS})
target_link_libraries(car_publisher ${catkin_LIBRARIES})
