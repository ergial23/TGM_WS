cmake_minimum_required(VERSION 3.0.2)
project(sensor_model)

find_package(catkin REQUIRED COMPONENTS
  roscpp
  rospy
  std_msgs
  geometry_msgs
  tf2_ros
  sensor_msgs
  nav_msgs
  pcl_ros
)
catkin_package()

find_package(PCL REQUIRED)
find_package(yaml-cpp REQUIRED)

include_directories(
  ${catkin_INCLUDE_DIRS}
  ${PCL_INCLUDE_DIRS}
  ${YAML_CPP_INCLUDE_DIR}
  include
  params
)
add_library(preprocessing_library
  src/preprocessing_library.cpp
)
add_library(occupancy_grid_map_library
  src/occupancy_grid_map_library.cpp
)

link_directories(${PCL_LIBRARY_DIRS})
add_definitions(${PCL_DEFINITIONS})

add_executable(preprocessing_node src/preprocessing_node.cpp)
add_executable(occupancy_grid_map_node src/occupancy_grid_map_node.cpp)

target_include_directories(preprocessing_node PUBLIC ${catkin_INCLUDE_DIRS} ${PCL_INCLUDE_DIRS} ${YAML_CPP_INCLUDE_DIR})
target_link_libraries(preprocessing_node ${catkin_LIBRARIES} ${PCL_LIBRARIES}  preprocessing_library ${YAML_CPP_LIBRARIES})

target_include_directories(occupancy_grid_map_node PUBLIC ${catkin_INCLUDE_DIRS}  ${YAML_CPP_INCLUDE_DIR})
target_link_libraries(occupancy_grid_map_node ${catkin_LIBRARIES}  occupancy_grid_map_library ${YAML_CPP_LIBRARIES})

