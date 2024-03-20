cmake_minimum_required(VERSION 3.0.2)
project(sensor_model)

find_package(catkin REQUIRED COMPONENTS
  roscpp
  std_msgs
  geometry_msgs
  tf2_ros
  sensor_msgs
  nav_msgs
  pcl_ros
  nodelet
  pluginlib
)

find_package(PCL REQUIRED)
find_package(yaml-cpp REQUIRED)

catkin_package()

include_directories(
  include
  ${catkin_INCLUDE_DIRS}
  ${PCL_INCLUDE_DIRS}
  ${YAML_CPP_INCLUDE_DIR}
  params
)

link_directories(${PCL_LIBRARY_DIRS})
add_definitions(${PCL_DEFINITIONS})

# Libraries for the shared functionalities
add_library(preprocessing_library
  src/preprocessing_library.cpp
)
add_library(occupancy_grid_map_library
  src/occupancy_grid_map_library.cpp
)

# Nodelet libraries
add_library(preprocessing_nodelet
  src/preprocessing_nodelet.cpp
)
add_library(occupancy_grid_map_nodelet
  src/occupancy_grid_map_nodelet.cpp
)

# Link your nodelet library against dependencies
target_link_libraries(preprocessing_nodelet
  ${catkin_LIBRARIES}
  ${PCL_LIBRARIES}
  ${YAML_CPP_LIBRARIES}
  preprocessing_library # Link against the preprocessing library if needed
)

target_link_libraries(occupancy_grid_map_nodelet
  ${catkin_LIBRARIES}
  ${PCL_LIBRARIES}
  ${YAML_CPP_LIBRARIES}
  occupancy_grid_map_library # Link against the occupancy grid map library if needed
)

# Ensure you're linking your shared libraries as well
target_link_libraries(preprocessing_library
  ${catkin_LIBRARIES}
  ${PCL_LIBRARIES}
  ${YAML_CPP_LIBRARIES}
)

target_link_libraries(occupancy_grid_map_library
  ${catkin_LIBRARIES}
  ${PCL_LIBRARIES}
  ${YAML_CPP_LIBRARIES}
)

# Install nodelets
install(TARGETS preprocessing_nodelet occupancy_grid_map_nodelet
  LIBRARY DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
)

# Install plugin description files
install(
  FILES nodelet_plugin.xml
  DESTINATION ${CATKIN_PACKAGE_SHARE_DESTINATION}
)