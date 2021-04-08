cmake_minimum_required(VERSION 3.0.2)
project(kamayuc_planning)

find_package(catkin REQUIRED COMPONENTS
  roscpp
  tf2_ros
  pcl_ros
  pcl_conversions
  sensor_msgs
  geometry_msgs
  tf2_sensor_msgs
  tf2_geometry_msgs
  rospy
  move_base
)

## System dependencies are found with CMake's conventions
# find_package(Boost REQUIRED COMPONENTS system)
find_package(PCL 1.8.1 REQUIRED)

if (PCL_VERSION VERSION_LESS "1.9.1")
  message("Compiled with [-std = c++11]")
  add_compile_options(-std=c++11)
else()
  message("Compiled with [-std = c++14]")
  add_compile_options(-std=c++14)
endif()

link_directories(${PCL_LIBRARY_DIRS})
add_definitions(${PCL_DEFINITIONS})

###################################
## catkin specific configuration ##
###################################
## The catkin_package macro generates cmake config files for your package
## Declare things to be passed to dependent projects
## INCLUDE_DIRS: uncomment this if your package contains header files
## LIBRARIES: libraries you create in this project that dependent projects also need
## CATKIN_DEPENDS: catkin_packages dependent projects also need
## DEPENDS: system dependencies of this project that dependent projects also need
catkin_package(
#  INCLUDE_DIRS include
#  LIBRARIES planners_middleware
#  CATKIN_DEPENDS roscpp rospy std_msgs
#  DEPENDS system_lib
)

###########
## Build ##
###########

## Specify additional locations of header files
## Your package locations should be listed before other locations
include_directories(
  ${catkin_INCLUDE_DIRS}
)

## Declare a C++ executable

add_executable(base_link_flattener  src/base_link_costmap_projector.cpp)
add_executable(point_cloud_filter  src/pc_filter.cpp)
add_executable(point_cloud_zfilter  src/pc_zfilter.cpp)
add_executable(point_cloud_slope  src/pc_slope.cpp)

## Specify libraries to link a library or executable target against

target_link_libraries(base_link_flattener  ${catkin_LIBRARIES} ${PCL_LIBRARIES})
target_link_libraries(point_cloud_filter  ${catkin_LIBRARIES} ${PCL_LIBRARIES})
target_link_libraries(point_cloud_zfilter  ${catkin_LIBRARIES} ${PCL_LIBRARIES})
target_link_libraries(point_cloud_slope  ${catkin_LIBRARIES} ${PCL_LIBRARIES})  