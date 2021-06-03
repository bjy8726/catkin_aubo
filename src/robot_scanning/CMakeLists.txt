cmake_minimum_required(VERSION 3.0.2)
project(robot_scanning)

## Compile as C++11, supported in ROS Kinetic and newer
add_compile_options(-std=c++11)


find_package(catkin REQUIRED COMPONENTS
    roscpp
    rospy
    std_msgs
    moveit_core
    moveit_visual_tools
    moveit_ros_planning
    moveit_ros_planning_interface
    pluginlib
    geometric_shapes
    pcl_ros
    pcl_conversions
    rosbag
    tf
    tf2_ros
    #tf2_eigen
    tf2_geometry_msgs
)


catkin_package(
#  INCLUDE_DIRS include
#  LIBRARIES robot_scanning
#  CATKIN_DEPENDS other_catkin_pkg
#  DEPENDS system_lib
)

###########
## Build ##
###########

## Specify additional locations of header files
## Your package locations should be listed before other locations
include_directories(
  include ${catkin_INCLUDE_DIRS}
# include
 ${catkin_INCLUDE_DIRS}
)

add_executable(main_node
  include/robot_scanning/trans_matrix.h
  src/trans_matrix.cpp
  src/run_track_points.cpp
  #include/robot_scanning/run_track_points.h 
  src/main_node.cpp)


## Specify libraries to link a library or executable target against
target_link_libraries(main_node ${catkin_LIBRARIES} ${Boost_LIBRARIES})

