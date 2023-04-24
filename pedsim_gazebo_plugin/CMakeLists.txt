cmake_minimum_required(VERSION 2.8.3)
project(pedsim_gazebo_plugin)

## Compile as C++11, supported in ROS Kinetic and newer
 add_compile_options(-std=c++11)

## Find catkin macros and libraries
## if COMPONENTS list like find_package(catkin REQUIRED COMPONENTS xyz)
## is used, also find other catkin packages
find_package(catkin REQUIRED COMPONENTS
  gazebo_ros
  geometry_msgs
  roscpp
  rospy
  gazebo_msgs
  pedsim_msgs
)

## System dependencies are found with CMake's conventions
# find_package(Boost REQUIRED COMPONENTS system)
 find_package(gazebo REQUIRED )
 link_directories(${GAZEBO_LIBRARY_DIRS})

#find_package(ignition-cmake2 REQUIRED)
#find_package(ignition-plugin1 REQUIRED COMPONENTS register)
#set(IGN_PLUGIN_VER ${ignition-plugin1_VERSION_MAJOR})
#
#find_package(ignition-gazebo3 REQUIRED)
#add_library(IgnActorPosesPlugin SHARED src/ign_actor_poses_plugin.cpp)
#set_property(TARGET IgnActorPosesPlugin PROPERTY CXX_STANDARD 17)
#target_link_libraries(IgnActorPosesPlugin
#  PRIVATE ignition-plugin${IGN_PLUGIN_VER}::ignition-plugin${IGN_PLUGIN_VER}
#  PRIVATE ignition-gazebo3::ignition-gazebo3)
#add_dependencies(IgnActorPosesPlugin ${catkin_EXPORTED_TARGETS})
################################################
## Declare ROS messages, services and actions ##
################################################



################################################
## Declare ROS dynamic reconfigure parameters ##
################################################


###################################
## catkin specific configuration ##
###################################

catkin_package(
#  INCLUDE_DIRS include
#  LIBRARIES gzb_vel_plugin
  CATKIN_DEPENDS gazebo_ros roscpp  geometry_msgs pedsim_msgs
#  DEPENDS system_lib
)

###########
## Build ##
###########

## Specify additional locations of header files
## Your package locations should be listed before other locations
include_directories(
  include
  ${catkin_INCLUDE_DIRS}
  ${GAZEBO_INCLUDE_DIRS}
)


add_library(ActorPosesPlugin src/actor_poses_plugin.cpp)
target_link_libraries(ActorPosesPlugin ${catkin_LIBRARIES} ${GAZEBO_LIBRARIES})
add_dependencies(ActorPosesPlugin ${catkin_EXPORTED_TARGETS})

#############
## Install ##
#############

