cmake_minimum_required(VERSION 2.8.3)
project(pacman_abstract_classes)

## Find catkin macros and libraries
## if COMPONENTS list like find_package(catkin REQUIRED COMPONENTS xyz)
## is used, also find other catkin packages
find_package(catkin REQUIRED COMPONENTS
  geometry_msgs
  pacman_interface
  roscpp
  rospy
  std_msgs
)

## System dependencies are found with CMake's conventions
# find_package(Boost REQUIRED COMPONENTS system)

################################################
## Declare ROS messages, services and actions ##
################################################


###################################
## catkin specific configuration ##
###################################
## The catkin_package macro generates cmake config files for your package
## Declare things to be passed to dependent projects
## INCLUDE_DIRS: uncomment this if you package contains header files
## LIBRARIES: libraries you create in this project that dependent projects also need
## CATKIN_DEPENDS: catkin_packages dependent projects also need
## DEPENDS: system dependencies of this project that dependent projects also need
catkin_package(
  INCLUDE_DIRS include
  LIBRARIES pacman_agent agent
  CATKIN_DEPENDS geometry_msgs pacman_interface roscpp rospy std_msgs
  DEPENDS system_lib
)

###########
## Build ##
###########

include_directories(
  include
  ${catkin_INCLUDE_DIRS}
)

## Declare a cpp library
add_library(util_functions
  src/${PROJECT_NAME}/util_functions.cpp
)
add_library(agent
  src/${PROJECT_NAME}/agent.cpp
)
add_library(pacman_agent
  src/${PROJECT_NAME}/pacman_agent.cpp
)

## Declare a cpp executable
# add_executable(pacman_abstract_classes_node src/pacman_abstract_classes_node.cpp)

## Add cmake target dependencies of the executable/library
## as an example, message headers may need to be generated before nodes
# add_dependencies(pacman_abstract_classes_node pacman_abstract_classes_generate_messages_cpp)

## Specify libraries to link a library or executable target against
target_link_libraries(agent
  ${catkin_LIBRARIES} util_functions
)