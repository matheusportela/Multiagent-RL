cmake_minimum_required(VERSION 2.8.3)
project(pacman_msgs)

## Find catkin macros and libraries
## if COMPONENTS list like find_package(catkin REQUIRED COMPONENTS xyz)
## is used, also find other catkin packages
find_package(catkin REQUIRED COMPONENTS
  geometry_msgs
  std_msgs
  message_generation
)

################################################
## Declare ROS messages, services and actions ##
################################################

## Generate messages in the 'msg' folder
add_message_files(
  FILES
  PacmanAction.msg
  AgentAction.msg
  AgentPose.msg
  MapLayout.msg
)

## Generate services in the 'srv' folder
add_service_files(
  FILES
  PacmanGetAction.srv
  PacmanInitializationInfo.srv
  PacmanMapInfo.srv
  StartGame.srv
  EndGame.srv
  RewardService.srv
  AgentPoseService.srv
)

## Generate added messages and services with any dependencies listed here
generate_messages(
  DEPENDENCIES
  std_msgs
  geometry_msgs
)

###################################
## catkin specific configuration ##
###################################

catkin_package(
#  INCLUDE_DIRS include
#  LIBRARIES pacman_msgs
  CATKIN_DEPENDS geometry_msgs std_msgs message_runtime
#  DEPENDS system_lib
)

###########
## Build ##
###########


#############
## Install ##
#############


#############
## Testing ##
#############
