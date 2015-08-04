cmake_minimum_required(VERSION 2.8.3)
project(bayesian_q_learning)

set(ROS_BUILD_TYPE Debug)

## Find catkin macros and libraries
## if COMPONENTS list like find_package(catkin REQUIRED COMPONENTS xyz)
## is used, also find other catkin packages
find_package(catkin REQUIRED COMPONENTS
  geometry_msgs
  pacman_abstract_classes
  pacman_msgs
  q_learning_pacman
  roscpp
  std_msgs
)

################################################
## Declare ROS messages, services and actions ##
################################################


###################################
## catkin specific configuration ##
###################################
## The catkin_package macro generates cmake config files for your package
## Declare things to be passed to dependent projects
catkin_package(
#  INCLUDE_DIRS include
#  LIBRARIES bayesian_q_learning
#  CATKIN_DEPENDS geometry_msgs pacman_abstract_classes pacman_msgs q_learning_pacman roscpp std_msgs
#  DEPENDS system_lib
)

###########
## Build ##
###########

## Specify additional locations of header files
include_directories(
  include
  ${catkin_INCLUDE_DIRS}
)

## Declare a cpp library
add_library(bayesian_q_learning_game_state
  src/${PROJECT_NAME}/bayesian_game_state.cpp
)
add_library(bayesian_behavior_agent
  src/${PROJECT_NAME}/bayesian_behavior_agent.cpp
)
add_library(bayesian_q_learning
  src/${PROJECT_NAME}/bayesian_q_learning.cpp
)

## Declare a cpp executable
add_executable(bayesian_q_learning_node src/bayesian_q_controller.cpp)

add_executable(test_node src/test.cpp)

## Specify libraries to link a library or executable target against
target_link_libraries(bayesian_q_learning_game_state
  ${catkin_LIBRARIES} game_state util_functions
)
target_link_libraries(bayesian_behavior_agent
  ${catkin_LIBRARIES} bayesian_q_learning_game_state pacman_agent util_functions
)
target_link_libraries(bayesian_q_learning
  ${catkin_LIBRARIES} bayesian_q_learning_game_state bayesian_behavior_agent util_functions
)

target_link_libraries(bayesian_q_learning_node
  ${catkin_LIBRARIES} bayesian_q_learning_game_state bayesian_behavior_agent bayesian_q_learning
)
target_link_libraries(test_node
  ${catkin_LIBRARIES} 
)

#############
## Install ##
#############


#############
## Testing ##
#############

