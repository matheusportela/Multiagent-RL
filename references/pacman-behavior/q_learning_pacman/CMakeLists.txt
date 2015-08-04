cmake_minimum_required(VERSION 2.8.3)
project(q_learning_pacman)

## Find catkin macros and libraries
## if COMPONENTS list like find_package(catkin REQUIRED COMPONENTS xyz)
## is used, also find other catkin packages
find_package(catkin REQUIRED COMPONENTS
  geometry_msgs
  pacman_abstract_classes
  pacman_msgs
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
## INCLUDE_DIRS: uncomment this if you package contains header files
## LIBRARIES: libraries you create in this project that dependent projects also need
## CATKIN_DEPENDS: catkin_packages dependent projects also need
## DEPENDS: system dependencies of this project that dependent projects also need
catkin_package(
  INCLUDE_DIRS include
  LIBRARIES game_state
  CATKIN_DEPENDS pacman_msgs
  DEPENDS system_lib
)

###########
## Build ##
###########

## Specify additional locations of header files
## Your package locations should be listed before other locations
include_directories(
  include
  ${catkin_INCLUDE_DIRS}
)

## Declare a cpp library
add_library(game_state
  src/${PROJECT_NAME}/game_state.cpp
)
add_library(bayesian_game_state
  src/${PROJECT_NAME}/bayesian_game_state.cpp
)
add_library(behavior_agent
  src/${PROJECT_NAME}/behavior_agent.cpp
)
add_library(q_learning_lib
  src/${PROJECT_NAME}/q_learning.cpp
)

## Declare a cpp executable
add_executable(q_learning_node src/q_learning_controller.cpp)

## Add cmake target dependencies of the executable/library
## as an example, message headers may need to be generated before nodes
# add_dependencies(q_learning_pacman_node q_learning_pacman_generate_messages_cpp)

## Specify libraries to link a library or executable target against
target_link_libraries(bayesian_game_state
  ${catkin_LIBRARIES} game_state util_functions
)
target_link_libraries(q_learning_lib
  ${catkin_LIBRARIES} bayesian_game_state util_functions
)

target_link_libraries(q_learning_node
  ${catkin_LIBRARIES} bayesian_game_state behavior_agent q_learning_lib
)

#############
## Install ##
#############


#############
## Testing ##
#############

