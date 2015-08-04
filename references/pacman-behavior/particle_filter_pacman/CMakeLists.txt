cmake_minimum_required(VERSION 2.8.3)
project(particle_filter_pacman)

## Find catkin macros and libraries
find_package(catkin REQUIRED COMPONENTS
  roscpp
  rospy
  std_msgs
  geometry_msgs
  pacman_interface
  pacman_abstract_classes
)

################################################
## Declare ROS messages, services and actions ##
################################################

###################################
## catkin specific configuration ##
###################################

catkin_package(
#  INCLUDE_DIRS include
#  LIBRARIES teste_pacman_map
#  CATKIN_DEPENDS roscpp rospy std_msgs
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
)

## Declare a cpp library
add_library(util_constants
  src/${PROJECT_NAME}/util_constants.cpp
)
add_library(util_functions_particle_filter
  src/${PROJECT_NAME}/util_functions.cpp
)
add_library(game_particle
  src/${PROJECT_NAME}/game_particle.cpp
)
add_library(particle_filter
  src/${PROJECT_NAME}/particle_filter.cpp
)
add_library(kb_behavior_agent
  src/${PROJECT_NAME}/behavior_keyboard_agent.cpp
)
add_library(learning_agent
  src/${PROJECT_NAME}/learning_agent.cpp
)
add_library(q_learning
  src/${PROJECT_NAME}/q_learning.cpp
)
add_library(q_learning_simple
  src/${PROJECT_NAME}/q_learning_simple.cpp
)


## Declare a cpp executable
add_executable(particle_controller src/particle_pacman_controller.cpp)

add_executable(kb_behavior_controller src/kb_behavior_controller.cpp)
add_executable(learning_controller src/learning_controller.cpp)

## Specify libraries to link a library or executable target against
target_link_libraries(particle_filter
  ${catkin_LIBRARIES} game_particle util_constants util_functions_particle_filter
)
target_link_libraries(kb_behavior_agent
  ${catkin_LIBRARIES} pacman_agent util_functions
)
target_link_libraries(learning_agent
  ${catkin_LIBRARIES} pacman_agent util_functions
)
target_link_libraries(q_learning_simple
  ${catkin_LIBRARIES} q_learning util_constants
)

target_link_libraries(particle_controller
  ${catkin_LIBRARIES} particle_filter
)

target_link_libraries(kb_behavior_controller
  ${catkin_LIBRARIES} particle_filter kb_behavior_agent
)

target_link_libraries(learning_controller
  ${catkin_LIBRARIES} particle_filter learning_agent q_learning_simple
)

#############
## Install ##
#############


#############
## Testing ##
#############
