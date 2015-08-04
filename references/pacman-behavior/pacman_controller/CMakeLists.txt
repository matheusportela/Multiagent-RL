cmake_minimum_required(VERSION 2.8.3)
project(pacman_controller)

find_package(catkin REQUIRED COMPONENTS
  rospy
  roscpp
  std_msgs
  pacman_interface
  pacman_abstract_classes
)

## System dependencies are found with CMake's conventions
find_package(Boost REQUIRED COMPONENTS system)

################################################
## Declare ROS messages, services and actions ##
################################################

###################################
## catkin specific configuration ##
###################################
catkin_package(
  INCLUDE_DIRS include
  LIBRARIES game_info
  CATKIN_DEPENDS rospy roscpp std_msgs pacman_interface
)

###########
## Build ##
###########

include_directories(
  include
  ${catkin_INCLUDE_DIRS}
)

## Declare a cpp executable
add_executable(keyboard_cpp_controller src/keyboardCppAgent.cpp)
#add_executable(new_keyboard_cpp_controller src/new_keyboard_agent.cpp src/agent.cpp src/pacman_agent.cpp)

# pacman world usefull functions
#add_library(util_functions src/util_functions.cpp)
add_library(game_info src/game_info.cpp)

# pacman agents
add_library(behavior_keyboard_agent src/behavior_keyboard_agent.cpp src/pacman_agent.cpp src/agent.cpp)

add_executable(moduled_keyboard_cpp_controller src/pacman_controller.cpp
      src/agent.cpp src/pacman_agent.cpp src/keyboard_agent.cpp src/ghost_agent.cpp src/visible_ghost_agent.cpp)

add_executable(behavior_keyboard_controller src/behavior_pacman_controller.cpp
      src/agent.cpp src/pacman_agent.cpp src/ghost_agent.cpp src/visible_ghost_agent.cpp)

# add_dependencies(keyboard_cpp_controller pacman_controller_generate_messages_cpp)

## Specify libraries to link a library or executable target against
target_link_libraries(keyboard_cpp_controller
  ${catkin_LIBRARIES}
)

#target_link_libraries(new_keyboard_cpp_controller
#  ${catkin_LIBRARIES}
#)
target_link_libraries(game_info
  ${catkin_LIBRARIES} util_functions
)

target_link_libraries(moduled_keyboard_cpp_controller
  ${catkin_LIBRARIES} util_functions game_info
)

target_link_libraries(behavior_keyboard_controller
  ${catkin_LIBRARIES} util_functions game_info behavior_keyboard_agent
)

#############
## Install ##
#############

#############
## Testing ##
#############

