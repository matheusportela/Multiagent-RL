cmake_minimum_required(VERSION 2.8.3)
project(deterministic_q_learning)

## Find catkin macros and libraries
## if COMPONENTS list like find_package(catkin REQUIRED COMPONENTS xyz)
## is used, also find other catkin packages
find_package(catkin REQUIRED COMPONENTS
  geometry_msgs
  pacman_abstract_classes
  pacman_msgs
  roscpp
  std_msgs
  q_learning_pacman
)

## System dependencies are found with CMake's conventions
# find_package(Boost REQUIRED COMPONENTS system)


## Uncomment this if the package has a setup.py. This macro ensures
## modules and global scripts declared therein get installed
## See http://ros.org/doc/api/catkin/html/user_guide/setup_dot_py.html
# catkin_python_setup()

################################################
## Declare ROS messages, services and actions ##
################################################

###################################
## catkin specific configuration ##
###################################

catkin_package(
#  INCLUDE_DIRS include
#  LIBRARIES deterministic_q_learning
#  CATKIN_DEPENDS geometry_msgs pacman_abstract_classes pacman_msgs roscpp std_msgs
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
add_library(deterministic_game_state
  src/${PROJECT_NAME}/deterministic_game_state.cpp
)
add_library(deterministic_q_learning
  src/${PROJECT_NAME}/deterministic_q_learning.cpp
)
add_library(deterministic_behavior_agent
  src/${PROJECT_NAME}/deterministic_behavior_agent.cpp
)

## Declare a cpp executable
add_executable(deterministic_q_node src/deterministic_q_controller.cpp)

## Add cmake target dependencies of the executable/library
## as an example, message headers may need to be generated before nodes
# add_dependencies(deterministic_q_learning_node deterministic_q_learning_generate_messages_cpp)

## Specify libraries to link a library or executable target against
target_link_libraries(deterministic_game_state
  ${catkin_LIBRARIES} game_state util_functions
)
target_link_libraries(deterministic_q_learning
  ${catkin_LIBRARIES} deterministic_game_state util_functions
)
target_link_libraries(deterministic_behavior_agent
  ${catkin_LIBRARIES} deterministic_game_state pacman_agent util_functions
)

target_link_libraries(deterministic_q_node
  ${catkin_LIBRARIES} deterministic_game_state deterministic_behavior_agent deterministic_q_learning
)

#############
## Install ##
#############

# all install targets should use catkin DESTINATION variables
# See http://ros.org/doc/api/catkin/html/adv_user_guide/variables.html

## Mark executable scripts (Python etc.) for installation
## in contrast to setup.py, you can choose the destination
# install(PROGRAMS
#   scripts/my_python_script
#   DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION}
# )

## Mark executables and/or libraries for installation
# install(TARGETS deterministic_q_learning deterministic_q_learning_node
#   ARCHIVE DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
#   LIBRARY DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
#   RUNTIME DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION}
# )

## Mark cpp header files for installation
# install(DIRECTORY include/${PROJECT_NAME}/
#   DESTINATION ${CATKIN_PACKAGE_INCLUDE_DESTINATION}
#   FILES_MATCHING PATTERN "*.h"
#   PATTERN ".svn" EXCLUDE
# )

## Mark other files for installation (e.g. launch and bag files, etc.)
# install(FILES
#   # myfile1
#   # myfile2
#   DESTINATION ${CATKIN_PACKAGE_SHARE_DESTINATION}
# )

#############
## Testing ##
#############

## Add gtest based cpp test target and link libraries
# catkin_add_gtest(${PROJECT_NAME}-test test/test_deterministic_q_learning.cpp)
# if(TARGET ${PROJECT_NAME}-test)
#   target_link_libraries(${PROJECT_NAME}-test ${PROJECT_NAME})
# endif()

## Add folders to be run by python nosetests
# catkin_add_nosetests(test)
