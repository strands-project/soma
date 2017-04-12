cmake_minimum_required(VERSION 2.8.3)
project(soma_trajectory)

find_package(catkin REQUIRED COMPONENTS
  human_trajectory
  rospy
  message_generation
  geometry_msgs
  interactive_markers
  visualization_msgs
  soma_msgs
  soma_map_manager
  mongodb_store
  std_msgs
  nav_msgs
)

catkin_python_setup()

################################################
## Declare ROS messages, services and actions ##
################################################

add_service_files(
   FILES
   SOMAQueryTrajectories.srv
)

generate_messages(
   DEPENDENCIES
   std_msgs
   soma_msgs
)

###################################
## catkin specific configuration ##
###################################

catkin_package(
#  INCLUDE_DIRS include
#  LIBRARIES soma_trajectory
  CATKIN_DEPENDS human_trajectory rospy soma_msgs geometry_msgs soma_map_manager mongodb_store message_runtime interactive_markers visualization_msgs std_msgs nav_msgs
#  DEPENDS system_lib
)

###########
## Build ##
###########

include_directories(
  ${catkin_INCLUDE_DIRS}
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
# install(TARGETS soma_trajectory soma_trajectory_node
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
