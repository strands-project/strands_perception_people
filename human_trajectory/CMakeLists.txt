cmake_minimum_required(VERSION 2.8.3)
project(human_trajectory)

find_package(catkin REQUIRED COMPONENTS
  geometry_msgs
  rospy
  std_msgs
  message_generation
  strands_navigation_msgs
  mongodb_store
  bayes_people_tracker
)

 catkin_python_setup()

################################################
## Declare ROS messages, services and actions ##
################################################
add_message_files(
  FILES
  Trajectory.msg
  Trajectories.msg
)

generate_messages(
  DEPENDENCIES
  geometry_msgs std_msgs
)

###################################
## catkin specific configuration ##
###################################
catkin_package(
  CATKIN_DEPENDS geometry_msgs rospy std_msgs message_runtime mongodb_store strands_navigation_msgs bayes_people_tracker
)

###########
## Build ##
###########

#include_directories(
  #${catkin_INCLUDE_DIRS}
#)

#############
## Install ##
#############

 install(PROGRAMS
   scripts/trajectory_publisher.py
   DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION}
 )

## Mark other files for installation (e.g. launch and bag files, etc.)
# install(FILES
#   # myfile1
#   # myfile2
#   DESTINATION ${CATKIN_PACKAGE_SHARE_DESTINATION}
# )
