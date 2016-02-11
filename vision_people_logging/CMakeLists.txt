cmake_minimum_required(VERSION 2.8.3)
project(vision_people_logging)

## Find catkin macros and libraries
## if COMPONENTS list like find_package(catkin REQUIRED COMPONENTS xyz)
## is used, also find other catkin packages
find_package(catkin REQUIRED COMPONENTS
  geometry_msgs
  message_filters
  message_generation
  mongodb_store
  rospy
  tf
  upper_body_detector
)

################################################
## Declare ROS messages, services and actions ##
################################################

add_message_files(
  FILES
    LoggingUBD.msg
)

generate_messages(
  DEPENDENCIES
    geometry_msgs
    sensor_msgs
    upper_body_detector
    std_msgs
)

###################################
## catkin specific configuration ##
###################################
catkin_package(
  CATKIN_DEPENDS
    geometry_msgs
    sensor_msgs
    mongodb_store
    tf
    upper_body_detector
)

#############
## Install ##
#############

install(PROGRAMS
  scripts/save_ubd.py
  DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION}
)

install(DIRECTORY launch
  DESTINATION ${CATKIN_PACKAGE_SHARE_DESTINATION}
)
