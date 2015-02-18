cmake_minimum_required(VERSION 2.8.3)
project(bayes_people_tracker_logging)

## Find catkin macros and libraries
## if COMPONENTS list like find_package(catkin REQUIRED COMPONENTS xyz)
## is used, also find other catkin packages
find_package(catkin REQUIRED COMPONENTS
  bayes_people_tracker
  geometry_msgs
  message_filters
  message_generation
  mongodb_store
  rospy
  mdl_people_tracker
  tf
  upper_body_detector
)

################################################
## Declare ROS messages, services and actions ##
################################################

add_message_files(
    FILES
    Logging.msg
)

generate_messages(
    DEPENDENCIES
    bayes_people_tracker
    geometry_msgs
    upper_body_detector
    std_msgs
    mdl_people_tracker
)

###################################
## catkin specific configuration ##
###################################
catkin_package(
  CATKIN_DEPENDS 
    bayes_people_tracker
    geometry_msgs
    mongodb_store
    mdl_people_tracker
    tf
    upper_body_detector
)

#############
## Install ##
#############

install(PROGRAMS
  scripts/save_locations.py
  DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION}
)

install(DIRECTORY launch
  DESTINATION ${CATKIN_PACKAGE_SHARE_DESTINATION}
)
