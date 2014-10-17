cmake_minimum_required(VERSION 2.8.3)
project(detector_msg_to_pose_array)

find_package(catkin REQUIRED COMPONENTS
  genpy
  geometry_msgs
  roslib
  rospy
  rostopic
)

###################################
## catkin specific configuration ##
###################################
catkin_package(
  CATKIN_DEPENDS genpy geometry_msgs roslib rospy rostopic
)

#############
## Install ##
#############

install(PROGRAMS
  scripts/to_pose_array.py
  DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION}
)

install(DIRECTORY launch
  DESTINATION ${CATKIN_PACKAGE_SHARE_DESTINATION}
)

install(DIRECTORY config 
  DESTINATION ${CATKIN_PACKAGE_SHARE_DESTINATION}
)
