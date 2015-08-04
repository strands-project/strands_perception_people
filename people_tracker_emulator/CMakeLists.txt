cmake_minimum_required(VERSION 2.8.3)
project(people_tracker_emulator)

## Find catkin macros and libraries
## if COMPONENTS list like find_package(catkin REQUIRED COMPONENTS xyz)
## is used, also find other catkin packages
find_package(catkin REQUIRED)

catkin_python_setup()

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
  CATKIN_DEPENDS bayes_people_tracker geometry_msgs people_msgs rospy std_msgs tf visualization_msgs
)

#############
## Install ##
#############

install(PROGRAMS
  scripts/posestamped_to_ppl_tracker_msgs.py
  DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION}
)