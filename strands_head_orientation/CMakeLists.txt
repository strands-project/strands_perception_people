cmake_minimum_required(VERSION 2.8.3)
project(strands_head_orientation)

## Find catkin macros and libraries
## if COMPONENTS list like find_package(catkin REQUIRED COMPONENTS xyz)
## is used, also find other catkin packages
find_package(catkin REQUIRED COMPONENTS
    mongodb_store   # NEEDS to come early, else the linker will insult you.
    cv_bridge
    geometry_msgs
    image_transport
    message_filters
    roscpp
    std_msgs
    message_generation
    opencv_warco
    upper_body_detector
)

## System dependencies are found with CMake's conventions
# find_package(Boost REQUIRED COMPONENTS system)


## Uncomment this if the package has a setup.py. This macro ensures
## modules and global scripts declared therein get installed
## See http://ros.org/doc/groovy/api/catkin/html/user_guide/setup_dot_py.html
# catkin_python_setup()

#######################################
## Declare ROS messages and services ##
#######################################

## Generate messages in the 'msg' folder
add_message_files(
  FILES
  HeadOrientations.msg
)

# Generate services in the 'srv' folder
add_service_files(
  FILES
  StartHeadAnalysis.srv
  StopHeadAnalysis.srv
  IsHeadAnalysisRunning.srv
)

## Generate added messages and services with any dependencies listed here
generate_messages(
  DEPENDENCIES
  std_msgs
  geometry_msgs
)

###################################
## catkin specific configuration ##
###################################
## The catkin_package macro generates cmake config files for your package
## Declare things to be passed to dependent projects
## LIBRARIES: libraries you create in this project that dependent projects also need
## CATKIN_DEPENDS: catkin_packages dependent projects also need
## DEPENDS: system dependencies of this project that dependent projects also need
catkin_package(
   #INCLUDE_DIRS include
#  LIBRARIES strands_head_orientation
#  CATKIN_DEPENDS geometry_msgs roscpp std_msgs 
#  DEPENDS system_lib
)

###########
## Build ##
###########

# We know that for STRANDS it's g++4.6 anyways.
add_definitions(-std=c++0x)

## Specify additional locations of header files
## Your package locations should be listed before other locations
include_directories(
  ${catkin_INCLUDE_DIRS}
)

## Declare a cpp library
# add_library(strands_head_orientation
#   src/${PROJECT_NAME}/strands_head_orientation.cpp
# )

## Declare a cpp executable
add_executable(strands_head_orientation src/main.cpp)
add_executable(store_detections src/store_detections.cpp)
add_executable(start src/start.cpp)
add_executable(stop src/stop.cpp)
add_executable(status src/status.cpp)

## Add cmake target dependencies of the executable/library
## as an example, message headers may need to be generated before nodes
add_dependencies(strands_head_orientation ${PROJECT_NAME}_generate_messages_cpp ${catkin_EXPORTED_TARGETS})
add_dependencies(store_detections ${PROJECT_NAME}_generate_messages_cpp ${catkin_EXPORTED_TARGETS})
add_dependencies(start ${PROJECT_NAME}_generate_messages_cpp ${catkin_EXPORTED_TARGETS})
add_dependencies(stop ${PROJECT_NAME}_generate_messages_cpp ${catkin_EXPORTED_TARGETS})
add_dependencies(status ${PROJECT_NAME}_generate_messages_cpp ${catkin_EXPORTED_TARGETS})

## Specify libraries to link a library or executable target against
target_link_libraries(strands_head_orientation ${catkin_LIBRARIES})
target_link_libraries(store_detections ${catkin_LIBRARIES})
target_link_libraries(start ${catkin_LIBRARIES})
target_link_libraries(stop ${catkin_LIBRARIES})
target_link_libraries(status ${catkin_LIBRARIES})

#############
## Install ##
#############

# all install targets should use catkin DESTINATION variables
# See http://ros.org/doc/groovy/api/catkin/html/adv_user_guide/variables.html

## Mark executable scripts (Python etc.) for installation
## in contrast to setup.py, you can choose the destination
# install(PROGRAMS
#   scripts/my_python_script
#   DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION}
# )

## Mark executables and/or libraries for installation
install(TARGETS strands_head_orientation store_detections start stop status
  ARCHIVE DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
  LIBRARY DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
  RUNTIME DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION}
)

install(DIRECTORY launch
  DESTINATION ${CATKIN_PACKAGE_SHARE_DESTINATION}
)

install(DIRECTORY models 
  DESTINATION ${CATKIN_PACKAGE_SHARE_DESTINATION}
)
