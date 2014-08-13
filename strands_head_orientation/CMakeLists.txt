cmake_minimum_required(VERSION 2.8.3)
project(strands_head_orientation)

## Find catkin macros and libraries
## if COMPONENTS list like find_package(catkin REQUIRED COMPONENTS xyz)
## is used, also find other catkin packages
find_package(catkin REQUIRED COMPONENTS
    cv_bridge
    geometry_msgs
    image_transport
    message_filters
    roscpp
    std_msgs
    strands_perception_people_msgs
    mongodb_store_cpp_client
    mongodb_store_msgs
    mongodb_store
    opencv_warco
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
# add_message_files(
#   FILES
#   Message1.msg
#   Message2.msg
# )

## Generate services in the 'srv' folder
# add_service_files(
#   FILES
#   Service1.srv
#   Service2.srv
# )

## Generate added messages and services with any dependencies listed here
# generate_messages(
#   DEPENDENCIES
#   geometry_msgs#   std_msgs#   strands_perception_people_msgs
# )

###################################
## catkin specific configuration ##
###################################
## The catkin_package macro generates cmake config files for your package
## Declare things to be passed to dependent projects
## LIBRARIES: libraries you create in this project that dependent projects also need
## CATKIN_DEPENDS: catkin_packages dependent projects also need
## DEPENDS: system dependencies of this project that dependent projects also need
catkin_package(
   INCLUDE_DIRS include
#  LIBRARIES strands_head_orientation
#  CATKIN_DEPENDS geometry_msgs roscpp std_msgs strands_perception_people_msgs
#  DEPENDS system_lib
)

###########
## Build ##
###########

# We know that for STRANDS it's g++4.6 anyways.
add_definitions(-std=c++0x)

## Specify additional locations of header files
## Your package locations should be listed before other locations
include_directories(include
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
add_dependencies(strands_head_orientation strands_perception_people_msgs_generate_messages_cpp)
add_dependencies(store_detections strands_perception_people_msgs_generate_messages_cpp mongodb_store_generate_messages_cpp)
add_dependencies(start strands_perception_people_msgs_generate_messages_cpp)
add_dependencies(stop strands_perception_people_msgs_generate_messages_cpp)
add_dependencies(status strands_perception_people_msgs_generate_messages_cpp)

## Specify libraries to link a library or executable target against
target_link_libraries(strands_head_orientation ${catkin_LIBRARIES})
target_link_libraries(store_detections mongoclient ${catkin_LIBRARIES})
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
# install(TARGETS strands_head_orientation strands_head_orientation_node
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
# catkin_add_gtest(${PROJECT_NAME}-test test/test_strands_head_orientation.cpp)
# if(TARGET ${PROJECT_NAME}-test)
#   target_link_libraries(${PROJECT_NAME}-test ${PROJECT_NAME})
# endif()

## Add folders to be run by python nosetests
# catkin_add_nosetests(test)
