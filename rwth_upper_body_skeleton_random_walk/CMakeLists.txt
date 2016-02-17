cmake_minimum_required(VERSION 2.8.3)
project(rwth_upper_body_skeleton_random_walk)

find_package(catkin REQUIRED COMPONENTS
  cv_bridge
  image_transport
  message_filters
  roscpp
  upper_body_detector
  sensor_msgs
  std_msgs
  message_generation
)

find_package(Qt4 REQUIRED COMPONENTS QtCore QtGui)
include(${QT_USE_FILE})
find_package(OpenCV)
find_package(OpenMP)
if (OPENMP_FOUND)
    set (CMAKE_C_FLAGS "${CMAKE_C_FLAGS} ${OpenMP_C_FLAGS}")
    set (CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} ${OpenMP_CXX_FLAGS}")
endif()

## Generate messages in the 'msg' folder
 add_message_files(
   FILES
   SkeletonDetector.msg
 )

## Generate services in the 'srv' folder
# add_service_files(
#   FILES
#   Service1.srv
#   Service2.srv
# )

## Generate actions in the 'action' folder
# add_action_files(
#   FILES
#   Action1.action
#   Action2.action
# )

## Generate added messages and services with any dependencies listed here
 generate_messages(
   DEPENDENCIES
  std_msgs
 )

###################################
## catkin specific configuration ##
###################################
## The catkin_package macro generates cmake config files for your package
## Declare things to be passed to dependent projects
## INCLUDE_DIRS: uncomment this if you package contains header files
## LIBRARIES: libraries you create in this project that dependent projects also need
## CATKIN_DEPENDS: catkin_packages dependent projects also need
## DEPENDS: system dependencies of this project that dependent projects also need

catkin_package(CATKIN_DEPENDS upper_body_detector message_runtime)


###########
## Build ##
###########

## Specify additional locations of header files
## Your package locations should be listed before other locations
# include_directories(include)
include_directories(include ${catkin_INCLUDE_DIRS} ${OpenCV_INCLUDE_DIRS} ${OpenMP_INCLUDE_DIRS})

## Declare a cpp library
# add_library(upper_body_skeleton_client
#   src/${PROJECT_NAME}/upper_body_skeleton_client.cpp
# )

## Declare a cpp executable
 add_executable(rwth_upper_body_skeleton_random_walk_node src/upper_body_skeleton_random_walk_node.cpp src/body_pose_regressor_utilities.cpp src/forest_utilities.cpp src/mean_shift_utilities.cpp src/tree_utilities.cpp)


add_dependencies(rwth_upper_body_skeleton_random_walk_node ${PROJECT_NAME}_generate_messages_cpp ${catkin_EXPORTED_TARGETS})

## Add cmake target dependencies of the executable/library
## as an example, message headers may need to be generated before nodes
 
## Specify libraries to link a library or executable target against
 target_link_libraries(rwth_upper_body_skeleton_random_walk_node
   ${catkin_LIBRARIES}
   ${OpenCV_LIBRARIES}
   ${QT_LIBRARIES}
   ${QT_QTCORE_LIBRARY}
   ${QT_QTGUI_LIBRARY}
 )

add_dependencies(rwth_upper_body_skeleton_random_walk_node rwth_upper_body_skeleton_random_walk_gencpp)


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
 install(TARGETS rwth_upper_body_skeleton_random_walk_node
   ARCHIVE DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
   LIBRARY DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
   RUNTIME DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION}
 )

## Mark cpp header files for installation
 install(DIRECTORY launch
   DESTINATION ${CATKIN_PACKAGE_SHARE_DESTINATION})

install(DIRECTORY Model 
  DESTINATION ${CATKIN_PACKAGE_SHARE_DESTINATION}
)
