cmake_minimum_required(VERSION 2.8.3)
project(opencv_warco)

## Find catkin macros and libraries
## if COMPONENTS list like find_package(catkin REQUIRED COMPONENTS xyz)
## is used, also find other catkin packages
find_package(catkin REQUIRED)

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
#   std_msgs  # Or other packages containing msgs
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
  LIBRARIES ${PROJECT_NAME}
  #CATKIN_DEPENDS other_catkin_pkg
  DEPENDS opencv
)

###########
## Build ##
###########

find_package(OpenCV REQUIRED core imgproc highgui)
if(NOT OPENCV_CORE_FOUND OR NOT OPENCV_HIGHGUI_FOUND OR NOT OPENCV_IMGPROC_FOUND)
    message(FATAL_ERROR "Couldn't find OpenCV!")
endif()
include_directories(${OpenCV_INCLUDE_DIRS})

FIND_PACKAGE(OpenMP)
if(OPENMP_FOUND)
    set(CMAKE_C_FLAGS "${CMAKE_C_FLAGS} ${OpenMP_C_FLAGS}")
    set(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} ${OpenMP_CXX_FLAGS}")
endif()

## Specify additional locations of header files
## Your package locations should be listed before other locations
include_directories(include ${catkin_INCLUDE_DIRS} ${OpenCV_INCLUDE_DIRS})

# We know that for STRANDS it's g++4.6 anyways.
add_definitions(-std=c++0x)

# For the dense libsvm
add_definitions(-D_DENSE_REP)

## Declare a cpp library
add_library(${PROJECT_NAME}
  src/opencv_warco/cvutils.cpp
  src/opencv_warco/covcorr.cpp
  src/opencv_warco/dists.cpp
  src/opencv_warco/features.cpp
  src/opencv_warco/filterbank.cpp
  src/opencv_warco/model.cpp
  src/opencv_warco/warco.cpp
  src/opencv_warco/libsvm/svm.cpp
)

set(COMMON_SRC
    src/opencv_warco/mainutils.cpp
    src/opencv_warco/jsoncpp.cpp
)

target_link_libraries(${PROJECT_NAME} ${OpenCV_LIBS})

## Declare a cpp executable
add_executable(opencv-warco-train src/opencv_warco/train.cpp ${COMMON_SRC})
add_executable(opencv-warco-pred src/opencv_warco/pred.cpp ${COMMON_SRC})
add_executable(opencv-warco-traintest src/opencv_warco/traintest.cpp ${COMMON_SRC})
add_executable(opencv-warco-utest src/opencv_warco/utest.cpp ${COMMON_SRC})
add_executable(opencv-warco-dists src/opencv_warco/save_dists.cpp ${COMMON_SRC})
add_executable(opencv-warco-covs src/opencv_warco/save_covs.cpp ${COMMON_SRC})

target_link_libraries(opencv-warco-train ${PROJECT_NAME} ${OpenCV_LIBS})
target_link_libraries(opencv-warco-pred ${PROJECT_NAME} ${OpenCV_LIBS})
target_link_libraries(opencv-warco-traintest ${PROJECT_NAME} ${OpenCV_LIBS})
target_link_libraries(opencv-warco-utest ${PROJECT_NAME} ${OpenCV_LIBS})
target_link_libraries(opencv-warco-dists ${PROJECT_NAME} ${OpenCV_LIBS})
target_link_libraries(opencv-warco-covs ${PROJECT_NAME} ${OpenCV_LIBS})

## Add cmake target dependencies of the executable/library
## as an example, message headers may need to be generated before nodes
# add_dependencies(opencv-warco_node opencv-warco_generate_messages_cpp)

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
install(TARGETS 
    ${PROJECT_NAME} 
    opencv-warco-train 
    opencv-warco-pred
    opencv-warco-traintest
    opencv-warco-utest
    opencv-warco-dists
    opencv-warco-covs
  ARCHIVE DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
  LIBRARY DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
  RUNTIME DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION}
)

## Mark cpp header files for installation
install(DIRECTORY include/${PROJECT_NAME}/
  DESTINATION ${CATKIN_PACKAGE_INCLUDE_DESTINATION}
  FILES_MATCHING PATTERN "*.h*"
)

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
# catkin_add_gtest(${PROJECT_NAME}-test test/test_opencv_warco.cpp)
# if(TARGET ${PROJECT_NAME}-test)
#   target_link_libraries(${PROJECT_NAME}-test ${PROJECT_NAME})
# endif()

## Add folders to be run by python nosetests
# catkin_add_nosetests(test)
