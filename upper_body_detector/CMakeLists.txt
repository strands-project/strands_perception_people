cmake_minimum_required(VERSION 2.8.3)
project(upper_body_detector)

find_package(catkin REQUIRED COMPONENTS 
    cv_bridge 
    geometry_msgs 
    ground_plane_estimation
    image_transport 
    message_filters 
    message_generation
    roscpp 
    sensor_msgs 
    std_msgs 
    visualization_msgs
    )

find_package(Boost REQUIRED COMPONENTS thread)
find_package(Qt4 REQUIRED COMPONENTS QtCore QtGui)
include(${QT_USE_FILE})

set(CMAKE_BUILD_TYPE Release)

if(CMAKE_COMPILER_IS_GNUCXX)
    set(CMAKE_CXX_FLAGS "-O3")        ## Optimize
endif()

#######################################
## Declare ROS messages and services ##
#######################################

add_message_files(
  FILES
  UpperBodyDetector.msg
)

generate_messages(
  DEPENDENCIES
  std_msgs
  geometry_msgs
)

###################################
## catkin specific configuration ##
###################################
catkin_package(
  INCLUDE_DIRS include
  CATKIN_DEPENDS 
    cv_bridge 
    geometry_msgs 
    ground_plane_estimation
    image_transport 
    message_filters 
    roscpp 
    sensor_msgs 
    std_msgs 
    visualization_msgs
)

###########
## Build ##
###########

include_directories( include
  ${Boost_INCLUDE_DIRS}
  ${catkin_INCLUDE_DIRS}
)

add_executable(upper_body_detector 
    src/main.cpp 
    src/Camera.cpp 
    src/Globals.cpp 
    src/KConnectedComponentLabeler.cpp 
    src/Math.cpp 
    src/pointcloud.cpp 
    src/ROI.cpp 
    src/CPoint.cpp 
    src/detector.cpp 
    src/AncillaryMethods.cpp 
    src/Hypo.cpp 
    src/FrameInlier.cpp
) 

add_dependencies(upper_body_detector ${PROJECT_NAME}_generate_messages_cpp ${catkin_EXPORTED_TARGETS})

target_link_libraries(upper_body_detector
  ${catkin_LIBRARIES}
  ${Boost_LIBRARIES}
  ${QT_LIBRARIES}
  ${QT_QTCORE_LIBRARY}
  ${QT_QTGUI_LIBRARY}
)

#############
## Install ##
#############

install(TARGETS upper_body_detector
  ARCHIVE DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
  LIBRARY DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
  RUNTIME DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION}
)

install(DIRECTORY launch
  DESTINATION ${CATKIN_PACKAGE_SHARE_DESTINATION}
)

install(DIRECTORY config
  DESTINATION ${CATKIN_PACKAGE_SHARE_DESTINATION}
)
