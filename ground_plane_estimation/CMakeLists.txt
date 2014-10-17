cmake_minimum_required(VERSION 2.8.3)
project(ground_plane_estimation)

## Find catkin macros and libraries
## if COMPONENTS list like find_package(catkin REQUIRED COMPONENTS xyz)
## is used, also find other catkin packages
find_package(catkin REQUIRED COMPONENTS 
    cv_bridge 
    image_transport
    message_filters 
    message_generation 
    roscpp 
    sensor_msgs 
    std_msgs 
    tf 
)
find_package(Boost REQUIRED COMPONENTS thread)

set(CMAKE_BUILD_TYPE Release)

if(CMAKE_COMPILER_IS_GNUCXX)
    set(CMAKE_CXX_FLAGS "-O3")        ## Optimize
endif()

#######################################
## Declare ROS messages and services ##
#######################################

add_message_files(
  FILES
  GroundPlane.msg
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
    image_transport
    message_filters 
    roscpp 
    sensor_msgs 
    std_msgs 
    tf 
)

###########
## Build ##
###########

include_directories(include
  ${catkin_INCLUDE_DIRS}
  ${Boost_INCLUDE_DIRS}
)

add_executable(ground_plane_estimated src/estimated_gp.cpp src/Camera.cpp src/Globals.cpp src/groundplaneestimator.cpp src/Math.cpp src/pointcloud.cpp)
add_executable(ground_plane_fixed src/fixed_gp.cpp)

add_dependencies(ground_plane_estimated ${PROJECT_NAME}_generate_messages_cpp ${catkin_EXPORTED_TARGETS})
add_dependencies(ground_plane_fixed ${PROJECT_NAME}_generate_messages_cpp ${catkin_EXPORTED_TARGETS})

target_link_libraries(ground_plane_estimated
  ${catkin_LIBRARIES}
  ${Boost_LIBRARIES}
)
target_link_libraries(ground_plane_fixed
  ${catkin_LIBRARIES}
)

#############
## Install ##
#############

install(TARGETS ground_plane_estimated ground_plane_fixed
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
