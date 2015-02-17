cmake_minimum_required(VERSION 2.8.3)
project(visual_odometry)

find_package(PkgConfig REQUIRED)
find_package(catkin REQUIRED COMPONENTS 
    cmake_modules
    cv_bridge 
    geometry_msgs 
    image_transport
    message_filters 
    message_generation 
    roscpp 
    sensor_msgs 
    std_msgs 
    )

# Include Eigen
find_package(Eigen REQUIRED) 
include_directories(${EIGEN_INCLUDE_DIRS}) 
add_definitions(${EIGEN_DEFINITIONS})

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
  VisualOdometry.msg
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
  CATKIN_DEPENDS
    cv_bridge 
    geometry_msgs 
    image_transport
    message_filters 
    roscpp 
    sensor_msgs 
    std_msgs 
)

###########
## Build ##
###########

include_directories(
  ${catkin_INCLUDE_DIRS}
  ${Boost_INCLUDE_DIRS}
  ${CMAKE_CURRENT_SOURCE_DIR}/3rd_party/fovis/libfovis
)

FILE(GLOB FOVIS_CPP ${CMAKE_CURRENT_SOURCE_DIR}/3rd_party/fovis/libfovis/*.cpp)
FILE(GLOB FOVIS_C ${CMAKE_CURRENT_SOURCE_DIR}/3rd_party/fovis/libfovis/*.c)

add_executable(visual_odometry src/main.cpp ${FOVIS_CPP} ${FOVIS_C})

add_dependencies(visual_odometry ${PROJECT_NAME}_generate_messages_cpp ${catkin_EXPORTED_TARGETS})

target_link_libraries(visual_odometry
    ${catkin_LIBRARIES}
    ${Boost_LIBRARIES}
)

#############
## Install ##
#############

install(TARGETS visual_odometry
  ARCHIVE DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
  LIBRARY DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
  RUNTIME DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION}
)

install(DIRECTORY launch
  DESTINATION ${CATKIN_PACKAGE_SHARE_DESTINATION}
)
