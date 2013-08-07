cmake_minimum_required(VERSION 2.8.3)
project(strands_perception_people_3rd_party)

## Find catkin macros and libraries
## if COMPONENTS list like find_package(catkin REQUIRED COMPONENTS xyz)
## is used, also find other catkin packages
find_package(catkin REQUIRED)

catkin_package(
#  INCLUDE_DIRS include
#  LIBRARIES strands_perception_people_3rd_party
#  CATKIN_DEPENDS other_catkin_pkg
#  DEPENDS system_lib
)

include(ExternalProject)
ExternalProject_Add(
    libgroundHOG
    PREFIX ${CMAKE_CURRENT_SOURCE_DIR}/libgroundHOG
    URL http://www.vision.rwth-aachen.de/projects/groundHOG_v1.1_b.tar.gz
    CONFIGURE_COMMAND qmake -makefile <SOURCE_DIR>
    BUILD_COMMAND make
)
