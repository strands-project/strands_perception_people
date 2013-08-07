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

##################################################################################################################
### Ground HOG ###
set(groundHOG_tar http://www.vision.rwth-aachen.de/projects/groundHOG_v1.1_b.tar.gz)
message(STATUS "Downloading and building libgroundHOG from: ${groundHOG_tar}")
include(ExternalProject)
ExternalProject_Add(
    libgroundHOG
    PREFIX ${CMAKE_CURRENT_BINARY_DIR}/libgroundHOG
    INSTALL_DIR ${CMAKE_CURRENT_BINARY_DIR}/libgroundHOG #May not be necessary if install step skipped. See below
    URL ${groundHOG_tar}
    CONFIGURE_COMMAND qmake -makefile <SOURCE_DIR>
    # BUILD_IN_SOURCE 1 #TODO: Check if necessary.
    BUILD_COMMAND make
    #INSTALL_COMMAND #TODO: May have to set custom install command or skipp install step with: INSTALL_COMMAND ""
)
##################################################################################################################
