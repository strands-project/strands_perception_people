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
### Check for cuda ###
message(STATUS "Checking for Nvidia CUDA")
if(NOT DEFINED ENV{CUDA_PATH})
    set(CUDA /usr/local/cuda)
    message(STATUS "CUDA_PATH environment variable has not be set.")
    message(STATUS "Please use `export CUDA_PATH=/cuda/install/path` to specify a custom install location.")
    message(STATUS "Will use standard path: ${CUDA}")
else(NOT DEFINED ENV{CUDA_PATH})
    set(CUDA $ENV{CUDA_PATH})
    message(STATUS "CUDA_PATH environment variable has been found.")
    message(STATUS "Will use: ${CUDA}")
endif(NOT DEFINED ENV{CUDA_PATH})
##################################################################################################################
### CUDA HOG ###
message(STATUS "Preparing libcudaHOB")
message(STATUS "Created patch script: ${CMAKE_CURRENT_BINARY_DIR}/libcudaHOG/rpath_patch.sh")
configure_file(
    ${CMAKE_CURRENT_SOURCE_DIR}/libcudaHOG/rpath_patch.sh.in
    ${CMAKE_CURRENT_BINARY_DIR}/libcudaHOG/rpath_patch.sh
)
message(STATUS "Created cutom make command script: ${CMAKE_CURRENT_BINARY_DIR}/libcudaHOG/custom_make.sh")
configure_file(
    ${CMAKE_CURRENT_SOURCE_DIR}/libcudaHOG/custom_make.sh.in
    ${CMAKE_CURRENT_BINARY_DIR}/libcudaHOG/custom_make.sh
)
set(groundHOG_tar http://www.vision.rwth-aachen.de/projects/groundHOG_v1.1_b.tar.gz)
message(STATUS "Will download libcudaHOG from: ${groundHOG_tar}")
include(ExternalProject)
ExternalProject_Add(
    libcudaHOG
    PREFIX ${CMAKE_CURRENT_BINARY_DIR}/libcudaHOG
    INSTALL_DIR ${CMAKE_CURRENT_BINARY_DIR}/libcudaHOG #May not be necessary if install step skipped. See below
    URL ${groundHOG_tar}
    PATCH_COMMAND sh ${CMAKE_CURRENT_BINARY_DIR}/libcudaHOG/rpath_patch.sh
    CONFIGURE_COMMAND qmake -makefile <SOURCE_DIR>
    # BUILD_IN_SOURCE 1 #TODO: Check if necessary.
    BUILD_COMMAND sh ${CMAKE_CURRENT_BINARY_DIR}/libcudaHOG/custom_make.sh
)
##################################################################################################################
### Use catkin to install files ###
install(FILES
  ${CMAKE_CURRENT_BINARY_DIR}/libcudaHOG/src/libcudaHOG-build/cudaHOG/libcudaHOG.so
  ${CMAKE_CURRENT_BINARY_DIR}/libcudaHOG/src/libcudaHOG-build/cudaHOG/libcudaHOG.so.1
  ${CMAKE_CURRENT_BINARY_DIR}/libcudaHOG/src/libcudaHOG-build/cudaHOG/libcudaHOG.so.1.0
  ${CMAKE_CURRENT_BINARY_DIR}/libcudaHOG/src/libcudaHOG-build/cudaHOG/libcudaHOG.so.1.0.0
  DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
)
