cmake_minimum_required(VERSION 2.8.3)
project(strands_perception_people_3rd_party)

##################################################################################################################
### Check for cuda ### TODO: Do it cmake style now that we are not using catkin anymore.
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
message(STATUS "Preparing libcudaHOG")
message(STATUS "Created patch script: ${CMAKE_CURRENT_BINARY_DIR}/libcudaHOG/scripts/patch.sh")
configure_file(
    ${CMAKE_CURRENT_SOURCE_DIR}/libcudaHOG/scripts/patch.sh.in
    ${CMAKE_CURRENT_BINARY_DIR}/libcudaHOG/scripts/patch.sh
)
message(STATUS "Created build script: ${CMAKE_CURRENT_BINARY_DIR}/libcudaHOG/scripts/make.sh")
configure_file(
    ${CMAKE_CURRENT_SOURCE_DIR}/libcudaHOG/scripts/make.sh.in
    ${CMAKE_CURRENT_BINARY_DIR}/libcudaHOG/scripts/make.sh
)
message(STATUS "Created install script: ${CMAKE_CURRENT_BINARY_DIR}/libcudaHOG/scripts/install.sh")
configure_file(
    ${CMAKE_CURRENT_SOURCE_DIR}/libcudaHOG/scripts/install.sh.in
    ${CMAKE_CURRENT_BINARY_DIR}/libcudaHOG/scripts/install.sh
)
message(STATUS "Created pkg-config file: ${CMAKE_CURRENT_BINARY_DIR}/libcudaHOG/pkgconfig/cudaHOG.pc")
configure_file( #TODO: Do not override variables in file like ${prefix} -> google
    ${CMAKE_CURRENT_SOURCE_DIR}/libcudaHOG/pkgconfig/cudaHOG.pc.in
    ${CMAKE_CURRENT_BINARY_DIR}/libcudaHOG/pkgconfig/cudaHOG.pc
)
set(groundHOG_tar http://www.vision.rwth-aachen.de/projects/groundHOG_v1.1_b.tar.gz)
message(STATUS "Will download libcudaHOG from: ${groundHOG_tar}")
include(ExternalProject)
ExternalProject_Add(
    libcudaHOG
    PREFIX ${CMAKE_CURRENT_BINARY_DIR}/libcudaHOG
    INSTALL_DIR ${CMAKE_INSTALL_PREFIX} #May not be necessary because of install script. See below
    URL ${groundHOG_tar}
    PATCH_COMMAND sh ${CMAKE_CURRENT_BINARY_DIR}/libcudaHOG/scripts/patch.sh
    CONFIGURE_COMMAND qmake -makefile <SOURCE_DIR>
    BUILD_COMMAND sh ${CMAKE_CURRENT_BINARY_DIR}/libcudaHOG/scripts/make.sh
    INSTALL_COMMAND sh ${CMAKE_CURRENT_BINARY_DIR}/libcudaHOG/scripts/install.sh
)
##################################################################################################################
