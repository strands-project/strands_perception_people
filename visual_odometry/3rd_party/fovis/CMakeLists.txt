cmake_minimum_required(VERSION 2.6.0)

# pull in the pods macros. See cmake/pods.cmake for documentation
set(POD_NAME libfovis)
include(cmake/pods.cmake)

# require pkg-config to build.  We use pkg-config to search for both
# required and optional dependencies (Eigen, LCM, libbot2, etc.)
find_package(PkgConfig REQUIRED)

pkg_check_modules(EIGEN REQUIRED eigen3)

# build the main library
add_subdirectory(libfovis)

add_subdirectory(testers)
add_subdirectory(examples)

message("

Configuration (fovis):

       libfovis:        Enabled
 OpenNI example:        ${OPENNI_EXAMPLE_MESSAGE}
 libfreenct example:    ${LIBFREENECT_EXAMPLE_MESSAGE}

")
