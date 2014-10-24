cmake_minimum_required(VERSION 2.8)
project(warco)

if(CMAKE_COMPILER_IS_GNUCXX)
    execute_process(COMMAND ${CMAKE_CXX_COMPILER} -dumpversion OUTPUT_VARIABLE GCC_VERSION)
    string(STRIP "${GCC_VERSION}" GCC_VERSION)
    message(STATUS "GCC Version ${GCC_VERSION} detected")
    message(STATUS "GCC is ${CMAKE_CXX_COMPILER}")

    # GCC below 4.6 does not have C++0x features we use.
    if(${GCC_VERSION} VERSION_LESS "4.6")
        message(FATAL_ERROR "You need at least GCC version 4.6")
    elseif(${GCC_VERSION} VERSION_LESS "4.7")
        add_definitions(-std=c++0x)
    else()
        add_definitions(-std=c++11)
    endif()

    add_definitions(-Wall)
endif()

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

# For compiling libsvm in dense mode.
add_definitions(-D_DENSE_REP)

set(LIB_SRC
    cvutils.cpp
    cvutils.hpp

    covcorr.cpp
    covcorr.hpp
    dists.cpp
    dists.hpp
    features.cpp
    features.hpp
    filterbank.cpp
    filterbank.hpp
    model.cpp
    model.hpp
    warco.cpp
    warco.hpp

    libsvm/svm.cpp
    libsvm/svm.h
)

set(COMMON_SRC
    mainutils.cpp
    mainutils.hpp

    jsoncpp.cpp
    json/json.h
    json/json-forwards.h
)

add_library(warco ${LIB_SRC})
add_executable(warco-train train.cpp ${COMMON_SRC})
add_executable(warco-pred pred.cpp ${COMMON_SRC})
add_executable(warco-traintest traintest.cpp ${COMMON_SRC})
add_executable(warco-utest utest.cpp ${COMMON_SRC})
add_executable(warco-dists save_dists.cpp ${COMMON_SRC})
add_executable(warco-covs save_covs.cpp ${COMMON_SRC})
target_link_libraries(warco-train warco ${OpenCV_LIBS})
target_link_libraries(warco-pred warco ${OpenCV_LIBS})
target_link_libraries(warco-traintest warco ${OpenCV_LIBS})
target_link_libraries(warco-utest warco ${OpenCV_LIBS})
target_link_libraries(warco-dists warco ${OpenCV_LIBS})
target_link_libraries(warco-covs warco ${OpenCV_LIBS})
