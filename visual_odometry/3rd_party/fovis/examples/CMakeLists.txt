# look for OpenNI
find_path(OPENNI_INCLUDE_DIRS
    NAMES XnOS.h
    PATH_SUFFIXES ni)
if("${OPENNI_INCLUDE_DIRS}" STREQUAL "OPENNI_INCLUDE_DIRS-NOTFOUND")
    set(OPENNI_EXAMPLE_MESSAGE "Disabled - XnOS.h not found" PARENT_SCOPE)
else()
    find_library(OPENNI_LIBS OpenNI)
    if("${OPENNI_LIBS}" STREQUAL "OPENNI_LIBS-NOTFOUND")
        set(OPENNI_EXAMPLE_MESSAGE "Disabled - OpenNI library not found" PARENT_SCOPE)
    else()
        set(OPENNI_EXAMPLE_MESSAGE "Enabled" PARENT_SCOPE)
        add_subdirectory(fv-example-openni)
    endif()
endif()

# look for libfreenect
find_path(LIBFREENECT_INCLUDE_DIRS
    NAMES libfreenect.h
    PATH_SUFFIXES libfreenect)
if("${LIBFREENECT_INCLUDE_DIRS}" STREQUAL "LIBFREENECT_INCLUDE_DIRS-NOTFOUND")
    set(LIBFREENECT_EXAMPLE_MESSAGE "Disabled - libfreenect.h not found" PARENT_SCOPE)
else()
    find_library(LIBFREENECT_LIBS freenect HINTS /usr/local/lib64)
    if("${LIBFREENECT_LIBS}" STREQUAL "LIBFREENECT_LIBS-NOTFOUND")
        set(LIBFREENECT_EXAMPLE_MESSAGE "Disabled - libfreenect not found" PARENT_SCOPE)
    else()
        set(LIBFREENECT_EXAMPLE_MESSAGE "Enabled" PARENT_SCOPE)
        add_subdirectory(fv-example-freenect)
    endif()
endif()
