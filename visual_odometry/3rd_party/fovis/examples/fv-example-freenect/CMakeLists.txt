include_directories(${LIBFREENECT_INCLUDE_DIRS} ${EIGEN_INCLUDE_DIRS})

add_executable(fv-example-freenect
    main.cpp
    data_capture.cpp
    )
target_link_libraries(fv-example-freenect ${LIBFREENECT_LIBS} fovis)

pods_install_executables(fv-example-freenect)
