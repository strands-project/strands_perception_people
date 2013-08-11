include_directories(${OPENNI_INCLUDE_DIRS} ${EIGEN_INCLUDE_DIRS})

add_executable(fv-example-openni
    main.cpp
    data_capture.cpp
    )
target_link_libraries(fv-example-openni ${OPENNI_LIBS} fovis)

pods_install_executables(fv-example-openni)
