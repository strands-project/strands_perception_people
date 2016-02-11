## Visual Odometry
This package calculates the visual odometry using depth and mono images.

### Run
Parameters:
* `queue_size` _default = 20_: The synchronisation queue size
* `camera_namespace` _default = /head_xtion_: The camera namespace.
* `depth_image` _default = /depth/image_rect_: `camera_namespace` + `depth_image` = depth image topic
* `mono_image` _default = /rgb/image_mono_: `camera_namespace` + `mono_image` = mono image topic
* `camera_info_depth` _default = /depth/camera_info_: `camera_namespace` + `camera_info_depth` = depth camera info topic
* `motion_parameters` _default = /visual_odometry/motion_matrix_: The visual odometry


rosrun:
```
rosrun visual_odometry visual_odometry [_parameter_name:=value]
```

roslaunch:
```
roslaunch visual_odometry visual_odometry.launch [parameter_name:=value]
```

### Troubleshooting
If you get an error message that states: 
```
/usr/lib/gcc/i686-linux-gnu/4.6/include/emmintrin.h:32:3: error: #error "SSE2 instruction set not enabled"
```
or similar, you have to add `set(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} -msse -msse2 -msse3")` to the CMakeLists.txt file after the
```
if(CMAKE_COMPILER_IS_GNUCXX)
    set(CMAKE_CXX_FLAGS "-O3")        ## Optimize
endif()
```
statement so that it looks like:
```
if(CMAKE_COMPILER_IS_GNUCXX)
    set(CMAKE_CXX_FLAGS "-O3")        ## Optimize
endif()

set(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} -msse -msse2 -msse3")
```
