## Visual Odometry
This package calculates the visual odometry using depth and mono images.

### Run
Parameters:
* `queue_size` _default = 20_: The synchronisation queue size
* `depth_image` _default = /camera/depth/image_: The Kinect depth image
* `mono_image` _default = /camera/rgb/image_mono_: The Kincet mono image
* `camera_info` _default = /camera/rgb/camera_info_: The Kinect camera info
* `motion_parameters` _default = /visual_odometry/motion_matrix_: The visual odometry


rosrun:
```
rosrun strands_visual_odometry visual_odometry [_parameter_name:=value]
```

roslaunch:
```
roslaunch strands_visual_odometry visual_odometry.launch [parameter_name:=value]

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
