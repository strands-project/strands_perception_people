## Ground Plane Estimation
This package estimates the ground plane using depth images.

### Run
Parameters:
* `queue_size` _default = 5_: The synchronisation queue size
* `config_file` _default = ""_: The global config file. Can be found in strands_ground_plane/config
* `depth_image` _default = /camera/depth/image_: The Kinect depth image
* `camera_info` _default = /camera/rgb/camera_info_: The Kinect camera info
* `ground_plane` _default = /ground_plane_: The estimated ground plane


rosrun:
```
rosrun strands_ground_plane ground_plane [_parameter_name:=value]
```

roslaunch:
```
roslaunch strands_ground_plane ground_plane.launch [parameter_name:=value]
```
