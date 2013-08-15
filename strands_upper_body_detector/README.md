## Upper Body Detector
This package detects the upper bodies of persons using depth and mono images.

### Run
Parameters:
* `queue_size` _default = 20_: The synchronisation queue size
* `config_file` _default = ""_: The global config file. Can be found in strands_upper_bodydetector/config
* `template_file` _default = ""_: The template file. Can be found in config.
* `depth_image` _default = /camera/depth/image_: The Kinect depth image
* `color_image` _default = /camera/rgb/image_color_: The Kincet colour image
* `camera_info` _default = /camera/rgb/camera_info_: The Kinect camera info
* `upper_body_detections` _default = /upper_body_detector/detections_: The deteced upper bodies
* `upper_body_image` _default = /upper_body_detector/image_: The resulting image showing the detections as a boundingbox
* `ground_plane` _default = /ground_plane_: The estimated ground plane


rosrun:
```
rosrun strands_upper_body_detector upper_body_detector [_parameter_name:=value]
```

roslaunch:
```
roslaunch strands_upper_body_detector upper_body_detector.launch [parameter_name:=value]
```
