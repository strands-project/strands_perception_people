## Upper Body Detector
This package detects the upper bodies of persons using the depth image.

### Run
Parameters:
* `queue_size` _default = 20_: The synchronisation queue size
* `config_file` _default = ""_: The global config file. Can be found in strands_upper_bodydetector/config
* `template_file` _default = ""_: The template file. Can be found in config.
* `camera_namespace` _default = /head_xtion_: The camera namespace.
* `ground_plane` _default = /ground_plane_: The estimated/fixed ground plane
* `upper_body_detections` _default = /upper_body_detector/detections_: The deteced upper bodies
* `upper_body_bb_centres` _default = /upper_body_detector/bounding_box_centres_: Publishing a pose array of the centres of the bounding boxes
* `upper_body_image` _default = /upper_body_detector/image_: The resulting image showing the detections as a boundingbox
* `upper_body_markers default = /upper_body_detector/marker_array_: A visualisation array for rviz


rosrun:
```
rosrun strands_upper_body_detector upper_body_detector [_parameter_name:=value]
```

roslaunch:
```
roslaunch strands_upper_body_detector upper_body_detector.launch [parameter_name:=value]
```
