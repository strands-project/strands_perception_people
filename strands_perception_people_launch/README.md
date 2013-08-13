## Launch package
This convenience package contains launch files to start-up the whole pedestrian tracker system.

### Launching the whole system
This depends on the strands_ground_hog package which has to be built with the libcudaHOG. See README file of 3rd_party directory.

Parameters:
* `gh_queue_size` _default = 20_: The ground plane sync queue size
* `vo_queue_size` _default = 5_: The visual odometry sync queue size
* `ubd_queue_size` _default = 5_: The upper body detector sync queue size
* `pt_queue_size` _default = 10_: The pedestrian tracking sync queue size
* `model` _default = $(find strands_ground_hog)/model/config_: The ground HOG detection models
* `config_file` _default = $(find strands_upper_body_detector)/config/config_Asus.inp_: The camera config file
* `template_file` _default = $(find strands_upper_body_detector)/config/upper_temp_n.txt_: The upper body templates
* `depth_image` _default = /camera/depth/image_: The Kinect depth image
* `color_image` _default = /camera/rgb/image_color_: The Kinect colour image
* `mono_image` _default = /camera/rgb/image_mono_: The Kinect mono image
* `camera_info` _default = /camera/rgb/camera_info_: The Kinect camera info
* `ground_plane` _default = /ground_plane_: The estimated ground plane
* `ground_hog_detections` _default = /groundHOG/detections_: The ground HOG detections
* `upper_body_detections` _default = /upper_body_detector/detections_: The detected upper body
* `ground_hog_image` _default = /groundHOG/image_: The ground HOG image
* `upper_body_image` _default = /upper_body_detector/image_: The detected upper body image
* `visual_odometry` _default = /visual_odometry/motion_matrix_: The visual odometry
* `pedestrain_array` _default = /pedestrian_tracking/pedestrian_array_: The detected and tracked pedestrians


* Launch everything:
```
roslaunch strands_perception_people_launch pedestrian_tracker.launch [parameter_name:=value]
```
* Launch everything with custom queue sizes: All the the packages rely heavily on the synchronisation of rgb and depth images and the generated data of the other nodes. The synchronization is realised using a queue which saves a predefined number of messages on which the synchronisation is performed. _As a rule of thumb: the faster your machine the shorter the queue to prevent unnecessary use of memory._ You can set queue sizes using:
```
roslaunch strands_perception_people_launch pedestrian_tracker.launch gh_queue_size:=11 vo_queue_size:=22 ubd_queue_size:=33 pt_queue_size:=44
```
This will overwrite the default values. _gh = ground_hog, vo = visual_odemetry, ubd = upper_body_detector, pt = pedestrian_tracking_

### Launching system without ground_hog
_Not implemented yet_
