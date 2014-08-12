# Launch package
This convenience package contains launch files to start-up the whole pedestrian tracker system.

## General remarks
All the the packages rely heavily on the synchronisation of rgb and depth images and the generated data of the other nodes. The synchronization is realised using a queue which saves a predefined number of messages on which the synchronisation is performed. _As a rule of thumb: the faster your machine the shorter the queue to prevent unnecessary use of memory._ You can set queue sizes using:
```
roslaunch strands_perception_people_launch file.launch gh_queue_size:=11 vo_queue_size:=22 ubd_queue_size:=33 pt_queue_size:=44
```
This will overwrite the default values.  _gh = ground_hog, vo = visual_odemetry, ubd = upper_body_detector, pt = pedestrian_tracking_

The whole pipeline is desinged to unsuscribe from everything if there is no subscriber to the published topics. This causes the nodes to not use any CPU when there is no one listening on the published topics. This might result in a 1-2 second dealy after subscribing to one of the topics before the first data is published. Also, setting `log` to true when starting the perception pipeline will cause it to always run and log data.

## Running on robot
These launch files will make use of the fixed ground plane which is just rotated according to the PTU tilt and the robot odometry instead of the visual odometry.

### pedestrian_tracker_robot.launch
This version of the tracking does not rely on the ground_hog feature extraction and is therefore usable on PCs with no NVIDIA graphics card (like the embedded robot PC). However, this has the drawback that the system only relies on depth data to detect people which limits the distance at which persons can be detected to approx. 5 meters. Where possible the ground_hog detection should be used to enhance tracking results. It also uses the fixed ground plane assumption because it is ment to be executed on the robots head xtion camera.

Parameters:
* `gp_queue_size` _default = 5_: The ground plane sync queue size
* `ubd_queue_size` _default = 5_: The upper body detector sync queue size
* `pt_queue_size` _default = 10_: The pedestrian tracking sync queue size
* `model` _default = $(find strands_ground_hog)/model/config_: The ground HOG detection models
* `gp_param_file` _default = $(find strands_ground_plane)/config/fixed_gp.yaml_: The ground plane normal and distance
* `ptu_state` _default = /ptu/state_: The ptu state topic
* `ubd_config_file` _default = $(find strands_upper_body_detector)/config/config_Asus.inp_: The camera config file
* `pt_config_file` _default = $(find strands_upper_body_detector)/config/config_Asus.inp_: The camera config file
* `template_file` _default = $(find strands_upper_body_detector)/config/upper_temp_n.txt_: The upper body templates
* `camera_namespace` _default = /head_xtion_: The camera namespace.
* `ground_plane` _default = /ground_plane_: The fixed ground plane
* `upper_body_detections` _default = /upper_body_detector/detections_: The detected upper body
* `upper_body_bb_centres` _default = /upper_body_detector/bounding_box_centres_: Publishing a pose array of the centres of the bounding boxes
* `upper_body_markers default = /upper_body_detector/marker_array_: A visualisation array for rviz
* `upper_body_image` _default = /upper_body_detector/image_: The detected upper body image
* `visual_odometry` _default = /visual_odometry/motion_matrix_: The odometry. This takes the real odometry and only follows naming conventions for the ease of use.
* `pedestrain_array` _default = /pedestrian_tracking/pedestrian_array_: The detected and tracked pedestrians
* `pedestrian_markers" default="/pedestrian_tracking/marker_array_: A visualisation array for rviz
* `pedestrian_poses" default = /pedestrian_tracking/pose_array_: A PoseArray of the detected pedestrians
* `tf_target_frame` _default = /map: The coordinate system into which the localisations should be transformed
* `pd_positions` _default = /people_tracker/positions_: The poses of the tracked people
* `pd_marker` _default = /people_tracker/marker_array_: A marker arry to visualise found people in rviz
* `log` _default = false_: Log people and robot locations together with tracking and detection results to message_store database into people_perception collection. Disabled by default because if it is enabled the perception is running continuously.


Running:
```
roslaunch strands_perception_people_launch pedestrian_tracker_robot.launch [parameter_name:=value]
```

### pedestrian_tracker_robot_with_HOG.launch
This version of the tracking does rely on the ground_hog feature extraction and is therefore only usable on PCs with an NVIDIA graphics card. It also relys on the fixed ground plane assumption made for the robot. To use this you have to run it remotely on a machine talking to the rosmaster on the robot, e.g. a laptop inside the robot.

Parameters:
* `gp_queue_size` _default = 5_: The ground plane sync queue size
* `gh_queue_size` _default = 20_: The ground plane sync queue size
* `ubd_queue_size` _default = 5_: The upper body detector sync queue size
* `pt_queue_size` _default = 10_: The pedestrian tracking sync queue size
* `model` _default = $(find strands_ground_hog)/model/config_: The ground HOG detection models
* `gp_param_file` _default = $(find strands_ground_plane)/config/fixed_gp.yaml_: The ground plane normal and distance
* `ptu_state` _default = /ptu/state_: The ptu state topic
* `ubd_config_file` _default = $(find strands_upper_body_detector)/config/config_Asus.inp_: The camera config file
* `pt_config_file` _default = $(find strands_upper_body_detector)/config/config_Asus.inp_: The camera config file
* `template_file` _default = $(find strands_upper_body_detector)/config/upper_temp_n.txt_: The upper body templates
* `camera_namespace` _default = /head_xtion_: The camera namespace.
* `ground_plane` _default = /ground_plane_: The fixed ground plane
* `upper_body_detections` _default = /upper_body_detector/detections_: The detected upper body
* `upper_body_bb_centres` _default = /upper_body_detector/bounding_box_centres_: Publishing a pose array of the centres of the bounding boxes
* `upper_body_markers default = /upper_body_detector/marker_array_: A visualisation array for rviz
* `upper_body_image` _default = /upper_body_detector/image_: The detected upper body image
* `visual_odometry` _default = /visual_odometry/motion_matrix_: The odometry. This takes the real odometry and only follows naming conventions for the ease of use.
* `pedestrain_array` _default = /pedestrian_tracking/pedestrian_array_: The detected and tracked pedestrians
* `pedestrian_markers" default="/pedestrian_tracking/marker_array_: A visualisation array for rviz
* `pedestrian_poses" default = /pedestrian_tracking/pose_array_: A PoseArray of the detected pedestrians
* `tf_target_frame` _default = /map: The coordinate system into which the localisations should be transformed
* `pd_positions` _default = /people_tracker/positions_: The poses of the tracked people
* `pd_marker` _default = /people_tracker/marker_array_: A marker arry to visualise found people in rviz
* `log` _default = false_: Log people and robot locations together with tracking and detection results to message_store database into people_perception collection. Disabled by default because if it is enabled the perception is running continuously.


Running:
```
roslaunch strands_perception_people_launch pedestrian_tracker_robot_with_HOG.launch [parameter_name:=value]
```

### pedestrian_tracker_standalone.launch
This version of the tracking does not rely on the ground_hog feature extraction and is therefore usable on PCs with no NVIDIA graphics card. However, this has the drawback that the system only relies on depth data to detect people which limits the distance at which persons can be detected to approx. 5 meters. Where possible the ground_hog detection should be used to enhance tracking results. 

It also uses the `/camera` namespace as a default and estimates the groundplane because it is not supposed to be run on the robot but on an external PC with a different set-up.


Parameters:
* `visualise` _default = false_: Set to true to render and publish result images.
* `gh_queue_size` _default = 20_: The ground plane sync queue size
* `vo_queue_size` _default = 5_: The visual odometry sync queue size
* `ubd_queue_size` _default = 5_: The upper body detector sync queue size
* `pt_queue_size` _default = 10_: The pedestrian tracking sync queue size
* `model` _default = $(find strands_ground_hog)/model/config_: The ground HOG detection models
* `gp_config_file` _default = $(find strands_ground_plane)/config/config_Asus.inp_: The camera config file
* `ubd_config_file` _default = $(find strands_upper_body_detector)/config/config_Asus.inp_: The camera config file
* `pt_config_file` _default = $(find strands_upper_body_detector)/config/config_Asus.inp_: The camera config file
* `template_file` _default = $(find strands_upper_body_detector)/config/upper_temp_n.txt_: The upper body templates
* `camera_namespace` _default = /camera_: The camera namespace.
* `ground_plane` _default = /ground_plane_: The estimated ground plane
* `upper_body_detections` _default = /upper_body_detector/detections_: The detected upper body
* `upper_body_bb_centres` _default = /upper_body_detector/bounding_box_centres_: Publishing a pose array of the centres of the bounding boxes
* `upper_body_markers default = /upper_body_detector/marker_array_: A visualisation array for rviz
* `upper_body_image` _default = /upper_body_detector/image_: The detected upper body image
* `visual_odometry` _default = /visual_odometry/motion_matrix_: The visual odometry
* `pedestrain_array` _default = /pedestrian_tracking/pedestrian_array_: The detected and tracked pedestrians
* `pedestrian_markers" default="/pedestrian_tracking/marker_array_: A visualisation array for rviz
* `pedestrian_poses" default = /pedestrian_tracking/pose_array_: A PoseArray of the detected pedestrians
* `tf_target_frame` _default = ""_: The coordinate system into which the localisations should be transformed. As this might not run on a robot and therefore no tf is available this is an empty string.


Running:
```
roslaunch strands_perception_people_launch pedestrian_tracker_standalone.launch [parameter_name:=value]
```

### pedestrian_tracker_standalone_with_HOG.launch
This depends on the strands_ground_hog package which has to be built with the libcudaHOG. See README file of 3rd_party directory. It also uses the `/camera` namespace as a default and estimates the groundplane because it is not supposed to be run on the robot but on an external PC with a different set-up.

Parameters:
* `gh_queue_size` _default = 10_: The ground hog sync queue size
* `gp_queue_size` _default = 5_: The ground plane sync queue size
* `vo_queue_size` _default = 5_: The visual odometry sync queue size
* `ubd_queue_size` _default = 5_: The upper body detector sync queue size
* `pt_queue_size` _default = 10_: The pedestrian tracking sync queue size
* `model` _default = $(find strands_ground_hog)/model/config_: The ground HOG detection models
* `gp_param_file` _default = $(find strands_ground_plane)/config/fixed_gp.yaml_: The ground plane normal and distance
* `ptu_state` _default = /ptu/state_: The ptu state topic
* `ubd_config_file` _default = $(find strands_upper_body_detector)/config/config_Asus.inp_: The camera config file
* `pt_config_file` _default = $(find strands_upper_body_detector)/config/config_Asus.inp_: The camera config file
* `template_file` _default = $(find strands_upper_body_detector)/config/upper_temp_n.txt_: The upper body templates
* `camera_namespace` _default = /camera_: The camera namespace.
* `ground_plane` _default = /ground_plane_: The estimated ground plane
* `ground_hog_detections` _default = /groundHOG/detections_: The ground HOG detections
* `upper_body_detections` _default = /upper_body_detector/detections_: The detected upper body
* `upper_body_bb_centres` _default = /upper_body_detector/bounding_box_centres_: Publishing a pose array of the centres of the bounding boxes
* `upper_body_markers default = /upper_body_detector/marker_array_: A visualisation array for rviz
* `ground_hog_image` _default = /groundHOG/image_: The ground HOG image
* `upper_body_image` _default = /upper_body_detector/image_: The detected upper body image
* `visual_odometry` _default = /visual_odometry/motion_matrix_: The visual odometry
* `pedestrian_markers" default="/pedestrian_tracking/marker_array_: A visualisation array for rviz
* `pedestrian_poses" default = /pedestrian_tracking/pose_array_: A PoseArray of the detected pedestrians
* `pedestrian_markers" default="/pedestrian_tracking/marker_array`: A visualisation array for rviz
* `tf_target_frame` _default = ""_: The coordinate system into which the localisations should be transformed. As this might not run on a robot and therefore no tf is available this is an empty string.


Running:
```
roslaunch strands_perception_people_launch pedestrian_tracker_standalone_with_HOG.launch [parameter_name:=value]
```

