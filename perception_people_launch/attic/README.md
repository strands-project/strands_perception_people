## Attic directory containing depricated launch files

these launch files are used to start the ppl perception using HOG featur detection. This is currently deprecated and not supported. The code for this can be found in the `attic` branch. This directory is not installed and therefore just to preserve the files themselves. Following are the instructions on how to use these files:


### people_tracker_robot_with_HOG.launch
This version of the tracking does rely on the ground_hog feature extraction and is therefore only usable on PCs with an NVIDIA graphics card. It also relys on the fixed ground plane assumption made for the robot. To use this you have to run it remotely on a machine talking to the rosmaster on the robot, e.g. a laptop inside the robot.

Parameters:
* `load_params_from_file` _default = true_: `false` tries to read parameters from datacentre, `true` reads parameters from YAML file specified by `param_file`
* `machine` _default = localhost_: Determines on which machine this node should run.
* `user` _default = ""_: The user used for the ssh connection if machine is not localhost.
* `gp_queue_size` _default = 5_: The ground plane sync queue size
* `gh_queue_size` _default = 20_: The ground plane sync queue size
* `ubd_queue_size` _default = 5_: The upper body detector sync queue size
* `pt_queue_size` _default = 10_: The people tracking sync queue size
* `ptu_state` _default = /ptu/state_: The ptu state topic
* `camera_namespace` _default = /head_xtion_: The camera namespace.
* `rgb_image` _default = /rgb/image_rect_color_: `camera_namespace` + `rgb_image` = rgb image topic
* `depth_image` _default = /depth/image_rect_meters_: `camera_namespace` + `depth_image` = depth image topic
* `mono_image` _default = /rgb/image_mono_: `camera_namespace` + `mono_image` = mono image topic
* `camera_info_rgb` _default = /rgb/camera_info_: `camera_namespace` + `camera_info_rgb` = rgb camera info topic
* `camera_info_depth` _default = /depth/camera_info_: `camera_namespace` + `camera_info_depth` = depth camera info topic
* `ground_plane` _default = /ground_plane_: The fixed ground plane
* `upper_body_detections` _default = /upper_body_detector/detections_: The detected upper body
* `upper_body_bb_centres` _default = /upper_body_detector/bounding_box_centres_: Publishing a pose array of the centres of the bounding boxes
* `upper_body_markers default = /upper_body_detector/marker_array_: A visualisation array for rviz
* `upper_body_image` _default = /upper_body_detector/image_: The detected upper body image
* `visual_odometry` _default = /visual_odometry/motion_matrix_: The odometry. This takes the real odometry and only follows naming conventions for the ease of use.
* `pedestrain_array` _default = /mdl_people_tracker/people_array_: The detected and tracked people
* `people_markers" default="/mdl_people_tracker/marker_array_: A visualisation array for rviz
* `people_poses" default = /mdl_people_tracker/pose_array_: A PoseArray of the detected people
* `tf_target_frame` _default = /map: The coordinate system into which the localisations should be transformed
* `pd_positions` _default = /people_tracker/positions_: The poses of the tracked people
* `pd_marker` _default = /people_tracker/marker_array_: A marker arry to visualise found people in rviz
* `log` _default = false_: Log people and robot locations together with tracking and detection results to message_store database into people_perception collection. Disabled by default because if it is enabled the perception is running continuously.


Running:
```
roslaunch perception_people_launch people_tracker_robot_with_HOG.launch [parameter_name:=value]
```

### people_tracker_standalone_with_HOG.launch
This depends on the strands_ground_hog package which has to be built with the libcudaHOG. See README file of 3rd_party directory. It also uses the `/camera` namespace as a default and estimates the groundplane because it is not supposed to be run on the robot but on an external PC with a different set-up.

Parameters:
* `load_params_from_file` _default = true_: `false` tries to read parameters from datacentre, `true` reads parameters from YAML file specified by `param_file`
* `machine` _default = localhost_: Determines on which machine this node should run.
* `user` _default = ""_: The user used for the ssh connection if machine is not localhost.
* `gh_queue_size` _default = 10_: The ground hog sync queue size
* `gp_queue_size` _default = 5_: The ground plane sync queue size
* `vo_queue_size` _default = 5_: The visual odometry sync queue size
* `ubd_queue_size` _default = 5_: The upper body detector sync queue size
* `pt_queue_size` _default = 10_: The people tracking sync queue size
* `camera_namespace` _default = /camera_: The camera namespace.
* `rgb_image` _default = /rgb/image_rect_color_: `camera_namespace` + `rgb_image` = rgb image topic
* `depth_image` _default = /depth/image_rect_meters_: `camera_namespace` + `depth_image` = depth image topic
* `mono_image` _default = /rgb/image_mono_: `camera_namespace` + `mono_image` = mono image topic
* `camera_info_rgb` _default = /rgb/camera_info_: `camera_namespace` + `camera_info_rgb` = rgb camera info topic
* `camera_info_depth` _default = /depth/camera_info_: `camera_namespace` + `camera_info_depth` = depth camera info topic
* `ground_plane` _default = /ground_plane_: The estimated ground plane
* `ground_hog_detections` _default = /groundHOG/detections_: The ground HOG detections
* `upper_body_detections` _default = /upper_body_detector/detections_: The detected upper body
* `upper_body_bb_centres` _default = /upper_body_detector/bounding_box_centres_: Publishing a pose array of the centres of the bounding boxes
* `upper_body_markers default = /upper_body_detector/marker_array_: A visualisation array for rviz
* `ground_hog_image` _default = /groundHOG/image_: The ground HOG image
* `upper_body_image` _default = /upper_body_detector/image_: The detected upper body image
* `visual_odometry` _default = /visual_odometry/motion_matrix_: The visual odometry
* `people_markers" default="/mdl_people_tracker/marker_array_: A visualisation array for rviz
* `people_poses" default = /mdl_people_tracker/pose_array_: A PoseArray of the detected people
* `people_markers" default="/mdl_people_tracker/marker_array`: A visualisation array for rviz
* `tf_target_frame` _default = ""_: The coordinate system into which the localisations should be transformed. As this might not run on a robot and therefore no tf is available this is an empty string.


Running:
```
roslaunch perception_people_launch people_tracker_standalone_with_HOG.launch [parameter_name:=value]
```

