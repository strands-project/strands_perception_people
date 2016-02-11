# Launch package
This convenience package contains launch files to start-up the whole people tracker system.

## General remarks
All the the packages rely heavily on the synchronisation of rgb and depth images and the generated data of the other nodes. The synchronization is realised using a queue which saves a predefined number of messages on which the synchronisation is performed. _As a rule of thumb: the faster your machine the shorter the queue to prevent unnecessary use of memory._ You can set queue sizes using:
```
roslaunch perception_people_launch file.launch gh_queue_size:=11 vo_queue_size:=22 ubd_queue_size:=33 pt_queue_size:=44
```
This will overwrite the default values.  _gh = ground_hog, vo = visual_odemetry, ubd = upper_body_detector, pt = mdl_people_tracker_

The whole pipeline is desinged to unsuscribe from everything if there is no subscriber to the published topics. This causes the nodes to not use any CPU when there is no one listening on the published topics. This might result in a 1-2 second dealy after subscribing to one of the topics before the first data is published. Also, setting `log` to true when starting the perception pipeline will cause it to always run and log data.

## Running on robot
These launch files will make use of the fixed ground plane which is just rotated according to the PTU tilt and the robot odometry instead of the visual odometry. Additionally, the necessary parameters are assumed to be provided by the parameter server (see mongodb_store confog_manager on default parameters) therefore the `load_params_from_file` parameter is set to `false` and the nodes will querry the config parameters from the parameter server. The standalone version on the other hand uses the provided config files. If parameters are not present in the paremeter server on the robot but you want to launch the ppl perception, run with `load_params_from_file:=true`.

### people_tracker_robot.launch
This version of the tracking does not rely on the ground_hog feature extraction and is therefore usable on PCs with no NVIDIA graphics card (like the embedded robot PC). However, this has the drawback that the system only relies on depth data to detect people which limits the distance at which persons can be detected to approx. 5 meters. Where possible the ground_hog detection should be used to enhance tracking results. It also uses the fixed ground plane assumption because it is ment to be executed on the robots head xtion camera.

Parameters:
* `load_params_from_file` _default = true_: `false` tries to read parameters from datacentre, `true` reads parameters from YAML file specified by `param_file`
* `machine` _default = localhost_: Determines on which machine this node should run.
* `user` _default = ""_: The user used for the ssh connection if machine is not localhost.
* `gp_queue_size` _default = 5_: The ground plane sync queue size
* `ubd_queue_size` _default = 5_: The upper body detector sync queue size
* `pt_queue_size` _default = 10_: The people tracking sync queue size
* `ptu_state` _default = /ptu/state_: The ptu state topic
* `camera_namespace` _default = /head_xtion_: The camera namespace.
* `rgb_image` _default = /rgb/image_rect_color_: `camera_namespace` + `rgb_image` = rgb image topic
* `depth_image` _default = /depth/image_rect_: `camera_namespace` + `depth_image` = depth image topic
* `mono_image` _default = /rgb/image_mono_: `camera_namespace` + `mono_image` = mono image topic
* `camera_info_rgb` _default = /rgb/camera_info_: `camera_namespace` + `camera_info_rgb` = rgb camera info topic
* `camera_info_depth` _default = /depth/camera_info_: `camera_namespace` + `camera_info_depth` = depth camera info topic
* `ground_plane` _default = /ground_plane_: The fixed ground plane
* `upper_body_detections` _default = /upper_body_detector/detections_: The detected upper body
* `upper_body_bb_centres` _default = /upper_body_detector/bounding_box_centres_: Publishing a pose array of the centres of the bounding boxes
* `upper_body_markers default = /upper_body_detector/marker_array_: A visualisation array for rviz
* `upper_body_image` _default = /upper_body_detector/image_: The detected upper body image
* `visual_odometry` _default = /visual_odometry/motion_matrix_: The odometry. This takes the real odometry and only follows naming conventions for the ease of use.
* `mdl_people_array` _default = /mdl_people_tracker/people_array_: The detected and tracked people
* `mdl_people_markers" default="/mdl_people_tracker/marker_array_: A visualisation array for rviz
* `mdl_people_poses" default = /mdl_people_tracker/pose_array_: A PoseArray of the detected people
* `tf_target_frame` _default = /map: The coordinate system into which the localisations should be transformed
* `bayes_people_positions` _default = /people_tracker/positions_: The poses of the tracked people
* `bayes_people_pose`: _Default: /people_tracker/pose_: The topic under which the closest detected person is published as a geometry_msgs/PoseStamped`
* `bayes_people_pose_array`: _Default: /people_tracker/pose_array_: The topic under which the detections are published as a geometry_msgs/PoseArray`
* `bayes_people_poeple`: _Default: /people_tracker/people_: The topic under which the results are published as people_msgs/People`
* `pd_marker` _default = /people_tracker/marker_array_: A marker arry to visualise found people in rviz
* `log` _default = false_: Log people and robot locations together with tracking and detection results to message_store database into people_perception collection. Disabled by default because if it is enabled the perception is running continuously.
* `with_mdl_tracker` _default = false_: Starts the mdl people tracker in addition to the bayes tracker
* `with_laser_filter` _default = true_: Starts the laser filter to reduce false positives from the leg detector
* `with_tracker_filter_map` _default = false_: Use a special map to filter the tracker results instead of just the map used for navigation.
* `tracker_filter_map`: The map to use instead of the navigation map to filter the tracker results.
* `tracker_filter_positions` _default = /people_tracker_filter/positions_: The filtered tracker results.
* `tracker_filter_pose` _default = /people_tracker_filter/pose_: The filtered pose for the closest person.
* `tracker_filter_pose_array` _default = /people_tracker_filter/pose_array_: The filetered pose array.
* `tracker_filter_people` _default = /people_tracker_filter/people_: The filetered people message.
* `tracker_filter_marker` _default = /people_tracker_filter/marker_array_: The filetered marker array.


Running:
```
roslaunch perception_people_launch people_tracker_robot.launch [parameter_name:=value]
```

### people_tracker_standalone.launch
This version of the tracking does not rely on the ground_hog feature extraction and is therefore usable on PCs with no NVIDIA graphics card. However, this has the drawback that the system only relies on depth data to detect people which limits the distance at which persons can be detected to approx. 5 meters. Where possible the ground_hog detection should be used to enhance tracking results. 

It also uses the `/camera` namespace as a default and estimates the groundplane because it is not supposed to be run on the robot but on an external PC with a different set-up.


Parameters:
* `load_params_from_file` _default = true_: `false` tries to read parameters from datacentre, `true` reads parameters from YAML file specified by `param_file`
* `machine` _default = localhost_: Determines on which machine this node should run.
* `user` _default = ""_: The user used for the ssh connection if machine is not localhost.
* `gh_queue_size` _default = 20_: The ground plane sync queue size
* `vo_queue_size` _default = 5_: The visual odometry sync queue size
* `ubd_queue_size` _default = 5_: The upper body detector sync queue size
* `pt_queue_size` _default = 10_: The people tracking sync queue size
* `camera_namespace` _default = /camera_: The camera namespace.
* `rgb_image` _default = /rgb/image_rect_color_: `camera_namespace` + `rgb_image` = rgb image topic
* `depth_image` _default = /depth/image_rect_: `camera_namespace` + `depth_image` = depth image topic
* `mono_image` _default = /rgb/image_mono_: `camera_namespace` + `mono_image` = mono image topic
* `camera_info_rgb` _default = /rgb/camera_info_: `camera_namespace` + `camera_info_rgb` = rgb camera info topic
* `camera_info_depth` _default = /depth/camera_info_: `camera_namespace` + `camera_info_depth` = depth camera info topic
* `ground_plane` _default = /ground_plane_: The estimated ground plane
* `upper_body_detections` _default = /upper_body_detector/detections_: The detected upper body
* `upper_body_bb_centres` _default = /upper_body_detector/bounding_box_centres_: Publishing a pose array of the centres of the bounding boxes
* `upper_body_markers default = /upper_body_detector/marker_array_: A visualisation array for rviz
* `upper_body_image` _default = /upper_body_detector/image_: The detected upper body image
* `visual_odometry` _default = /visual_odometry/motion_matrix_: The visual odometry
* `pedestrain_array` _default = /mdl_people_tracker/people_array_: The detected and tracked people
* `people_markers" default="/mdl_people_tracker/marker_array_: A visualisation array for rviz
* `people_poses" default = /mdl_people_tracker/pose_array_: A PoseArray of the detected people
* `tf_target_frame` _default = ""_: The coordinate system into which the localisations should be transformed. As this might not run on a robot and therefore no tf is available this is an empty string.


Running:
```
roslaunch perception_people_launch people_tracker_standalone.launch [parameter_name:=value]
```

