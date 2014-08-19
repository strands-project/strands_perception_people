## People Tracker
This package uses the bayestracking library developed by Nicola Bellotto (University of Lincoln). A catkinized version can be found [here](https://github.com/LCAS/bayestracking/tree/catkin-devel) and is also included as a submodule into this repository.

The people_tracker uses a single config file to add an arbitrary amount of detectors. The file `config/detectors.yaml` contains the necessary information for the upper_body_detector and the ROS leg_detector (see `to_pose_array` in strands_perception_people_utils/README.md):

```
strands_people_tracker:
    detectors:                                                 # Add detectors under this namespace
        upper_body_detector:                                   # Name of detector (used internally to identify them. Has to be unique.
            topic: "/upper_body_detector/bounding_box_centres" # The topic on which the geometry_msgs/PoseArray is published
            noise_model:
                velocity:                                      # The std deviation for the constant velocity model
                    x: 1.4
                    y: 1.4
                position:                                      # The std deviation for the cartesian model
                    x: 1.2
                    y: 1.2
            matching_algorithm: "NNJPDA"                       # The algorthim to match different detections. NN = Nearest Neighbour, JPDA = Joint Probability Data Association, NNJPDA = NN + JPDA
        leg_detector:                                          # Name of detector (used internally to identify them. Has to be unique.
            topic: "/to_pose_array/leg_detector" # The topic on which the geometry_msgs/PoseArray is published
            noise_model:
                velocity:                                      # The std deviation for the constant velocity model
                    x: 1.4
                    y: 1.4
                position:                                      # The std deviation for the cartesian model
                    x: 0.2
                    y: 0.2
            matching_algorithm: "NNJPDA"                       # The algorthim to match different detections. NN = Nearest Neighbour, JPDA = Joint Probability Data Association, NNJPDA = NN + JPDA
```

New detectors are added under the parameter namespace `strands_people_tracker/detectors`. Let's have a look at the upper body detector as an example:

* For every detector you have to create a new namespace where the name is used as an internal identifier for this detector. Therefore it has to be unique. In this case it is `upper_body_detector`
* The `topic` parameter specifies the topic under which the detections are published. The type has to be `geometry_msgs/PoseArray`. See `to_pose_array` in strands_perception_people_utils/README.md if your detector does not publish a PoseArray.
* The `noise_model` parameter is used for the Kalman Filter (currently: Extended Kalman Filter).
 * `velocity` specifies the standard deviation in x and y direction of the constant velocity model in meters per second.
 * `position` specifies the standard deviation of x and y in the cartesian model in meters.
* `matching_algorithm` specifies the algorithm used to match detections from different sensors/detectors. Currently there are three different algorithms which are based on the Mahalanobis distance of the detections (default being NNJPDA if parameter is misspelled):
 * NN: Nearest Neighbour
 * JPDA: Joint Probability Data Association
 * NNJPDA: NN + JPDA

All of these are just normal ROS parameters and can be either specified by the parameter server or using the yaml file in the provided launch file.

### Message Type:
The PeopleTracker Message can be found in the `starnds_perception_people` package in the `strands_msgs` repository:

```
std_msgs/Header header
string[] uuids             # Unique uuid5 (NAMESPACE_DNS) person id as string. Id is based on system time on start-up and tracker id. Array index matches ids array index
geometry_msgs/Pose[] poses # The real world poses of the detected people in the given target frame. Default: /map. Array index matches ids/uuids array index
float64[] distances        # The distances of the detected persons to the robot (polar coordinates). Array index matches ids array index.
float64[] angles           # Angles of the detected persons to the coordinate frames z axis (polar coordinates). Array index matches ids array index.
float64 min_distance       # The minimal distance in the distances array.
float64 min_distance_angle # The angle according to the minimal distance.
```

The poses will be published in a given `target_frame` (see below) but the distances and angles will always be relative to the robot in the `/base_link` tf frame.

### Running
Parameters:

* `target_frame`: _Default: /base_link_:the tf frame in which the tracked poses will be published. 
* `position`: _Default: /people_tracker/positions_: The topic under which the results are published as strands_perception_people_msgs/PeopleTracker`
* `marker`: _Default /people_tracker/marker_array_: A visualisation marker array.

You can run the node with:

```
roslaunch strands_people_tracker people_tracker.launch
```

This is the recommended way of launching it since this will also read the config file and set the right parameters for the detectors.
