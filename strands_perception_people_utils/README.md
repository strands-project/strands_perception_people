## People perception utils package
This packge contains useful utils for the people perception like the conversion of the robots odometry to a motion matrix to substitude the visual odometry and a logging node to save the detections to the message_store.

All the information given on how to run the nodes should only be used if you need to run them seperately. In normal cases please refer to the `strands_perception_people_launch` package to start the whole perception pipeline.

### odom2visual
This node creates a motion matrix from the robots odometry using the Eigen library to substitude the visual odometry.

Run with:

`roslaunch strands_perception_people_utils odom2visual.launch`

Parameters:
* `odom`: _Default: /odom_ The topic on which the robots odometry is published
* `motion_parameters`: _Default: /visual_odometry/motion_matrix_ The topic on which the resulting motion matrix is published

### Logging
This node uses the `strands_msgs/strands_perception_people_msgs/msg/Logging.msg` to save the detected people together with their realworld position, the robots pose, the upper body detector and pedestrian tracker results, and the tf transform used to create the real world coordinates in the message store.

Run with:

`roslaunch strands_perception_people_utils logging.launch`

Parameters:
* `log`: _Default: true_ This convenience parameter allows to start the whole system without logging the data
