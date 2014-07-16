std_msgs/Header header #The header taken from the pedestrian localosation msg
string[] ids #Unique uuid5 (NAMESPACE_DNS) person id as string. Id is based on system time on start-up and tracker id.
geometry_msgs/Pose[] people #The real world poses of the detected people in the given target frame. Default: /base_link. Array index matches ids array index
geometry_msgs/Pose robot #The last received robot pose
float64[] scores #The confidence score of the tracker. Array index matches ids array index.
float64[] distances #The distances of the detected persons to the robot (polar coordinates). Array index matches ids array index.
float64[] angles #Angles of the detected persons to the coordinate frames z axis (polar coordinates). Array index matches ids array index.
float64 min_distance #The minimal distance in the distances array.
float64 min_distance_angle #The angle according to the minimal distance.
PedestrianTracking[] pedestrian_tracking #Output of the pedestrian tracker
UpperBodyDetector upper_body_detections #Ouput of the upper body detector
geometry_msgs/Transform tf #The transformation used to optain real world coordinates of detections
