std_msgs/Header header
string[] uuids                             # Unique uuid5 (NAMESPACE_DNS) person id as string. Id is based on system time on start-up and tracker id. Array index matches ids array index
geometry_msgs/Pose[] people                # The real world poses of the detected people in the given target frame. Default: /map. Array index matches ids/uuids array index
geometry_msgs/Pose robot                   # The last received robot pose
bayes_people_tracker/PeopleTracker people_tracker               # Output of the people_tracker
mdl_people_tracker/MdlPeopleTracker[] mdl_people_tracker   # Output of the people tracker
upper_body_detector/UpperBodyDetector upper_body_detections    # Ouput of the upper body detector
geometry_msgs/Transform target_frame       # The transformation used to optain real world coordinates of detections. Default /map
geometry_msgs/Transform base_link          # The transformation used to optain distances and angles relative to the robot. /head_xtion_depth_optical_frame to /base_link
