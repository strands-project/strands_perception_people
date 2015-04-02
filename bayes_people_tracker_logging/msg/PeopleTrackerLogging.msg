std_msgs/Header header
string[] uuids                      # Unique uuid5 (NAMESPACE_DNS) person id as string. Id is based on system time on start-up and tracker id. Array index matches ids array index
geometry_msgs/PoseStamped[] people  # The real world poses of the detected people in the given target frame. Default: /map. Array index matches ids/uuids array index
geometry_msgs/Pose robot            # The last received robot pose
bayes_people_tracker/PeopleTracker people_tracker # Output of the people_tracker
