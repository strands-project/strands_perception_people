std_msgs/Header header
string uuid                             # human id
geometry_msgs/PoseStamped[] trajectory  # human trajectory
geometry_msgs/Pose[] robot              # robot's trajectory
time start_time                         # time for the first detected pose
time end_time                           # time for the last detected pose
float32 trajectory_length                 # in meters 
bool complete                           # complete or incremental trajectory
int32 sequence_id                         # sequence id if incremental trajectory is choosen 
