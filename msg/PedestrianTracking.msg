Header header
# position projected on the GP in world coordinates
float64[] traj_x 
float64[] traj_y 
float64[] traj_z
# position projected on the GP in camera coordinates
float64[] traj_x_camera 
float64[] traj_y_camera 
float64[] traj_z_camera 
float64[] dir           # Orientation of person for every point in traj
float64 speed           # Speed of detected person
int64 id                # Tracking id. Will be reset to 0 after restart
string uuid             # Unique uuid5 (NAMESPACE_DNS) person id as string. Id is based on system time on start-up and id.
float64 score           # Tracker confidence
