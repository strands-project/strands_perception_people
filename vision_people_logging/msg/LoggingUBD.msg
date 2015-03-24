std_msgs/Header header
geometry_msgs/Pose robot                   # The robot's position in world-coordinates.
geometry_msgs/Point[] ubd_pos              # The UBDs in world-coordinates.
upper_body_detector/UpperBodyDetector ubd  # Ouput of the upper body detector
sensor_msgs/Image[] ubd_rgb                # RGB cutouts of the UBDs, 3x height.
sensor_msgs/Image[] ubd_d                  # Depth cutouts of the UBDs, 3x height.
