# Human Trajectory
This is a ROS package that extracts human poses information from bayes_people_tracker_logging and stitches each pose for each human together. For now, this package runs offline meaning that the poses are taken from a database. The trajectories can be stored into a database or be published via ROS message.

Run this package by typing 

```
rosrun human_trajectory trajectory.py [publish_interval] [store_or_publish]
```

where ```[publish_interval]``` is the interval you want to publish the data (up to 3600 seconds) and [store_or_publish] is whether you want to store the trajectories to a database (1) or to publish them via ROS message (0).
