# Human Trajectory
This is a ROS package that extracts human poses information from bayes_people_tracker_logging and stitches each pose for each human together. For now, this package runs offline meaning that the poses are taken from a database. The trajectories are stored into a database and published via ROS message.

Run this package by typing 

```
rosrun human_trajectory trajectory.py [publish_interval]
```

where ```[publish_interval]``` is the interval (in second) for between complete trajectories to be published in each published message. 
