# Human Trajectory
This is a ROS package that extracts human poses information from bayes_people_tracker_logging and stitches each pose for each human together. This package be run online by subscribing to /people_tracker/positions or offline by retrieving human poses from perception_people 

Run this package by typing 

```
rosrun human_trajectory trajectory.py [publish_interval] [online/offline(1/0)]
```

where ```[publish_interval]``` is the interval (in second) between complete/incremental trajectories to be published in each published message. 
```[online/offline]``` is the choice to stitch human poses online (1) from other packages (bayes_people_tracker) or offline (0) from perception_people collection in mongodb.
The online stitching provides both mini batch trajectories messages where trajectories are split into chunked and complete batch trajectories which only publishes complete trajectory messages for particular persons. 
