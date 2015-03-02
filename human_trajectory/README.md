# Human Trajectory
This is a ROS package that extracts human poses information from bayes_people_tracker_logging and stitches each pose for each human together. This package be run online by subscribing to /people_tracker/positions or offline by retrieving human poses from perception_people 

Run this package by typing 

```
rosrun human_trajectory trajectory.py [publish_interval] [online/offline(1/0)] [incremental/complete(1/0)]
```

where ```[publish_interval]``` is the interval (in second) between complete/incremental trajectories to be published in each published message. 
```[online/offline]``` is the choice to stitch human poses online (1) from other packages (bayes_people_tracker) or offline (0) from perception_people collection in mongodb.
```[incremental/complete]``` is the enabled choice when online sticthing is chosen. 
Incremental online stitching (1) provides trajectories messages incrementally when
human poses are available for each individual person, whereas complete stitching (0) only publishes complete trajectory messages for particular persons. 
However, both incremental and complete online stitching only stores complete trajectories to the database.
