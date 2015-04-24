# Human Trajectory
This is a ROS package that extracts human poses information from people tracker and stitches each pose for each human together. This package be run online by subscribing to /people_tracker/positions or offline by retrieving human poses from perception_people 

Run this package by typing 

```
roslaunch human_trajectory human_trajectory.launch
```
The online stitching provides both mini batch trajectories messages where trajectories are split into chunked and complete batch trajectories which only publishes complete trajectory messages for particular persons. 

## Parameters
* `with_logging_manager`: the option (true/false) to subscribe to logging manager and get
  permission to store obtained data
* `path_visualisation`:  the option to visualise each detected trajectory in rviz using Path. However, it only works for online construction. 
* `online_construction`: the option (true/false) to stitch human poses online from other packages (bayes_people_tracker) or offline from perception_people or people_trajectory collection in mongodb.
* `tracker_topic`: the name of the people tracker topic, default is /people_tracker/positions
* `logging_manager_topic`: the name of the logging manager topic, default is /logging_manager/log_stamped
