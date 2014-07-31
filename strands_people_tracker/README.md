## Pedestrian Localisation
This package takes the pedestrian tracking results and transforms them into the /base_link coordinate system. It also identifies the closest humand and gives distance and angle in polar coordinates.

### Run
Parameters:
* `target_frame` _default = "/base_link"_: The coordinate system in which to transform
* `pedestrian_array` _default = "/pedestrian_tracking/pedestrian_array"_: The tracking results
* `localisations` _default = /pedestrian_localisation/localisations_: The localisation results
* `marker` _default = /pedestrian_localisation/marker_array_: The markler array to visualise humans in rviz


rosrun:
```
rosrun strands_pedestrian_localisation pedestrian_localisation [_parameter_name:=value]
```

roslaunch:
```
roslaunch strands_pedestrian_localisation pedestrian_localisation.launch [parameter_name:=value]
```
