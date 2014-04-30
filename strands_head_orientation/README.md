Head orientation estimator
==========================

This package estimates the orientation of the head of people detected by the upper body detector.

Run
===

Parameters
----------

* `queue_size` *default = 10*: The synchronisation queue size
* `camera_namespace` *default = /head_xtion*: The camera namespace.
* `upper_body_detections` *default = /upper_body_detector/detections*: The deteced upper bodies
* `head_ori` *default = /head_orientation/head_ori*: Publishing an orientation array for each detected upper body.
* `model_dir` *default = ""*: The learned model to use to do predictions. Models can be found in `strands_head_orientation/models/`.

rosrun
------
```
rosrun strands_head_orientation strands_head_orientation [_parameter_name:=value]
```

roslaunch
---------
```
roslaunch strands_head_orientation head_orientation.launch [parameter_name:=value]
```
