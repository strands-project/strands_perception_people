Head orientation estimator
==========================

This package estimates the orientation of the head of people detected by the upper body detector.

Install
=======
Please download and extract the model file into the `models` folder. By default:

```
cd strands_head_orientation/models
wget http://lucasb.eyer.be/academic/face_orientation/model-default.txz
tar xJv < model-default.txz
```

This should create the `models/default` folder with a bunch of files.

Run
===

Dependencies
------------

This node needs `strands_upper_body_detector/upper_body_detector.launch` to run,
which in turn needs `strands_ground_plane/ground_plane_fixed.launch`.
Or just run the people tracker which starts both of the above.

Parameters
----------

* `queue_size` *default = 10*: The synchronisation queue size
* `camera_namespace` *default = /head_xtion*: The camera namespace.
* `upper_body_detections` *default = /upper_body_detector/detections*: The deteced upper bodies
* `head_ori` *default = /head_orientation/head_ori*: Publishing an orientation array for each detected upper body.
* `model_dir` *default = ""*: The learned model to use to do predictions. Models can be found in `strands_head_orientation/models/`.
* `autostart` *default = true*: Whether to start analyzing detections right away or wait for a signal.

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

Start/stop
----------

This node should be permanently running. While running, it can be started and
stopped and when stopped it should hardly use any resources, except for some
memory.

Its activity can be controlled on-demand through calls to the following services:
  - `start_head_analysis`: starts the analysis of all incoming detections.
  - `stop_head_analysis`: stops the analysis of any detections.
  - `status_head_analysis`: returns whether the analysis is currently started.

For manual usage, you can use the `start`, `stop` and `status` executables:

```
rosrun strands_head_orientation start
rosrun strands_head_orientation status
rosrun strands_head_orientation stop
```

