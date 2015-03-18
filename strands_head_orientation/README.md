Head orientation estimator
==========================

This package estimates the orientation of the head of people detected by the upper body detector.

For the G4S Y1 deployment
=========================

For the G4S Y1 deployment, we won't run the actual estimation, we'll only run
logging code and that'll run 24/7, but can be paused/resumed by service calls.

Everything is logged into the `heads` collection.

Launching
---------

Use the g5s launchfile:

```
roslaunch strands_head_orientation g4s.launch
```

Directly edit the parameters in the launchfile if they need adapting.
Especially, please check `expected_runtime` and modify it if the expected
runtime is *not* 3 weeks.

There is a hardcoded (by intent) limit of detection and image count to be logged
(`g_maxStored*` in [store_detections.cpp](src/store_detections.cpp)) which
amounts to roughly 40Gb and 10Gb, respectively.

Pausing and resuming
--------------------

In total, ~1 CPU core is used by this node and its dependencies. It can be paused
and later on resumed if all available power is needed by someone else.

### Pausing

Send a message of type `strands_head_orientation/StopHeadAnalysis` to the
`stop_head_analysis` service and all activity will be paused.

The `stop` executable in this package does exactly that, see its [source](src/stop.cpp)
for an example or just execute it with `recording` as parameter:

```
rosrun strands_head_orientation stop recording
```

### Resuming

For resuming from the paused state, send a message of type
`strands_head_orientation/StartHeadAnalysis` to the `start_head_analysis`
service.

Again, the `start` executable and its [source](src/start.cpp) can be helpful:

```
rosrun strands_head_orientation start recording
```

### Poking

You can also check for the current state by sending a message of type
`strands_head_orientation/IsHeadAnalysisRunning` to the `status_head_analysis`
service.

Yet again, the `status` executable and its [source](src/status.cpp) help you:

```
rosrun strands_head_orientation status recording
```

Anything below this line can be ignored for the g4s scenario.

---------------------------------------

Install
=======

Please download and extract one of the [available model files](https://omnomnom.vision.rwth-aachen.de/strands/data/ghmodels-l/) into the `models` folder. The larger models will have better prediction but will run slower. By default:

```
cd strands_head_orientation/models
wget https://omnomnom.vision.rwth-aachen.de/strands/data/ghmodels-l/model-small.tar.bz2
tar xjv < model-small.tar.bz2
ln -s default-0.1 default
```

This should create the `models/default` link pointing to the `models/default-0.1` folder with a bunch of files.

Run
===

Dependencies
------------

This node needs `upper_body_detector/upper_body_detector.launch` to run,
which in turn needs `ground_plane_estimation/ground_plane_fixed.launch`.
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

