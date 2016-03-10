# DROW detector

This is the DROW detector package (http://arxiv.org/abs/1603.02636). Given a laser scan it will detect two types of mobility aids and publish these as a pose array.

As input only a laser scan is needed (of type `LaserScan`). Currently there are three output topics (all of type `PoseArray`), two class specific ones for wheelchairs and walkers and one class agnostic one, that summarizes both topics. Our pretrained model was trained based on a Laser scanner with a field of view of 225 degrees and an angular resolution of 0.5 degrees at a height of ~37cm. The model will generalize to different field of views and should be robust to slight height deviations. In order to apply it to different angular resolutions or completely different heights you will need to train it on your own data though. See the paper for further info.



## Params
* `laser_topic` default: `\scan` the input laser scan.
*

    wheelchair_detection_topic = rospy.get_param(ns + 'wheelchair_detection_topic', '/wheelchair_detections')
    walker_detection_topic = rospy.get_param(ns + 'walker_detection_topic', '/walker_detections')
    class_agnostic_detection_topic = rospy.get_param(ns + 'class_agnostic_detection_topic', '/mobility_aid_detections')
    global threshold
    threshold = rospy.get_param(ns + 'detection_threshold', 0.5)

    #Load the network
    global network
    network_file = rospy.get_param(ns + 'network_param_file')

## Dependencies
* A decent GPU
* CUDA
* CUDNN


## Running the detector
In order to run the detector, we need to setup a virtual environment, install some software and download the network model.

Once in order to get the model and setup the env do the following:
    * `$ roscd wheelchair_detector`
    * `$ virtualenv --system-site-packages ros_dl_env`
    * `$ pip install Theano`
    * `$ pip install git+https://github.com/lucasb-eyer/DeepFried2.git`
    * `$ cd scripts`
    * `$ source get_network_parameter_file.sh`


In order to launch the node, you will need to source the environment and launch the node:
    * `$ roscd wheelchair_detector`
    * `$ source ros_dl_env/bin/activate`
    * `$ roslaunch wheelchair_detector drow.launch`

## TODO
* Find a cleaner way to install Theano and DeepFried2 and add depenencies to CUDA and CUDNN.
* Make the voting parameters public. This is especially important for lasers with a larger FoV.
* Add a project page with training code, data, etc.
* Change the name to DROW?
* Make a proper setup.py to take care of some of the above issues.