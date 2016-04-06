# DROW detector

This is the DROW detector package (http://arxiv.org/abs/1603.02636). Given a laser scan it will detect two types of mobility aids and publish these as a pose array.

As input only a laser scan is needed (of type `LaserScan`). Currently there are three output topics (all of type `PoseArray`), two class specific ones for wheelchairs and walkers and one class agnostic one, that summarizes both topics. Our pretrained model was trained based on a Laser scanner with a field of view of 225 degrees and an angular resolution of 0.5 degrees at a height of ~37cm. The model will generalize to different field of views and should be robust to slight height deviations. In order to apply it to different angular resolutions or completely different heights you will need to train it on your own data though. See the paper for further info.



## Params
* `laser_topic` default: `/scan` the input laser scan.
* `wheelchair_detection_topic` default: `/wheelchair_detections` the topics wheelchair detections will be published on.
* `walker_detection_topic` default: `/walker_detections` the topics walker detections will be published on.
* `class_agnostic_detection_topic` default: `/mobility_aid_detections` the topic where both types of mobility aid detections will be published to jointly.
* `threshold` default: `0.5` the detection threshold. This will increase the precision at the cost of recall and vice versa.
* `use_cudnn`, default: `false` determines if we force cudnn for the convolutions. With is faster, but harder to setup. (see dependencies)
* `network_param_file` no default, the path to the network parameter file.

## Dependencies
* A decent GPU (the ones on our side pcs will do :D)
* CUDA (tested with 7.5 download here https://developer.nvidia.com/cuda-downloads)
* CUDNN (HIGHLY RECOMMENDED. Without this the detector will not go beyond 4fps, while with it, it has no problems keeping up with the laser frequency. You need to register for this, download the archive, put it somewhere and export the correct paths for theano to find http://deeplearning.net/software/theano/library/sandbox/cuda/dnn.html)


## Running the detector
In order to run the detector, make sure all dependencies are installed and download the network model from our server.
* `$ roscd wheelchair_detector`
* `$ cd scripts`
* `$ sh get_network_parameter_file.sh`
The launch file is configured in such a way that it will look for the network in the resources folder, but you can put it anywhere you like it to be as long as you change the launch file accordingly.

In order to launch the node, you will need to provide the theano flags:
* `$ THEANO_FLAGS='cuda.root=/usr/local/cuda/,floatX=float32,device=gpu0' roslaunch wheelchair_detector drow.launch`
If you have cuda somewhere else, make sure to adapt the flag. All of these can also be set in a [theano config](http://deeplearning.net/software/theano/library/config.html) once instead.

## TODO
* Make the voting parameters public. This is especially important for lasers with a larger FoV.
* Add a project page with training code, data, etc.
* Change the name to DROW?