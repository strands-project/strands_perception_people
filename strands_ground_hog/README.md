## groundHOG
This package uses cuda to extract HOG features and detect persons just using the RGB image of the Kinect.

### Dependencies
* libcudaHOG: see strands_preception_people/3rd_party
_If this dependency is not met, the package will still compile but won't have any functionality. This is meant to not break the overall build process on systems without a NVIDIA graphics card._

### Run
Parameters:
* `load_params_from_file` _default = true_: `false` tries to read parameters from datacentre, `true` reads parameters from YAML file specified by `param_file`
* `param_file` _default = $(find strands_ground_hog)/config/ground_hog.yaml_: The config file containing all the essential parameters. Only used if `load_params_from_file == true`.
* `machine` _default = localhost_: Determines on which machine this node should run.
* `user` _default = ""_: The user used for the ssh connection if machine is not localhost.
* `queue_size` _default = 20_: The synchronisation queue size
* `image_color` _default = /camera/rgb/image_color_: The Kincet colour image
* `camera_info` _default = /camera/rgb/camera_info_: The Kinect camera info
* `ground_plane` _default = ""_: The ground plane. Published by upper_body_detector. Will only be used if set. Is used to track speed up detections.
* `detections` _default = /groundHOG/detections_: The generated data output topic
* `result_image` _default = /groundHOG/image_: The generated image output topic showing the detections as boundingboxes.

rosrun:
```
rosrun strands_ground_hog groundHOG [_parameter_name:=value]
```

roslaunch:
* Using only the RGB image: `roslaunch strands_ground_hog ground_hog.launch [parameter_name:=value]`
* Using ground plane to enhance detection `roslaunch strands_ground_hog ground_hog_with_GP.launch [parameter_name:=value]`

### Troubleshooting
If this node is launched remotely it might happen that libcudart and libcuda cannot be found and therefore the node fails to start. To prevent this, make sure that the path to the cuda/lib64 directory is in the `LD_LIBRARY_PATH` of the machine from which the node is started (See 3rdParty README on how to install cuda and libcudaHOG). ssh does not surce the `.bashrc` on the remote machine so the environment varaibaleas have to be read from the current machine and sent to the remote machine.
