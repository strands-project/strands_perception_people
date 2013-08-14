## groundHOG
This package uses cuda to extract HOG features and detect persons just using the RGB image of the Kinect.

### Dependencies
* libcudaHOG: see strands_preception_people/3rd_party
_If this dependency is not met, the package will still compile but won't have any functionality. This is meant to not break the overall build process on systems without a NVIDIA graphics card._

### Run
Parameters:
* `visualise` _default = false_: Set to true to render and publish result images.
* `queue_size` _default = 20_: The synchronisation queue size
* `model` _default = ""_: The model file, can be found in model/config
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
