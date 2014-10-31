^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^
Changelog for package perception_people_launch
^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^

0.0.13 (2014-10-31)
-------------------
* Making camera topic reconfigurable
  So far only the camera namespace was configurable but that introduced an implicit dependency on the openni_wrapper.
  With these changes the whole topic is reconfigurable via a parameter, e.g.:
  camera_namespace:=/my_cam
  depth_image:=/depth/image
  results in `/my_cam/depth/image` as a topic for the depth image. So `camera_namespace` + `depth_image` = the topic on which to look for the depth image.
* Contributors: Christian Dondrup

0.0.11 (2014-10-30)
-------------------

0.0.10 (2014-10-30)
-------------------

0.0.9 (2014-10-30)
------------------

0.0.8 (2014-10-30)
------------------

0.0.7 (2014-10-29)
------------------

0.0.6 (2014-10-29)
------------------

0.0.5 (2014-10-29)
------------------
* The people_msgs package does not exist in indigo yet
* Contributors: Christian Dondrup

0.0.4 (2014-10-29)
------------------
* Removing the leg_detector from the run_dependencies of the launch package for indigo release.
  leg_detector is not released for indigo yet.
* Contributors: Christian Dondrup

0.0.3 (2014-10-23)
------------------
* Added LICENSE files. Fixes `#101 <https://github.com/strands-project/strands_perception_people/issues/101>`_
* Contributors: Lucas Beyer

0.0.2 (2014-10-18)
------------------
* Removed ground_hog from run_deps
* Contributors: Christian Dondrup

0.0.1 (2014-10-18)
------------------
* removed groundHOG references from code.
* renaming mdl-People_tracker launch files
  to comply with the rest of the structure and to make releasing easier.
* Renamed strands_pedestrian_tracking to mdl_people_tracker
  This also includes renaming the messages and most of the parameters.
* Some bug fixes
* Prepared launch package for release.
* Renamed pedestrian_tracker launch files
* Calling the leg_detector directly to not need a private fork anymore.
* Splitting utils package into seperate packages.
* strands_visual_odometry is now visual_odometry
* strands_perception_people_launch is now perception_people_launch
* Contributors: Christian Dondrup
