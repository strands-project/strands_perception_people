^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^
Changelog for package mdl_people_tracker
^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^

0.0.14 (2014-11-23)
-------------------

0.0.12 (2014-10-31)
-------------------
* Making camera topic reconfigurable
  So far only the camera namespace was configurable but that introduced an implicit dependency on the openni_wrapper.
  With these changes the whole topic is reconfigurable via a parameter, e.g.:
  camera_namespace:=/my_cam
  depth_image:=/depth/image
  results in `/my_cam/depth/image` as a topic for the depth image. So `camera_namespace` + `depth_image` = the topic on which to look for the depth image.
* Contributors: Christian Dondrup

0.0.3 (2014-10-23)
------------------
* Added LICENSE files. Fixes `#101 <https://github.com/strands-project/strands_perception_people/issues/101>`_
* Contributors: Lucas Beyer

0.0.2 (2014-10-18)
------------------

0.0.1 (2014-10-18)
------------------
* removed groundHOG references from code.
* renaming mdl-People_tracker launch files
  to comply with the rest of the structure and to make releasing easier.
* Renamed strands_pedestrian_tracking to mdl_people_tracker
  This also includes renaming the messages and most of the parameters.
* Contributors: Christian Dondrup
