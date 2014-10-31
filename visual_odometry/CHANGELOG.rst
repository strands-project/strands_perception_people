^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^
Changelog for package visual_odometry
^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^

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
* Damn you Copy&Paste
  Trying to hide my stupidity.
* Contributors: Christian Dondrup

0.0.10 (2014-10-30)
-------------------
* Adding tf following a suggestion on ros answers to solve the linker error.
* Contributors: Christian Dondrup

0.0.9 (2014-10-30)
------------------
* Stupid mistake including cmake_modules in catkin_depends
* Contributors: Christian Dondrup

0.0.8 (2014-10-30)
------------------
* eigen still has to be in the package.xml as a build_dependency even though the migration instructions don't suggest that.
* Contributors: Christian Dondrup

0.0.7 (2014-10-29)
------------------

0.0.6 (2014-10-29)
------------------
* Eigen is now part of cmake_modules
* Contributors: Christian Dondrup

0.0.5 (2014-10-29)
------------------

0.0.4 (2014-10-29)
------------------

0.0.3 (2014-10-23)
------------------

0.0.2 (2014-10-18)
------------------

0.0.1 (2014-10-18)
------------------
* Some bug fixes
* Prepared visual_odometry for release.
* strands_visual_odometry is now visual_odometry
* Addressing issue `#15 <https://github.com/strands-project/strands_perception_people/issues/15>`_ and `#16 <https://github.com/strands-project/strands_perception_people/issues/16>`_
* Closing issue `#7 <https://github.com/strands-project/strands_perception_people/issues/7>`_
  Added tracking into repository
* cv_bridge solved the problem with the depth image
* Fixes issue `#1 <https://github.com/strands-project/strands_perception_people/issues/1>`_.
  Also fixes a bug where the _msgs at the end of strands_perception_people was missing.
* Not needed anymore. Moved to msgs package with previous commit.
* Moved VO msg to perception_people_msgs package.
* removed endline at end of file
* Initial commit of ros node for visual odometry.
* Added 3rdparty fovis files.
* Contributors: Christian Dondrup, Dennis Mitzel, cdondrup
