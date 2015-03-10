^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^
Changelog for package visual_odometry
^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^

1.0.0 (2015-03-10)
------------------

0.1.4 (2015-03-06)
------------------

0.1.3 (2015-02-25)
------------------

0.1.1 (2015-02-18)
------------------

0.1.0 (2015-02-18)
------------------
* Setting correct version number. The changelogs will be regenerated because the ones from the release branch would not be consistent with the changes made in the devel branch.
* Update build files to indigo.
  (commit modified by Lucas during merge.)
  Conflicts:
  odometry_to_motion_matrix/CMakeLists.txt
  odometry_to_motion_matrix/package.xml
* Changed launch files to new format.
* Changed launch files to new format.
* Contributors: Christian Dondrup

0.0.14 (2014-11-23)
-------------------
* Updating changelogs and adjusting version numbers
* 0.0.12
* Adjusting version number.
* Updated changelogs
* 0.0.3
* Updated changelogs
* 0.0.2
* Updated changelog
* 0.0.1
* Created changelogs
* Contributors: Christian Dondrup

0.0.13 (2014-10-31 16:14)
-------------------------
* Updating changelogs and manually bumping version number.
* 0.0.11
* Updated changelogs
* Damn you Copy&Paste
  Trying to hide my stupidity.
* 0.0.10
* Updating changelog
* Adding tf following a suggestion on ros answers to solve the linker error.
* 0.0.9
* Updated changelogs
* Stupid mistake including cmake_modules in catkin_depends
* 0.0.8
* Updated changelogs
* eigen still has to be in the package.xml as a build_dependency even though the migration instructions don't suggest that.
* 0.0.7
* Updated changelogs
* 0.0.6
* Updated changelogs
* Eigen is now part of cmake_modules
* 0.0.5
* Updated changelogs
* 0.0.4
* Updating changelogs
* 0.0.3
* Updated changelogs
* 0.0.2
* Updated changelog
* 0.0.1
* Created changelogs
* 0.0.11
* Updated changelogs
* Damn you Copy&Paste
  Trying to hide my stupidity.
* 0.0.10
* Updating changelog
* Adding tf following a suggestion on ros answers to solve the linker error.
* 0.0.9
* Updated changelogs
* Stupid mistake including cmake_modules in catkin_depends
* 0.0.8
* Updated changelogs
* eigen still has to be in the package.xml as a build_dependency even though the migration instructions don't suggest that.
* 0.0.7
* Updated changelogs
* 0.0.6
* Updated changelogs
* Eigen is now part of cmake_modules
* 0.0.5
* Updated changelogs
* 0.0.4
* Updating changelogs
* Contributors: Christian Dondrup

0.0.12 (2014-10-31 16:07)
-------------------------
* Adjusting version number.
* Updated changelogs
* 0.0.3
* Updated changelogs
* 0.0.2
* Updated changelog
* 0.0.1
* Created changelogs
* Making camera topic reconfigurable
  So far only the camera namespace was configurable but that introduced an implicit dependency on the openni_wrapper.
  With these changes the whole topic is reconfigurable via a parameter, e.g.:
  camera_namespace:=/my_cam
  depth_image:=/depth/image
  results in `/my_cam/depth/image` as a topic for the depth image. So `camera_namespace` + `depth_image` = the topic on which to look for the depth image.
* Contributors: Christian Dondrup

0.0.11 (2014-10-30 11:18)
-------------------------
* Updated changelogs
* Damn you Copy&Paste
  Trying to hide my stupidity.
* Contributors: Christian Dondrup

0.0.10 (2014-10-30 10:19)
-------------------------
* Updating changelog
* Adding tf following a suggestion on ros answers to solve the linker error.
* Contributors: Christian Dondrup

0.0.9 (2014-10-30 09:52)
------------------------
* Updated changelogs
* Stupid mistake including cmake_modules in catkin_depends
* Contributors: Christian Dondrup

0.0.8 (2014-10-30 09:32)
------------------------
* Updated changelogs
* eigen still has to be in the package.xml as a build_dependency even though the migration instructions don't suggest that.
* Contributors: Christian Dondrup

0.0.7 (2014-10-29 20:40)
------------------------
* Updated changelogs
* Contributors: Christian Dondrup

0.0.6 (2014-10-29 20:32)
------------------------
* Updated changelogs
* Eigen is now part of cmake_modules
* Contributors: Christian Dondrup

0.0.5 (2014-10-29 18:30)
------------------------
* Updated changelogs
* Contributors: Christian Dondrup

0.0.4 (2014-10-29 18:22)
------------------------
* Updating changelogs
* Contributors: Christian Dondrup

0.0.3 (2014-10-23)
------------------
* Updated changelogs
* Contributors: Christian Dondrup

0.0.2 (2014-10-18 17:39)
------------------------
* Updated changelog
* Contributors: Christian Dondrup

0.0.1 (2014-10-18 17:28)
------------------------
* Created changelogs
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
