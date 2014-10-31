^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^
Changelog for package upper_body_detector
^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^

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
* Merge branch 'hydro-devel' into hydro-release
  Conflicts:
  opencv_warco/package.xml
  strands_head_orientation/CMakeLists.txt
  strands_head_orientation/package.xml
* Added LICENSE files. Fixes `#101 <https://github.com/strands-project/strands_perception_people/issues/101>`_
* Contributors: Christian Dondrup, Lucas Beyer

0.0.2 (2014-10-18)
------------------

0.0.1 (2014-10-18)
------------------
* Prepared upper_body_detector for release.
* Renamed strands_upper_body_detector to upper_body_detector
* Addressing issue `#15 <https://github.com/strands-project/strands_perception_people/issues/15>`_
* Closing issue `#10 <https://github.com/strands-project/strands_perception_people/issues/10>`_ and `#12 <https://github.com/strands-project/strands_perception_people/issues/12>`_ and addresses issue `#16 <https://github.com/strands-project/strands_perception_people/issues/16>`_.
* Closing issue `#7 <https://github.com/strands-project/strands_perception_people/issues/7>`_
  Added tracking into repository
* Closing Issue `#6 <https://github.com/strands-project/strands_perception_people/issues/6>`_
  Added upperbody detector to the project
* Contributors: Christian Dondrup, Dennis Mitzel, cdondrup
