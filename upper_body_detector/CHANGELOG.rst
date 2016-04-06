^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^
Changelog for package upper_body_detector
^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^

1.5.0 (2016-03-15)
------------------

1.4.0 (2016-02-17)
------------------

1.3.1 (2016-02-11)
------------------

1.3.0 (2016-02-01)
------------------

1.2.1 (2016-01-28)
------------------
* Making the bbox red again.
* Changing default topic name for use with openni2
* Contributors: Christian Dondrup

1.2.0 (2015-11-11)
------------------
* Make UBD viz padding-aware and not share ROS buffer.
* UBD viz only works for rgb8 images.
* Contributors: lucasb-eyer

1.1.8 (2015-09-03)
------------------

1.1.6 (2015-06-24)
------------------
* 1.1.5
* updated changelogs
* 1.1.4
* updated changelogs
* Contributors: Jenkins

1.1.5 (2015-05-22)
------------------

1.1.4 (2015-05-10)
------------------

1.1.3 (2015-04-10)
------------------

1.1.2 (2015-04-07)
------------------

1.1.1 (2015-04-03)
------------------

1.0.0 (2015-03-10)
------------------

0.1.4 (2015-03-06)
------------------

0.1.3 (2015-02-25)
------------------
* Fix for boundig box scaling issue.
* Contributors: Pandoro

0.1.1 (2015-02-18)
------------------

0.1.0 (2015-02-18)
------------------
* Setting correct version number. The changelogs will be regenerated because the ones from the release branch would not be consistent with the changes made in the devel branch.
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
* 0.0.10
* Updating changelog
* 0.0.9
* Updated changelogs
* 0.0.8
* Updated changelogs
* 0.0.7
* Updated changelogs
* 0.0.6
* Updated changelogs
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
* 0.0.10
* Updating changelog
* 0.0.9
* Updated changelogs
* 0.0.8
* Updated changelogs
* 0.0.7
* Updated changelogs
* 0.0.6
* Updated changelogs
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
* Contributors: Christian Dondrup

0.0.10 (2014-10-30 10:19)
-------------------------
* Updating changelog
* Contributors: Christian Dondrup

0.0.9 (2014-10-30 09:52)
------------------------
* Updated changelogs
* Contributors: Christian Dondrup

0.0.8 (2014-10-30 09:32)
------------------------
* Updated changelogs
* Contributors: Christian Dondrup

0.0.7 (2014-10-29 20:40)
------------------------
* Updated changelogs
* Contributors: Christian Dondrup

0.0.6 (2014-10-29 20:32)
------------------------
* Updated changelogs
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
* Merge branch 'hydro-devel' into hydro-release
  Conflicts:
  opencv_warco/package.xml
  strands_head_orientation/CMakeLists.txt
  strands_head_orientation/package.xml
* Added LICENSE files. Fixes `#101 <https://github.com/strands-project/strands_perception_people/issues/101>`_
* Contributors: Christian Dondrup, Lucas Beyer

0.0.2 (2014-10-18 17:39)
------------------------
* Updated changelog
* Contributors: Christian Dondrup

0.0.1 (2014-10-18 17:28)
------------------------
* Created changelogs
* Prepared upper_body_detector for release.
* Renamed strands_upper_body_detector to upper_body_detector
* Addressing issue `#15 <https://github.com/strands-project/strands_perception_people/issues/15>`_
* Closing issue `#10 <https://github.com/strands-project/strands_perception_people/issues/10>`_ and `#12 <https://github.com/strands-project/strands_perception_people/issues/12>`_ and addresses issue `#16 <https://github.com/strands-project/strands_perception_people/issues/16>`_.
* Closing issue `#7 <https://github.com/strands-project/strands_perception_people/issues/7>`_
  Added tracking into repository
* Closing Issue `#6 <https://github.com/strands-project/strands_perception_people/issues/6>`_
  Added upperbody detector to the project
* Contributors: Christian Dondrup, Dennis Mitzel, cdondrup
