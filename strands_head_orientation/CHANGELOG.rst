^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^
Changelog for package strands_head_orientation
^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^

1.3.1 (2016-02-11)
------------------

1.3.0 (2016-02-01)
------------------

1.2.1 (2016-01-28)
------------------

1.2.0 (2015-11-11)
------------------

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

0.1.1 (2015-02-18)
------------------
* Refactoring opencv_warco to be able to build strands_head_orientation from install and devel.
* Contributors: Christian Dondrup

0.1.0 (2015-02-18)
------------------
* Setting correct version number. The changelogs will be regenerated because the ones from the release branch would not be consistent with the changes made in the devel branch.
* Prepared strands_hed_orientation for release
* Changed launch files to new format.
* Changed launch files to new format.
* Contributors: Christian Dondrup

0.0.14 (2014-11-23)
-------------------
* Removed packages that are not supposed to be released.
* Contributors: Christian Dondrup

0.0.13 (2014-10-31 16:14)
-------------------------
* Removed packages that are not supposed to be released.
* Contributors: Christian Dondrup

0.0.12 (2014-10-31 16:07)
-------------------------
* Removed packages that are not supposed to be released.
* Contributors: Christian Dondrup

0.0.11 (2014-10-30 11:18)
-------------------------

0.0.10 (2014-10-30 10:19)
-------------------------

0.0.9 (2014-10-30 09:52)
------------------------

0.0.8 (2014-10-30 09:32)
------------------------

0.0.7 (2014-10-29 20:40)
------------------------

0.0.6 (2014-10-29 20:32)
------------------------

0.0.5 (2014-10-29 18:30)
------------------------

0.0.4 (2014-10-29 18:22)
------------------------

0.0.3 (2014-10-23)
------------------
* Added LICENSE files. Fixes `#101 <https://github.com/strands-project/strands_perception_people/issues/101>`_
* Make the compiler happy.
  (Missing upper_body_detector dependency specification.)
* Author/license info.
* Make the linker happy.
  Longer explanation: the mongodb_store links to static libmongoclient.a which needs dynamic libboost-filesystem-mt.so.
  The problem is another component links to that boost one too, earlier. So CMake doesn't add that into the linker line
  later when mongodb_store kicks in. Moving it to the front fixes it, even though it's not clean.
* Removing mongoclient from linked libraries
* Removing mongodb_store_cpp_client dependency
* Contributors: Lucas Beyer, cdondrup

0.0.2 (2014-10-18 17:39)
------------------------

0.0.1 (2014-10-18 17:28)
------------------------
* Removed packages that are not supposed to be released.
* Renamed strands_upper_body_detector to upper_body_detector
* Renamed strands_ground_plane to ground_plane_estimation
* Moving messages into respective packages
* Replaced ros_datacentre with mongodb_store
* Implemented log-rate computation and added a hard(-coded) limit.
* Added g4s-specific info+launchfile.
* Added logging to the ros_database.
* Adding machine tags to launch files.
* Added visualization/debugging output to heads_ori.
* Merge branch 'hydro-devel' of github.com:strands-project/strands_perception_people into hydro-devel
* Added services for start/stop/status to head_ori.
* Documented deps of head_orientation.
* Updated README about model download.
* Shame on me!
* Cut the ROI to the actual image. Maybe.
* Added more certainty!
* Initial opencv-warco based head pose estimator.
* Contributors: Christian Dondrup, Nick Hawes, cdondrup, lucasb-eyer, strands G5
