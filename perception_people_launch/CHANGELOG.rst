^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^
Changelog for package perception_people_launch
^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^

1.2.0 (2015-11-11)
------------------
* Adding new parameters to readme file.
* Adding people tracker filter to overall launch file. Is started by default using the map provided on `/map`.
  If not subscribed to, will not use any CPU.
* Contributors: Christian Dondrup

1.1.8 (2015-09-03)
------------------
* Adding parameter description to README
* Fixing config file location
* Adding laser filters to the launch file to get rid of false psoitives by the leg detector.
* Contributors: Christian Dondrup

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
* Adding logging manager topic to overall launch file
* Contributors: Christian Dondrup

1.1.2 (2015-04-07)
------------------

1.1.1 (2015-04-03)
------------------

1.0.0 (2015-03-10)
------------------
* * Publishing a pose array for all detected people to have more generic output
  * Added missing bayes tracker parameters to launch files and READMEs
  * Starting the mdl tracker is now optional when using the robot launch file. `with_mdl_tracker=true` starts the mdl tracker in addition to the bayes tracker. Default is `false`
* Contributors: Christian Dondrup

0.1.4 (2015-03-06)
------------------

0.1.3 (2015-02-25)
------------------

0.1.1 (2015-02-18)
------------------

0.1.0 (2015-02-18)
------------------
* Setting correct version number. The changelogs will be regenerated because the ones from the release branch would not be consistent with the changes made in the devel branch.
* Removed strands_ground_hog
* Setting default value of load_params_from_file to true
* removing HOG launch files
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
* Removed ground_hog from run_deps
* 0.0.1
* Created changelogs
* removed groundHOG references from code.
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
* The people_msgs package does not exist in indigo yet
* 0.0.4
* Updating changelogs
* Removing the leg_detector from the run_dependencies of the launch package for indigo release.
  leg_detector is not released for indigo yet.
* 0.0.3
* Updated changelogs
* 0.0.2
* Updated changelog
* Removed ground_hog from run_deps
* 0.0.1
* Created changelogs
* removed groundHOG references from code.
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
* The people_msgs package does not exist in indigo yet
* 0.0.4
* Updating changelogs
* Removing the leg_detector from the run_dependencies of the launch package for indigo release.
  leg_detector is not released for indigo yet.
* Contributors: Christian Dondrup

0.0.12 (2014-10-31 16:07)
-------------------------
* Adjusting version number.
* Updated changelogs
* 0.0.3
* Updated changelogs
* 0.0.2
* Updated changelog
* Removed ground_hog from run_deps
* 0.0.1
* Created changelogs
* removed groundHOG references from code.
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
* The people_msgs package does not exist in indigo yet
* Contributors: Christian Dondrup

0.0.4 (2014-10-29 18:22)
------------------------
* Updating changelogs
* Removing the leg_detector from the run_dependencies of the launch package for indigo release.
  leg_detector is not released for indigo yet.
* Contributors: Christian Dondrup

0.0.3 (2014-10-23)
------------------
* Updated changelogs
* Added LICENSE files. Fixes `#101 <https://github.com/strands-project/strands_perception_people/issues/101>`_
* Contributors: Christian Dondrup, Lucas Beyer

0.0.2 (2014-10-18 17:39)
------------------------
* Updated changelog
* Removed ground_hog from run_deps
* Contributors: Christian Dondrup

0.0.1 (2014-10-18 17:28)
------------------------
* Created changelogs
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
