^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^
Changelog for package perception_people_launch
^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^

1.10.0 (2020-02-12)
-------------------

1.9.0 (2019-12-19)
------------------

1.8.1 (2019-05-21)
------------------

1.8.0 (2018-10-01)
------------------

1.7.0 (2018-09-04)
------------------

1.6.0 (2017-09-01)
------------------
* changelogs
* changed from cdondrup to marc
* Contributors: Marc Hanheide

1.5.5 (2017-07-02)
------------------
* updated changelogs
* Merge pull request `#215 <https://github.com/strands-project/strands_perception_people/issues/215>`_ from sbreuers/indigo-devel
  Fix for visual odometry motion matrix+MDL Tracker
* corrects visual odometry motion matrix, based on tf-tree (fixes mdl-tracker)
* Contributors: Jenkins, Lucas Beyer, sbreuers

1.5.4 (2016-11-03)
------------------
* updated changelogs
* Adding the wheelchair detector to the tracker launch file. (`#211 <https://github.com/strands-project/strands_perception_people/issues/211>`_)
  * Made wheelchair detector name more descriptive.
  * Adds the wheelchair detector to the standard tracker launch file.
  Turned off by default enabled by using with_wheelchair_detector:=true.
  If it is used, a machine has to be specifieD: wheelchair_machine:=<machine name>.
  There is no proper default here since the machine needs to be specifically
  configured with CUDA and cudnn.
* Contributors: Alexander Hermans, Jenkins

1.5.3 (2016-07-04)
------------------
* updated changelogs
* Contributors: Jenkins

1.5.2 (2016-07-02 20:52)
------------------------
* updated changelogs
* Contributors: Jenkins

1.5.1 (2016-07-02 17:54)
------------------------
* updated changelogs
* Merge branch 'indigo-devel' of https://github.com/strands-project/strands_perception_people into people_sitting_feature
* Contributors: Ferdian Jovan, Jenkins

1.5.0 (2016-03-15)
------------------
* updated changelogs
* Contributors: Jenkins

1.4.0 (2016-02-17)
------------------
* updated changelogs
* Contributors: Jenkins

1.3.1 (2016-02-11)
------------------
* updated changelogs
* Merge remote-tracking branch 'upstream/indigo-devel' into indigo-devel
* Contributors: Alexander Hermans, Jenkins

1.3.0 (2016-02-01)
------------------
* updated changelogs
* Merge pull request `#185 <https://github.com/strands-project/strands_perception_people/issues/185>`_ from cdondrup/filter_run_dep
  Adding people_tracker_filter to run dependencies of meta and launch package
* Adding people_tracker_filter to run dependencies of meta and launch package.
* Contributors: Christian Dondrup, Jenkins

1.2.1 (2016-01-28)
------------------
* updated changelogs
* Merge pull request `#174 <https://github.com/strands-project/strands_perception_people/issues/174>`_ from cdondrup/new_topic_name
  New topic names for use with openni2
* Changing default topic name for use with openni2
* Contributors: Christian Dondrup, Jenkins

1.2.0 (2015-11-11)
------------------
* updated changelogs
* Merge pull request `#182 <https://github.com/strands-project/strands_perception_people/issues/182>`_ from cdondrup/ppl_tracker_filter
  Adding a filter for the people tracker based on a map
* Adding new parameters to readme file.
* Adding people tracker filter to overall launch file. Is started by default using the map provided on `/map`.
  If not subscribed to, will not use any CPU.
* Contributors: Christian Dondrup, Jenkins, Marc Hanheide

1.1.8 (2015-09-03)
------------------
* updated changelogs
* Merge pull request `#172 <https://github.com/strands-project/strands_perception_people/issues/172>`_ from cdondrup/laser_filters
  Adding laser filters to launch file
* Adding parameter description to README
* Fixing config file location
* Adding laser filters to the launch file to get rid of false psoitives by the leg detector.
* Contributors: Christian Dondrup, Jenkins, Marc Hanheide

1.1.7 (2015-08-25)
------------------
* Merge pull request `#170 <https://github.com/strands-project/strands_perception_people/issues/170>`_ from cdondrup/param_file
  Exposing bayes_people_tracker parameter file via top-level launch files.
* Exposing baye_people_tracker parameter file via top-level launch files.
* Contributors: Christian Dondrup

1.1.6 (2015-06-24)
------------------
* updated changelogs
* Merge branch 'indigo-devel' of https://github.com/strands-project/strands_perception_people into indigo-devel
* 1.1.5
* updated changelogs
* 1.1.4
* updated changelogs
* Merge branch 'indigo-devel' of https://github.com/strands-project/strands_perception_people into indigo-devel
* Contributors: Ferdian Jovan, Jenkins

1.1.5 (2015-05-22)
------------------
* updated changelogs
* Contributors: Jenkins

1.1.4 (2015-05-10)
------------------
* updated changelogs
* Contributors: Jenkins

1.1.3 (2015-04-10)
------------------
* updated changelogs
* Merge pull request `#160 <https://github.com/strands-project/strands_perception_people/issues/160>`_ from cdondrup/topolog
  Supporting the topological logging manager via launch files
* Adding logging manager topic to overall launch file
* Merge branch 'indigo-devel' of http://github.com/strands-project/strands_perception_people into topolog
  Conflicts:
  bayes_people_tracker_logging/launch/logging.launch
* Contributors: Christian Dondrup, Jenkins

1.1.2 (2015-04-07)
------------------
* updated changelogs
* Contributors: Jenkins

1.1.1 (2015-04-03)
------------------
* updated changelogs
* Contributors: Jenkins

1.1.0 (2015-04-02)
------------------

1.0.0 (2015-03-10)
------------------
* Updating changelogs.
* Merge pull request `#146 <https://github.com/strands-project/strands_perception_people/issues/146>`_ from cdondrup/pose_array
  Bayes tracker visualisation improvements and making the mdl tracker optional.
* * Publishing a pose array for all detected people to have more generic output
  * Added missing bayes tracker parameters to launch files and READMEs
  * Starting the mdl tracker is now optional when using the robot launch file. `with_mdl_tracker=true` starts the mdl tracker in addition to the bayes tracker. Default is `false`
* Contributors: Christian Dondrup

0.1.4 (2015-03-06)
------------------
* updated changelogs
* Merge branch 'indigo-devel' of https://github.com/strands-project/strands_perception_people into indigo-devel
* Contributors: Ferdian Jovan, Jenkins

0.1.3 (2015-02-25)
------------------
* updated changelogs
* Merge branch 'indigo-devel' of https://github.com/strands-project/strands_perception_people into indigo-devel
* Contributors: Ferdian Jovan, Jenkins

0.1.2 (2015-02-20)
------------------

0.1.1 (2015-02-18 18:37)
------------------------
* updated changelogs
* Contributors: Jenkins

0.1.0 (2015-02-18 16:59)
------------------------
* Updating changelogs
* Merge pull request `#130 <https://github.com/strands-project/strands_perception_people/issues/130>`_ from cdondrup/fixomatic
  Preparing indigo-devel to be released
* Setting correct version number. The changelogs will be regenerated because the ones from the release branch would not be consistent with the changes made in the devel branch.
* Merged latest version from indigo-devel
* Merge pull request `#128 <https://github.com/strands-project/strands_perception_people/issues/128>`_ from cdondrup/ground_hog
  Removing strands_ground_hog from indigo-devel after it has been moved to attic
* Removed strands_ground_hog
* Setting default value of load_params_from_file to true
* removing HOG launch files
* Changed launch files to new format.
* Merge pull request `#114 <https://github.com/strands-project/strands_perception_people/issues/114>`_ from cdondrup/hydro-devel
  Changed launch files to new format.
* Changed launch files to new format.
* Merge pull request `#109 <https://github.com/strands-project/strands_perception_people/issues/109>`_ from cdondrup/topics
  Making camera topic reconfigurable
* Making camera topic reconfigurable
  So far only the camera namespace was configurable but that introduced an implicit dependency on the openni_wrapper.
  With these changes the whole topic is reconfigurable via a parameter, e.g.:
  camera_namespace:=/my_cam
  depth_image:=/depth/image
  results in `/my_cam/depth/image` as a topic for the depth image. So `camera_namespace` + `depth_image` = the topic on which to look for the depth image.
* Merge pull request `#105 <https://github.com/strands-project/strands_perception_people/issues/105>`_ from lucasb-eyer/hydro-devel
  Fixing `#101 <https://github.com/strands-project/strands_perception_people/issues/101>`_ (Licenses)
* Added LICENSE files. Fixes `#101 <https://github.com/strands-project/strands_perception_people/issues/101>`_
* Merge pull request `#100 <https://github.com/strands-project/strands_perception_people/issues/100>`_ from cdondrup/hydro-devel
  Renaming mdl-People_tracker launch files
* renaming mdl-People_tracker launch files
  to comply with the rest of the structure and to make releasing easier.
* Merge pull request `#98 <https://github.com/strands-project/strands_perception_people/issues/98>`_ from strands-project/rename
  Renamed strands_pedestrian_tracking to mdl_people_tracker
* Renamed strands_pedestrian_tracking to mdl_people_tracker
  This also includes renaming the messages and most of the parameters.
* Merge pull request `#97 <https://github.com/strands-project/strands_perception_people/issues/97>`_ from strands-project/dependencies
  Release preparations
* Some bug fixes
* Prepared launch package for release.
* Merge pull request `#96 <https://github.com/strands-project/strands_perception_people/issues/96>`_ from cdondrup/rename
  Renaming most of the packages to comply with ROS naming conventions
* Renamed pedestrian_tracker launch files
* Calling the leg_detector directly to not need a private fork anymore.
* Splitting utils package into seperate packages.
* strands_visual_odometry is now visual_odometry
* strands_perception_people_launch is now perception_people_launch
* Contributors: Christian Dondrup, Lucas Beyer
