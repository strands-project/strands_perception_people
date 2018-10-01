^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^
Changelog for package bayes_people_tracker
^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^

1.8.0 (2018-10-01)
------------------
* WIP: working towards tag-aware tracking (`#221 <https://github.com/strands-project/strands_perception_people/issues/221>`_)
  * WIP
  * WIP
  * wip
  * good testing node
  * WIP
  * Fixed count
  * WIP
  * more documentation and testing
  * colors for variances
  * added rviz config
  * prune_named added
  * legacy param support
* Contributors: Marc Hanheide

1.7.0 (2018-09-04)
------------------
* Adapting to changes in bayestracking (`#220 <https://github.com/strands-project/strands_perception_people/issues/220>`_)
  * Adapted to scosar/bayestracking with 2D Polar Model and Added option to select Particle Filter
  * random fixes
  * Adapted to backward-compatibility changes of bayestracking
* Namespaces and topics specified as parameters. (`#218 <https://github.com/strands-project/strands_perception_people/issues/218>`_)
  * Merged with ENRICHME branch. Parametrized topics/frame_ids
  * Undone the timestamp change
  Hi,
  It was a careless change. I originally thought that the timestamp should reflect time creation of the data, but it also makes sense the way you made it plus it's better not to change the meaning of a field now.
  * Reverted changes on original config.
  * Update detector.h
  * Default initialization on constructor
  * Update KConnectedComponentLabeler.cpp
  * Update KConnectedComponentLabeler.cpp
  * Update detector.cpp
  * Suggested change in data interpretation.
  Still pending to check that 0 corresponds to NaN in ushort
  * Bug corrected.
  * commented duplicate code
* Contributors: Manuel Fernandez-Carmona, scosar

1.6.0 (2017-09-01)
------------------
* changelogs
* changed from cdondrup to marc
* Contributors: Marc Hanheide

1.5.5 (2017-07-02)
------------------
* updated changelogs
* Contributors: Jenkins

1.5.4 (2016-11-03)
------------------
* updated changelogs
* Contributors: Jenkins

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
* added mobility aid detection config (`#201 <https://github.com/strands-project/strands_perception_people/issues/201>`_)
  to replace `#200 <https://github.com/strands-project/strands_perception_people/issues/200>`_
* Merge branch 'indigo-devel' of https://github.com/strands-project/strands_perception_people into people_sitting_feature
* Merge pull request `#192 <https://github.com/strands-project/strands_perception_people/issues/192>`_ from hawesie/indigo-devel
  Compilation fix for OS X.
* Compilation fix for OS X.
  Removed unnecessary d suffix for double.
* Merge branch 'indigo-devel' of https://github.com/strands-project/strands_perception_people into people_sitting_feature
* Contributors: Ferdian Jovan, Jenkins, Lucas Beyer, Marc Hanheide, Nick Hawes, ferdianjovan

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
* Contributors: Jenkins

1.2.1 (2016-01-28)
------------------
* updated changelogs
* Contributors: Jenkins

1.2.0 (2015-11-11)
------------------
* updated changelogs
* Merge pull request `#182 <https://github.com/strands-project/strands_perception_people/issues/182>`_ from cdondrup/ppl_tracker_filter
  Adding a filter for the people tracker based on a map
* Adding missing tf include
* Renaming directories in bayes_people_tracker. Before this the header files were not installed correctly.
  Also creating a a new cals dedictaed to creating the human marker for rviz. Will be used in people_tracker_filter.
* Merge pull request `#169 <https://github.com/strands-project/strands_perception_people/issues/169>`_ from cdondrup/velo_message
  Adding the velocity of detect people to PeopleTracker message
* Adding mygrate.py and missing install targets.
* Adding rule to migrate rosbags to new message format
* Adding velocities of detected people as a geometry_msgs/Vector3 to PeopleTracker message
* Contributors: Christian Dondrup, Jenkins, Marc Hanheide

1.1.8 (2015-09-03)
------------------
* updated changelogs
* Contributors: Jenkins

1.1.7 (2015-08-25)
------------------

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
* Merge pull request `#157 <https://github.com/strands-project/strands_perception_people/issues/157>`_ from cdondrup/respawn
  Adding respawn flags
* Adding respawn flags
  Closes `#152 <https://github.com/strands-project/strands_perception_people/issues/152>`_
  Bad workaround for `#156 <https://github.com/strands-project/strands_perception_people/issues/156>`_ and `#76 <https://github.com/strands-project/strands_perception_people/issues/76>`_
* Merge pull request `#153 <https://github.com/strands-project/strands_perception_people/issues/153>`_ from cdondrup/distance_fix
  [bayes_people_tracker] Fixing a bug in calculation of distances and angles
* Stupid mistake in if statement
* Actually using the transformed values helps when calculating the distance.
  Cleaning up unused code fragments.
* Fixed a bug where the min_distance was calculated for the target frame instead of base_link.
* restore detectors.yaml in bayes_people_tracker
* replacing time with number of poses as suggested by Nick
* Contributors: Christian Dondrup, Ferdian Jovan, Jaime Pulido Fentanes

1.0.0 (2015-03-10)
------------------
* Updating changelogs.
* Merge pull request `#147 <https://github.com/strands-project/strands_perception_people/issues/147>`_ from cdondrup/pose_array
  Restructuring tracker parameters, adding Unscented Kalman filter
* Nicer print
* Adding ability to switch between Extended and Unscented Kalman Filter
* Making simple_tracking template based.
* Changed config file structure and made necessary changes to the code.
* Merge pull request `#146 <https://github.com/strands-project/strands_perception_people/issues/146>`_ from cdondrup/pose_array
  Bayes tracker visualisation improvements and making the mdl tracker optional.
* Adding pose, pose_array and people publishers to connection callback.
* * Publishing a pose array for all detected people to have more generic output
  * Added missing bayes tracker parameters to launch files and READMEs
  * Starting the mdl tracker is now optional when using the robot launch file. `with_mdl_tracker=true` starts the mdl tracker in addition to the bayes tracker. Default is `false`
* forgot (again) to change default detector.yaml in bayes_people_tracker
* adding visualization to rviz via nav_msgs/Path
* Contributors: Christian Dondrup, Ferdian Jovan

0.1.4 (2015-03-06)
------------------
* updated changelogs
* Merge pull request `#144 <https://github.com/strands-project/strands_perception_people/issues/144>`_ from cdondrup/people_msgs
  Publishing people_msgs/People and adding orientation.
* Publishin people_msgs/People and adding orientation.
* forgot to undo my config for detectors.yaml in bayes_people_tracker
* provide online stitching poses into trajectories
* Merge branch 'indigo-devel' of https://github.com/strands-project/strands_perception_people into indigo-devel
* add online trajectory construction from /people_tracker/positions
* Contributors: Christian Dondrup, Ferdian Jovan, Jenkins

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
* Small bug in ros_debug statment
* Changed launch files to new format.
* Merge pull request `#114 <https://github.com/strands-project/strands_perception_people/issues/114>`_ from cdondrup/hydro-devel
  Changed launch files to new format.
* Changed launch files to new format.
* Added proper link to paper describing bayes_tracker
* Merge pull request `#105 <https://github.com/strands-project/strands_perception_people/issues/105>`_ from lucasb-eyer/hydro-devel
  Fixing `#101 <https://github.com/strands-project/strands_perception_people/issues/101>`_ (Licenses)
* Added LICENSE files. Fixes `#101 <https://github.com/strands-project/strands_perception_people/issues/101>`_
* Merge pull request `#98 <https://github.com/strands-project/strands_perception_people/issues/98>`_ from strands-project/rename
  Renamed strands_pedestrian_tracking to mdl_people_tracker
* Renamed strands_pedestrian_tracking to mdl_people_tracker
  This also includes renaming the messages and most of the parameters.
* Merge pull request `#97 <https://github.com/strands-project/strands_perception_people/issues/97>`_ from strands-project/dependencies
  Release preparations
* Forgot to install the config dir.
* Fixed missing things
* Prepared bayes_people_tracker for release.
* Merge pull request `#96 <https://github.com/strands-project/strands_perception_people/issues/96>`_ from cdondrup/rename
  Renaming most of the packages to comply with ROS naming conventions
* Splitting utils package into seperate packages.
* Renamed strands_people_tracker to bayes_people_tracker
* Contributors: Christian Dondrup, Lucas Beyer
