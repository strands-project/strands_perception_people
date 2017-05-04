^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^
Changelog for package bayes_people_tracker
^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^

1.5.4 (2016-11-03)
------------------

1.5.3 (2016-07-04)
------------------

1.5.2 (2016-07-02)
------------------

1.5.1 (2016-07-02)
------------------
* added mobility aid detection config (`#201 <https://github.com/strands-project/strands_perception_people/issues/201>`_)
  to replace `#200 <https://github.com/strands-project/strands_perception_people/issues/200>`_
* Compilation fix for OS X.
  Removed unnecessary d suffix for double.
* Contributors: Marc Hanheide, Nick Hawes

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

1.2.0 (2015-11-11)
------------------
* Adding missing tf include
* Renaming directories in bayes_people_tracker. Before this the header files were not installed correctly.
  Also creating a a new cals dedictaed to creating the human marker for rviz. Will be used in people_tracker_filter.
* Merge pull request `#169 <https://github.com/strands-project/strands_perception_people/issues/169>`_ from cdondrup/velo_message
  Adding the velocity of detect people to PeopleTracker message
* Adding mygrate.py and missing install targets.
* Adding rule to migrate rosbags to new message format
* Adding velocities of detected people as a geometry_msgs/Vector3 to PeopleTracker message
* Contributors: Christian Dondrup, Marc Hanheide

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
* Nicer print
* Adding ability to switch between Extended and Unscented Kalman Filter
* Making simple_tracking template based.
* Changed config file structure and made necessary changes to the code.
* Adding pose, pose_array and people publishers to connection callback.
* * Publishing a pose array for all detected people to have more generic output
  * Added missing bayes tracker parameters to launch files and READMEs
  * Starting the mdl tracker is now optional when using the robot launch file. `with_mdl_tracker=true` starts the mdl tracker in addition to the bayes tracker. Default is `false`
* forgot (again) to change default detector.yaml in bayes_people_tracker
* adding visualization to rviz via nav_msgs/Path
* Contributors: Christian Dondrup, Ferdian Jovan

0.1.4 (2015-03-06)
------------------
* Publishin people_msgs/People and adding orientation.
* forgot to undo my config for detectors.yaml in bayes_people_tracker
* provide online stitching poses into trajectories
* add online trajectory construction from /people_tracker/positions
* Contributors: Christian Dondrup, Ferdian Jovan

0.1.3 (2015-02-25)
------------------

0.1.1 (2015-02-18)
------------------

0.1.0 (2015-02-18)
------------------
* Setting correct version number. The changelogs will be regenerated because the ones from the release branch would not be consistent with the changes made in the devel branch.
* Small bug in ros_debug statment
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
* Removing the leg_detector from the run_dependencies of the launch package for indigo release.
  leg_detector is not released for indigo yet.
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
* 0.0.1
* Created changelogs
* Added proper link to paper describing bayes_tracker
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
* Contributors: Christian Dondrup

0.0.1 (2014-10-18 17:28)
------------------------
* Created changelogs
* Renamed strands_pedestrian_tracking to mdl_people_tracker
  This also includes renaming the messages and most of the parameters.
* Forgot to install the config dir.
* Fixed missing things
* Prepared bayes_people_tracker for release.
* Splitting utils package into seperate packages.
* Renamed strands_people_tracker to bayes_people_tracker
* Contributors: Christian Dondrup
