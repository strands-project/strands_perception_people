^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^
Changelog for package human_trajectory
^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^

1.7.0 (2018-09-04)
------------------

1.6.0 (2017-09-01)
------------------
* changelogs
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
* Merge pull request `#193 <https://github.com/strands-project/strands_perception_people/issues/193>`_ from ferdianjovan/people_sitting_feature
  People sitting feature
* merge with indigo-devel strands
* unimportant fixing
* modify CMake and package definition
* fixing calculation and has been tested on betty
* fixing bug with people sitting feature
* Merge branch 'indigo-devel' of https://github.com/strands-project/strands_perception_people into people_sitting_feature
* make trajectory package able to pick sitting people
* modify launch file
* add or cond for save_ubd, add argument for human_trajectory.launch
* Contributors: Ferdian Jovan, Jenkins, Nick Hawes, Rares Ambrus, STRANDS, ferdianjovan

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
* Contributors: Jenkins

1.1.8 (2015-09-03)
------------------
* updated changelogs
* Contributors: Jenkins

1.1.7 (2015-08-25)
------------------
* Merge pull request `#167 <https://github.com/strands-project/strands_perception_people/issues/167>`_ from PDuckworth/indigo-devel
  updated trajectory message to include the displacement
* Merge pull request `#1 <https://github.com/strands-project/strands_perception_people/issues/1>`_ from ferdianjovan/paul_update
  Update for your PR
* add resizing max retrieved trajectories to launch file
* update README, modify the OfflineTrajectories to work on query for both collections, fix an error in trajectory.py
* add a script to import trajectories from mongo to file
* to make launch file accessible
* mini update to remove _meta
* updater tool to query, edit, and then re-upload people_trajectory documents
* updated trajectory message to include the displacement and displacement_to_pose ratio used for filtering
* forgot to put launch folder in install CMakeLists.txt
* Contributors: Christian Dondrup, Ferdian Jovan, Paul, Paul Duckworth

1.1.6 (2015-06-24)
------------------
* updated changelogs
* Merge pull request `#165 <https://github.com/strands-project/strands_perception_people/issues/165>`_ from ferdianjovan/indigo-devel
  Fixing error in https://github.com/strands-project/strands_perception_people/issues/163
* Merge branch 'indigo-devel' of https://github.com/strands-project/strands_perception_people into indigo-devel
* 1.1.5
* updated changelogs
* 1.1.4
* updated changelogs
* Merge branch 'indigo-devel' of https://github.com/strands-project/strands_perception_people into indigo-devel
* Fixing error length problem in get_trajectory_message function, making visualisation adaptive to the length of poses (preventing error),
  Add the option to choose trajectories from a specific map.
* Contributors: Christian Dondrup, Ferdian Jovan, Jenkins

1.1.5 (2015-05-22)
------------------
* updated changelogs
* Contributors: Jenkins

1.1.4 (2015-05-10)
------------------
* updated changelogs
* Merge pull request `#161 <https://github.com/strands-project/strands_perception_people/issues/161>`_ from ferdianjovan/indigo-devel
  Add logging manager permission
* forgot to add launch folder
* add launch file, add logging_manager permission, add restriction to post trajectories longer than 1 pose.
* add logging manager, fix fluctuating cpu usage
* Contributors: Ferdian Jovan, Jenkins, Nick Hawes

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
* Merge pull request `#151 <https://github.com/strands-project/strands_perception_people/issues/151>`_ from ferdianjovan/indigo-devel
  Add visualisation aid for trajectories
* fixing visualization
* add visualisation aid taken from database
* replacing time with number of poses as suggested by Nick
* Contributors: Christian Dondrup, Ferdian Jovan

1.0.0 (2015-03-10)
------------------
* Updating changelogs.
* Merge pull request `#145 <https://github.com/strands-project/strands_perception_people/issues/145>`_ from ferdianjovan/indigo-devel
  adding visualization to rviz via nav_msgs/Path
* keep visualization for 1 min
* forgot (again) to change default detector.yaml in bayes_people_tracker
* adding visualization to rviz via nav_msgs/Path
* Contributors: Christian Dondrup, Ferdian Jovan

0.1.4 (2015-03-06)
------------------
* updated changelogs
* Merge pull request `#140 <https://github.com/strands-project/strands_perception_people/issues/140>`_ from ferdianjovan/indigo-devel
  Provide online stitching poses into trajectories
* provide online stitching poses into trajectories
* Merge branch 'indigo-devel' of https://github.com/strands-project/strands_perception_people into indigo-devel
* add online trajectory construction from /people_tracker/positions
* Contributors: Christian Dondrup, Ferdian Jovan, Jenkins

0.1.3 (2015-02-25)
------------------
* updated changelogs
* Merge pull request `#136 <https://github.com/strands-project/strands_perception_people/issues/136>`_ from ferdianjovan/indigo-devel
  Change message that is stored in Mongo, remove store_to_db option
* Merge branch 'indigo-devel' of https://github.com/strands-project/strands_perception_people into indigo-devel
* polish code, change message that is stored, remove store_to_database option
* Contributors: Christian Dondrup, Ferdian Jovan, Jenkins

0.1.2 (2015-02-20)
------------------
* updated changelogs
* Merge pull request `#134 <https://github.com/strands-project/strands_perception_people/issues/134>`_ from cdondrup/fix
  Fixing my sed mistakes and the install targets for human_trajectory.
* Fixing my sed mistakes and the install targets for human_trajectory.
* Merge pull request `#131 <https://github.com/strands-project/strands_perception_people/issues/131>`_ from ferdianjovan/indigo-devel
  add human_trajectory package to indigo-devel branch
* fixing Cmake and package.xml, add this package in metapackage
* add dependency in Cmake and package.xml
* add human_trajectory package to indigo-devel branch
* Contributors: Christian Dondrup, Ferdian Jovan, Jenkins

0.1.1 (2015-02-18 18:37)
------------------------

0.1.0 (2015-02-18 16:59)
------------------------
