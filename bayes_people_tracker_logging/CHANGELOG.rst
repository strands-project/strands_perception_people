^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^
Changelog for package bayes_people_tracker_logging
^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^

Forthcoming
-----------
* Message type change. (`#225 <https://github.com/strands-project/strands_perception_people/issues/225>`_)
  * Peoplestamped.msg added for cellphone GPS data. Composed of an array of PersonStamped.msg
  * People msg chsnged to PeopleStamped message
  * People msg changed to PeopleStamped msg
  * code cleaned
  * people_msgs dependency added
  * Now supports both People and PeopleStamped msg types
* Contributors: Khan

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
* Logging component logs the filtered output now.
* Merge pull request `#169 <https://github.com/strands-project/strands_perception_people/issues/169>`_ from cdondrup/velo_message
  Adding the velocity of detect people to PeopleTracker message
* Adding mygrate.py and missing install targets.
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
* Merge pull request `#160 <https://github.com/strands-project/strands_perception_people/issues/160>`_ from cdondrup/topolog
  Supporting the topological logging manager via launch files
* Merge branch 'indigo-devel' of http://github.com/strands-project/strands_perception_people into topolog
  Conflicts:
  bayes_people_tracker_logging/launch/logging.launch
* Adjusting code and loaunch files of logging nodes for proper use of manager topic.
* Merge pull request `#158 <https://github.com/strands-project/strands_perception_people/issues/158>`_ from cdondrup/topolog
  Adding topological information to ppl logging
* Adding topological information to ppl logging
  Closes `#113 <https://github.com/strands-project/strands_perception_people/issues/113>`_
  Needs testing
* Contributors: Christian Dondrup, Jenkins, Marc Hanheide

1.1.2 (2015-04-07)
------------------
* updated changelogs
* Copy and paste error
  d'oh
* Contributors: Christian Dondrup, Jenkins

1.1.1 (2015-04-03)
------------------
* updated changelogs
* Contributors: Jenkins

1.1.0 (2015-04-02)
------------------
* Merge pull request `#157 <https://github.com/strands-project/strands_perception_people/issues/157>`_ from cdondrup/respawn
  Adding respawn flags
* Merge pull request `#155 <https://github.com/strands-project/strands_perception_people/issues/155>`_ from cdondrup/logging
  First version of new logging for trajectories
* Adding respawn flags
  Closes `#152 <https://github.com/strands-project/strands_perception_people/issues/152>`_
  Bad workaround for `#156 <https://github.com/strands-project/strands_perception_people/issues/156>`_ and `#76 <https://github.com/strands-project/strands_perception_people/issues/76>`_
* First version of new logging for trajectories
* Contributors: Christian Dondrup

1.0.0 (2015-03-10)
------------------
* Updating changelogs.
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
* Fixing logger cmake and package files.
* The logging node now only loggs the output of the bayes people tracker. Waiting for the mdl tracker, reduces the capabilities of the bayestrackeer significantly.
* Changed launch files to new format.
* Merge pull request `#114 <https://github.com/strands-project/strands_perception_people/issues/114>`_ from cdondrup/hydro-devel
  Changed launch files to new format.
* Changed launch files to new format.
* Merge pull request `#105 <https://github.com/strands-project/strands_perception_people/issues/105>`_ from lucasb-eyer/hydro-devel
  Fixing `#101 <https://github.com/strands-project/strands_perception_people/issues/101>`_ (Licenses)
* Added LICENSE files. Fixes `#101 <https://github.com/strands-project/strands_perception_people/issues/101>`_
* Merge pull request `#98 <https://github.com/strands-project/strands_perception_people/issues/98>`_ from strands-project/rename
  Renamed strands_pedestrian_tracking to mdl_people_tracker
* Renamed strands_pedestrian_tracking to mdl_people_tracker
  This also includes renaming the messages and most of the parameters.
* Merge pull request `#97 <https://github.com/strands-project/strands_perception_people/issues/97>`_ from strands-project/dependencies
  Release preparations
* Prepared bayes_people_tracker_logging for release
* Merge pull request `#96 <https://github.com/strands-project/strands_perception_people/issues/96>`_ from cdondrup/rename
  Renaming most of the packages to comply with ROS naming conventions
* Splitting utils package into seperate packages.
* Contributors: Christian Dondrup, Lucas Beyer
