^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^
Changelog for package bayes_people_tracker_logging
^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^

1.5.4 (2016-11-03)
------------------

1.5.3 (2016-07-04)
------------------

1.5.2 (2016-07-02)
------------------

1.5.1 (2016-07-02)
------------------

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
* Logging component logs the filtered output now.
* Adding mygrate.py and missing install targets.
* Contributors: Christian Dondrup

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
* Merge branch 'indigo-devel' of http://github.com/strands-project/strands_perception_people into topolog
  Conflicts:
  bayes_people_tracker_logging/launch/logging.launch
* Adjusting code and loaunch files of logging nodes for proper use of manager topic.
* Adding topological information to ppl logging
  Closes `#113 <https://github.com/strands-project/strands_perception_people/issues/113>`_
  Needs testing
* Contributors: Christian Dondrup

1.1.2 (2015-04-07)
------------------
* Copy and paste error
  d'oh
* Contributors: Christian Dondrup

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

0.1.0 (2015-02-18)
------------------
* Setting correct version number. The changelogs will be regenerated because the ones from the release branch would not be consistent with the changes made in the devel branch.
* Fixing logger cmake and package files.
* The logging node now only loggs the output of the bayes people tracker. Waiting for the mdl tracker, reduces the capabilities of the bayestrackeer significantly.
* Changed launch files to new format.
* Changed launch files to new format.
* Contributors: Christian Dondrup

0.0.14 (2014-11-23)
-------------------
* Updating changelogs and adjusting version numbers
* Removing upped body detector and mdl people tracker from logging
  Those two are slowing down the logging and prevent logging if there is only detections via the leg detector.
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
* Prepared bayes_people_tracker_logging for release
* Splitting utils package into seperate packages.
* Contributors: Christian Dondrup
