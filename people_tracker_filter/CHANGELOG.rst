^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^
Changelog for package people_tracker_filter
^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^

1.8.1 (2019-05-21)
------------------

1.8.0 (2018-10-01)
------------------

1.7.0 (2018-09-04)
------------------
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
* Contributors: Manuel Fernandez-Carmona

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
* Updating CMakeLists and package.xml
* Adding velocities to filter.
* Actually publishing people message now.
* Removing occupancy grid utils dependency by copying the relevant functions.
* Updating launch file to make the map server optional. Also adding more arguments.
* * People tracker now has the same functionality as all the other components meaning that it lies dormant until someone subscribes to any of its topics.
  * Filter now filters all the people_tracker topics.
  * Using new people_marker class to create human markers for filtered tracks.
* Adding try catch for cell out of bounds
* Adding people tracker filter based on a map.
* Contributors: Christian Dondrup, Jenkins, Marc Hanheide

1.1.8 (2015-09-03)
------------------

1.1.7 (2015-08-25)
------------------

1.1.6 (2015-06-24)
------------------

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

1.1.0 (2015-04-02)
------------------

1.0.0 (2015-03-10)
------------------

0.1.4 (2015-03-06)
------------------

0.1.3 (2015-02-25)
------------------

0.1.2 (2015-02-20)
------------------

0.1.1 (2015-02-18 18:37)
------------------------

0.1.0 (2015-02-18 16:59)
------------------------
