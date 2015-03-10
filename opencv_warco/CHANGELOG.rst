^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^
Changelog for package opencv_warco
^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^

1.0.0 (2015-03-10)
------------------

0.1.4 (2015-03-06)
------------------
* Added missing include and linking. These were necessary to compile under OS X.
* Contributors: Nick Hawes

0.1.3 (2015-02-25)
------------------

0.1.1 (2015-02-18)
------------------
* Merge pull request `#132 <https://github.com/strands-project/strands_perception_people/issues/132>`_ from cdondrup/opencv_warco
  Refactoring opencv_warco to enable build from install and devel
* Refactoring opencv_warco to be able to build strands_head_orientation from install and devel.
* Contributors: Christian Dondrup

0.1.0 (2015-02-18)
------------------
* Setting correct version number. The changelogs will be regenerated because the ones from the release branch would not be consistent with the changes made in the devel branch.
* Started to make opencv_warco installable
* Contributors: Christian Dondrup

0.0.14 (2014-11-23)
-------------------
* Merging in latest changes and removed some artifacts of that.
* Removed packages that are not supposed to be released.
* Contributors: Christian Dondrup

0.0.13 (2014-10-31 16:14)
-------------------------
* Rebasing latest changes and removing artifacts.
* Removed packages that are not supposed to be released.
* Contributors: Christian Dondrup

0.0.12 (2014-10-31 16:07)
-------------------------
* Merging in latest changes and removed some artifacts of that.
* Removed packages that are not supposed to be released.
* Updated opencv-warco version.
* Contributors: Christian Dondrup, lucasb-eyer

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
* Contributors: Lucas Beyer

0.0.2 (2014-10-18 17:39)
------------------------

0.0.1 (2014-10-18 17:28)
------------------------
* Removed packages that are not supposed to be released.
* Revert to downscaling the images to 50x50.
  Arbitrary patch-sizes don't make sense when the size of the filters in
  the bank is not adapted.
* Shame on me!
* Initial opencv-warco based head pose estimator.
* Contributors: Christian Dondrup, lucasb-eyer
