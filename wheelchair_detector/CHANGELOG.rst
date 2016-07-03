^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^
Changelog for package wheelchair_detector
^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^

1.5.2 (2016-07-02)
------------------
* Update package.xml
* Contributors: Marc Hanheide

1.5.1 (2016-07-02)
------------------
* Wheelchair fixing (`#202 <https://github.com/strands-project/strands_perception_people/issues/202>`_)
  * Moved several files and configured the CMakeLists.txt to correctly install everything.
  * Added missing python files through the setup.py
  * Automatically downloads the model and install it.
  * Updated to default detection threshold to a better value
  * Further cleaned the way the model is installed.
  This will overwrite the model file instead of creating several copies.
  Furthermore the file is found properly based on the launch file when the ws is sourced.
* Contributors: Alexander Hermans

1.5.0 (2016-03-15)
------------------
* Merge pull request `#189 <https://github.com/strands-project/strands_perception_people/issues/189>`_ from Pandoro/indigo-devel
  Adding functionality to the DROW detector
* Updated README and package xmls.
* obviously the wrong names!
* Adding dependencies
* Updated readme wrt. to CUDNN
* Changes to skip messages and not queue them if the detector is too slow.
* Readme fixes.
* Updated readme.
* Made cudnn optional.
* Furthes fixes to the README.
* Updated the README.
* Added the launch file.
* Added script to get the model.
* Removed old message.
* made the network parameter file a parameter.
* Added gitignore.
* Fixed definition of network to use CUDNN pooling and convs.
* Initial push towards functionality.
* Contributors: Alexander Hermans, Marc Hanheide

1.4.0 (2016-02-17)
------------------

1.3.1 (2016-02-11)
------------------
* Added install target.
* Fixed a typo in the package.xml
* Fixed version
* Added some more info to the README file.
* Added a quick readme for the wheelchair detector.
* Initial commit of a wheelchair detection dummy node.
  This node currently only published a pose array for every scan filled with dummy detections.
  It is only here for interface defenitions and integration purposes.
* Contributors: Alexander Hermans

* Added install target.
* Fixed a typo in the package.xml
* Fixed version
* Added some more info to the README file.
* Added a quick readme for the wheelchair detector.
* Initial commit of a wheelchair detection dummy node.
  This node currently only published a pose array for every scan filled with dummy detections.
  It is only here for interface defenitions and integration purposes.
* Contributors: Alexander Hermans

1.3.0 (2016-02-01)
------------------

1.2.1 (2016-01-28)
------------------

1.2.0 (2015-11-11)
------------------

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

0.0.14 (2014-11-23)
-------------------

0.0.13 (2014-10-31 16:14)
-------------------------

0.0.12 (2014-10-31 16:07)
-------------------------

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

0.0.2 (2014-10-18 17:39)
------------------------

0.0.1 (2014-10-18 17:28)
------------------------
