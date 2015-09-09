strands_perception_people
=========================

Please see perception_people_launch/README.md for start-up information.

When using the default STRANDS perception pipeline, please cite:

```
@inproceedings{dondrup2015tracking,
  title={Real-time multisensor people tracking for human-robot spatial interaction},
  author={Dondrup, Christian and Bellotto, Nicola and Jovan, Ferdian and Hanheide, Marc},
  publisher={ICRA/IEEE},
  booktitle={Workshop on Machine Learning for Social Robotics at International Conference on Robotics and Automation (ICRA)},
  year={2015}
}
```

This package contains the people perception pipeline. It is comprised of two detectors:
* Upper body detector
* Leg Detector: http://wiki.ros.org/leg_detector

Depricated and moved to attic branch:
* Ground HOG feature detector

Two trackers:
* Bayesian People Tracker
* Pedestrian Tracker (currently depricated)

And a lot of utility and helper nodes. See https://www.youtube.com/watch?v=zdnvhQU1YNo for a concise explanation. 

Please refere to the READMEs in the specific packages.
