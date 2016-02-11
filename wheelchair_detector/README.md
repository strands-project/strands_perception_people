# Wheelchair detector

This is the wheelchair detector package. Given a laser scan it will detect wheelchairs and publish these as a pose array.

Both the input (`LaserScan`) as well as the output (`PoseArray`) topics can be set based on the ros parameters `laser_topic` and `detection_topic`. They will default to `/scan` and `/wheelchair_detections`.

Currently this is only a dummy node and the readme is here because there should be a readme!

## The wills and won'ts
To clarify what we are currently planning to do:
1. The node will publish wheelchair detections as pose arrays.
2. In case stuff goes great we might even distinguish between wheelchairs and possible other walking aids. In such a case we will publish them on seperate topics.

We currently do not plan to and thus likely won't:
1. Publish orientations. This is not feasible based on laser data.
2. Explicitely track detections ourselves.