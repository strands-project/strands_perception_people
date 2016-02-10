# Wheelchair detector

This is the wheelchair detector package. Given a laser scan it will detect wheelchairs and publish these as a pose array.

Both the input (`LaserScan`) as well as the output (`PoseArray`) topics can be set based on the ros parameters `laser_topic` and `detection_topic`. They will default to `/scan` and `/wheelchair_detections`.

Currently this is only a dummy node and the readme is here because there should be a readme!