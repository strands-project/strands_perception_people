#!/usr/bin/env python

import sys
import rospy
from human_trajectory.trajectories import Trajectories

if __name__ == '__main__':
    rospy.init_node('human_trajectories')

    if len(sys.argv) < 3:
        rospy.logerr("usage: trajectory publish_interval store_or_publish[1/0]")
        sys.exit(2)

    trajs = Trajectories(rospy.get_name(), int(sys.argv[1]))
    trajs.publish_trajectories(int(sys.argv[2]))
