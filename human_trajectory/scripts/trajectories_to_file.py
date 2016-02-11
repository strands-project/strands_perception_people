#!/usr/bin/env python

import sys
import rospy
from human_trajectory.trajectories import OfflineTrajectories


class TrajectoryToFile(object):

    def __init__(self):
        self.trajs = OfflineTrajectories()

    def store_to_file(self, path):
        fo = open(path + "/trajectories.data", "w+")
        for uuid, traj in self.trajs.traj.iteritems():
            data = uuid + " ="
            for humrob in traj.humrobpose:
                time = str(humrob[0].header.stamp.secs) + "," + str(humrob[0].header.stamp.nsecs)
                xytime = (
                    humrob[0].pose.position.x, humrob[0].pose.position.y, time
                )
                data += " " + str(xytime)
            fo.write(data + "\n")
        fo.close()


if __name__ == '__main__':
    rospy.init_node("trajectory_to_file_converter")
    if len(sys.argv) < 2:
        rospy.logerr("usage: converter [path]")
        sys.exit(2)

    rospy.loginfo("Convert and importing trajectories to %s" % sys.argv[1])
    ttf = TrajectoryToFile()
    ttf.store_to_file(sys.argv[1])

    rospy.loginfo("Finish importing to specified path...")
