#!/usr/bin/env python

import sys
import rospy
import human_trajectory.trajectories
import human_trajectory.msg
from strands_navigation_msgs.msg import TopologicalMap
from mongodb_store.message_store import MessageStoreProxy


class TrajectoryPublisher(object):

    def __init__(self, name, interval):
        self.map_info = ''
        # self._publish_rate = rospy.Rate(1 / float(interval))
        self._publish_rate = rospy.Rate(2)
        self._publish_interval = interval

        rospy.loginfo("Connecting to mongodb...")
        self._store_client = MessageStoreProxy(collection="people_trajectory")
        rospy.loginfo("Connecting to topological_map...")
        self._sub = rospy.Subscriber("/topological_map", TopologicalMap,
                                     self.map_callback, None, 10)
        rospy.loginfo("Creating human_trajectory/trajectories topic...")
        self._pub = rospy.Publisher(name+'/trajectories',
                                    human_trajectory.msg.Trajectories,
                                    queue_size=10)

        self.trajs = human_trajectory.trajectories.Trajectories()

    # get map info from topological navigation
    def map_callback(self, msg):
        self.map_info = msg.map
        self._sub.unregister()

    # publish as ros message or store human trajectories into mongodb
    # sort poses for each human, delete noisy trajectory
    def publish_trajectories(self):
        start_time = self.trajs.start_secs
        seq = 1
        published_traj = []
        counter = 0

        # While start time does not exceed the latest trajectory
        while len(published_traj) != len(self.trajs.traj):
            rospy.loginfo("Total trajectories from %d", start_time)
            trajs = human_trajectory.msg.Trajectories()
            trajs.header.seq = seq
            trajs.header.stamp = rospy.Time.now()
            trajs.header.frame_id = '/map'
            seq += 1
            for uuid, traj in self.trajs.traj.iteritems():
                end_time = start_time + self._publish_interval
                last_index = len(traj.humrobpose)-1
                traj_end = traj.humrobpose[last_index][0].header.stamp.secs
                if traj_end <= end_time and uuid not in published_traj:
                    traj_msg = traj.get_trajectory_message()
                    trajs.trajectories.append(traj_msg)
                    published_traj.append(uuid)
                    if not self.trajs.from_people_trajectory:
                        meta = dict()
                        meta["map"] = self.map_info
                        self._store_client.insert(traj_msg, meta)

            start_time += self._publish_interval
            counter += len(trajs.trajectories)
            rospy.loginfo("up to %d is %d",
                          start_time, len(trajs.trajectories))

            self._pub.publish(trajs)
            self._publish_rate.sleep()

        rospy.loginfo("Total trajectories: %d", counter)


if __name__ == '__main__':
    rospy.init_node('human_trajectories')

    if len(sys.argv) < 2:
        rospy.logerr("usage: trajectory publish_interval store_or_publish[1/0]")
        sys.exit(2)

    tp = TrajectoryPublisher(rospy.get_name(), int(sys.argv[1]))
    tp.publish_trajectories()
