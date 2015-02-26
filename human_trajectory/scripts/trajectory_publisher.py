#!/usr/bin/env python

import sys
import rospy
from human_trajectory.msg import Trajectories
from human_trajectory.trajectories import OfflineTrajectories
from human_trajectory.trajectories import OnlineTrajectories
from strands_navigation_msgs.msg import TopologicalMap
from mongodb_store.message_store import MessageStoreProxy


class TrajectoryPublisher(object):

    def __init__(self, name, interval, online):
        self.map_info = ''
        self._publish_interval = interval
        self.online = online

        rospy.loginfo("Connecting to mongodb...")
        self._store_client = MessageStoreProxy(collection="people_trajectory")
        rospy.loginfo("Connecting to topological_map...")
        self._sub = rospy.Subscriber(
            "/topological_map", TopologicalMap, self.map_callback, None, 10
        )
        rospy.loginfo("Creating human_trajectory/trajectories topic...")
        self._pub = rospy.Publisher(
            name+'/trajectories', Trajectories, queue_size=10
        )

        if online:
            self._publish_rate = rospy.Rate(1 / float(interval))
            self.trajs = OnlineTrajectories()
        else:
            self._publish_rate = rospy.Rate(1)
            self.trajs = OfflineTrajectories()

    # get map info from topological navigation
    def map_callback(self, msg):
        self.map_info = msg.map
        self._sub.unregister()

    # publish as ros message or store human trajectories into mongodb
    # sort poses for each human, delete noisy trajectory
    def publish_trajectories(self):
        start_time = self.trajs.start_secs
        seq = counter = 0

        while not rospy.is_shutdown():
            published_traj = []
            seq += 1
            trajs = Trajectories()
            trajs.header.seq = seq
            trajs.header.stamp = rospy.Time.now()
            trajs.header.frame_id = '/map'

            for uuid, traj in self.trajs.traj.iteritems():
                if self.online:
                    traj_msg = traj.get_trajectory_message()
                    trajs.trajectories.append(traj_msg)
                    published_traj.append(uuid)
                    meta = dict()
                    meta["map"] = self.map_info
                    meta["taken"] = "online"
                    self._store_client.insert(traj_msg, meta)
                else:
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
                            meta["taken"] = "offline"
                            self._store_client.insert(traj_msg, meta)

            for i, uuid in enumerate(published_traj):
                del self.trajs.traj[uuid]

            start_time += self._publish_interval
            counter += len(trajs.trajectories)
            rospy.loginfo("Total trajectories: %d", counter)

            self._pub.publish(trajs)
            self._publish_rate.sleep()


if __name__ == '__main__':
    rospy.init_node('human_trajectories')

    if len(sys.argv) < 3:
        rospy.logerr("usage: trajectory publish_interval online/offline[1/0]")
        sys.exit(2)

    tp = TrajectoryPublisher(
        rospy.get_name(),
        int(sys.argv[1]),
        int(sys.argv[2])
    )
    tp.publish_trajectories()

    rospy.spin()
