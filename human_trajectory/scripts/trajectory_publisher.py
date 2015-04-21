#!/usr/bin/env python

import sys
import rospy
from human_trajectory.msg import Trajectories
from human_trajectory.trajectories import OfflineTrajectories
from human_trajectory.trajectories import OnlineTrajectories
from nav_msgs.msg import Path
from topological_logging_manager.msg import LoggingManager
from strands_navigation_msgs.msg import TopologicalMap
from mongodb_store.message_store import MessageStoreProxy


class TrajectoryManager(object):

    def __init__(self, name, visualisation, online):
        self.name = name
        self.pub_nav = dict()
        self._last_seen = dict()
        self.map_info = 'unknown'
        self._publish_interval = 1.0
        self._vis = visualisation
        self.online = online
        self.counter = self.seq = 0
        self._log_permitted = True

        rospy.loginfo("Connecting to mongodb...")
        self._store_client = MessageStoreProxy(collection="people_trajectory")

        rospy.loginfo("Connecting to topological_map...")
        self._sub_topo = rospy.Subscriber(
            "/topological_map", TopologicalMap, self.map_callback, None, 10
        )
        rospy.loginfo("Creating human_trajectory/trajectories topic...")
        self._pub = rospy.Publisher(
            name+'/trajectories/complete', Trajectories, queue_size=10
        )

        if online:
            mgr_topic = rospy.get_param("~manager_topic", "")
            if mgr_topic != '':
                self._sub_log = rospy.Subscriber(
                    mgr_topic, LoggingManager, self.log_man_cb, None, 10
                )
            self._pub_incr = rospy.Publisher(
                name+'/trajectories/batch', Trajectories, queue_size=10
            )
            self._publish_rate = rospy.Rate(1 / self._publish_interval)
            self.trajs = OnlineTrajectories()
        else:
            self._publish_rate = rospy.Rate(1)
            self.trajs = OfflineTrajectories()

        rospy.loginfo("human_trajectory is ready...")

    # check logging manager permission
    def log_man_cb(self, msg):
        self._log_permitted = msg.log

    # construct trajectories message header
    def _construct_header(self):
        trajs = Trajectories()
        self.seq += 1
        trajs.header.seq = self.seq
        trajs.header.stamp = rospy.Time.now()
        trajs.header.frame_id = '/map'
        return trajs

    # publish based on online data from people_tracker
    def _publish_online_data(self):
        while not rospy.is_shutdown():
            trajs = self._construct_header()
            trajs_com = self._construct_header()

            for uuid, traj in self.trajs.traj.items():
                traj_msg = traj.get_trajectory_message(True)
                # if trajectory is not long enough, no need to print.
                # it will be stored in database though.
                if traj_msg is not None:
                    if len(traj_msg.trajectory) >= self.trajs.tracker_freq/2.0:
                        trajs.trajectories.append(traj_msg)
                if self._vis:
                    self._add_in_nav_msgs(uuid)

            for uuid, traj in self.trajs.complete_traj.items():
                if self._log_permitted:
                    traj_msg = traj.get_trajectory_message()
                    trajs_com.trajectories.append(traj_msg)
                    meta = dict()
                    meta["map"] = self.map_info
                    meta["taken"] = "online"
                    self._store_client.insert(traj_msg, meta)
                    self.counter += 1
                    rospy.loginfo("Total trajectories: %d", self.counter)
                del self.trajs.complete_traj[uuid]

            if self._vis:
                self._publish_in_nav_msgs()
            self._pub_incr.publish(trajs)
            self._pub.publish(trajs_com)
            self._publish_rate.sleep()

    # add each traj to be published in nav_msg/Path
    def _add_in_nav_msgs(self, uuid):
        if uuid not in self.pub_nav:
            rospy.loginfo("Creating a publisher for %s...", uuid)
            name = uuid.replace("-", "0")
            self.pub_nav[uuid] = rospy.Publisher(
                self.name + '/' + name, Path, latch=True, queue_size=10
            )

    # publish each traj in nav_msg/Path format
    def _publish_in_nav_msgs(self):
        current_time = rospy.Time.now()
        for uuid, pub in self.pub_nav.items():
            if uuid in self.trajs.traj:
                nav_msg = self.trajs.traj[uuid].get_nav_message()
                pub.publish(nav_msg)
            elif uuid in self._last_seen:
                if (current_time - self._last_seen[uuid]).secs > 60:
                    pub.unregister()
                    del self.pub_nav[uuid]
                    del self._last_seen[uuid]
            else:
                self._last_seen[uuid] = current_time

    # publish based on offline data in mongodb
    def _publish_offline_data(self):
        start_time = self.trajs.start_secs
        while not rospy.is_shutdown():
            trajs = self._construct_header()

            for uuid, traj in self.trajs.traj.items():
                end_time = start_time + self._publish_interval
                traj_end = traj.humrobpose[-1][0].header.stamp.secs
                if traj_end <= end_time:
                    traj_msg = traj.get_trajectory_message()
                    trajs.trajectories.append(traj_msg)
                    if not self.trajs.from_people_trajectory:
                        meta = dict()
                        meta["map"] = self.map_info
                        meta["taken"] = "offline"
                        self._store_client.insert(traj_msg, meta)
                    del self.trajs.traj[uuid]

            start_time += self._publish_interval
            self.counter += len(trajs.trajectories)
            rospy.loginfo("Total trajectories: %d", self.counter)

            self._pub.publish(trajs)
            self._publish_rate.sleep()

    # get map info from topological navigation
    def map_callback(self, msg):
        self.map_info = msg.map
        self._sub_topo.unregister()

    # publish as ros message or store human trajectories into mongodb
    # sort poses for each human, delete noisy trajectory
    def publish_trajectories(self):
        if self.online:
            self._publish_online_data()
        else:
            self._publish_offline_data()

if __name__ == '__main__':
    rospy.init_node('human_trajectories')

    if len(sys.argv) < 3:
        rospy.logerr("usage: trajectory visualisation[1/0] online[1/0]")
        sys.exit(2)

    tp = TrajectoryManager(
        rospy.get_name(),
        int(sys.argv[1]),
        int(sys.argv[2])
    )
    tp.publish_trajectories()

    rospy.spin()
