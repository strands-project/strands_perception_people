#!/usr/bin/env python

import rospy
from human_trajectory.msg import Trajectories
from human_trajectory.trajectories import OfflineTrajectories
from human_trajectory.trajectories import OnlineTrajectories
from nav_msgs.msg import Path
from topological_logging_manager.msg import LoggingManager
from strands_navigation_msgs.msg import TopologicalMap
from mongodb_store.message_store import MessageStoreProxy


class TrajectoryManager(object):

    def __init__(self, name):
        self.name = name
        self.pub_nav = dict()
        self._last_seen = dict()
        self.map_info = rospy.get_param("~map_info", "")
        self._publish_interval = 1.0
        self._vis = rospy.get_param("~path_visualisation", "false")
        self.online = rospy.get_param("~online_construction", "true")
        self.counter = self.seq = 0
        self._log_permitted = True
        self._with_logman = rospy.get_param("~with_logging_manager", "false")

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

        if self.online:
            if self._with_logman:
                self._sub_log = rospy.Subscriber(
                    rospy.get_param(
                        "~logging_manager_topic", "/logging_manager/log_stamped"
                    ),
                    LoggingManager, self._log_man_cb, None, 10
                )
            self._pub_incr = rospy.Publisher(
                name+'/trajectories/batch', Trajectories, queue_size=10
            )
            self._publish_rate = rospy.Rate(1 / self._publish_interval)
            self.trajs = OnlineTrajectories(
                rospy.get_param("~tracker_topic", "/people_tracker/positions")
            )
        else:
            self._publish_rate = rospy.Rate(1)
            self.trajs = OfflineTrajectories(size=rospy.get_param("~max_trajectories", 10000))

        rospy.loginfo("human_trajectory is ready...")

    # check logging manager permission
    def _log_man_cb(self, msg):
        self._log_permitted = msg.log

    # check logging manager connection
    def _check_log_connection(self):
        return True if self._sub_log.get_num_connections() else False

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
                if traj_msg is not None and len(traj_msg.trajectory) > 1:
                    trajs.trajectories.append(traj_msg)
                if self._vis:
                    self._add_in_nav_msgs(uuid)

            for uuid, traj in self.trajs.complete_traj.items():
                # store trajectories if it is allowed and connections are still
                # intact. LOGGING MANAGER CAN POSSIBLY NOT ALLOW TO STORE DATA
                # (LOG = FALSE), EVEN THOUGH IT SHOULD ALLOW (WP IS LISTED IN
                # CONFIG)
                if not self._with_logman or (self._log_permitted and self._check_log_connection()):
                    traj_msg = self._traj_size_checking(traj.get_trajectory_message())
                    meta = dict()
                    meta["map"] = self.map_info
                    meta["taken"] = "online"
                    if traj_msg is not None:
                        trajs_com.trajectories.append(traj_msg)
                        self._store_client.insert(traj_msg, meta)
                        self.counter += 1
                        rospy.loginfo("Total trajectories: %d", self.counter)
                elif not self._check_log_connection():
                    rospy.logwarn("Connection with logging manager is lost, trajectories will not be saved")
                else:
                    rospy.logwarn("Restricted area, trajectories will not be saved")
                del self.trajs.complete_traj[uuid]

            if self._vis:
                self._publish_in_nav_msgs()
            self._pub_incr.publish(trajs)
            self._pub.publish(trajs_com)
            self._publish_rate.sleep()

    # check how many poses the trajectory has.
    # too long trajectory will not be stored (size restriction from mongodb)
    def _traj_size_checking(self, traj):
        if len(traj.robot) > 100000:
            rospy.logwarn("Trajectory %s is too big in size. It will not be stored" % traj.uuid)
            return None
        else:
            return traj

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
                # showing trajectories each 10 mins
                end_time = start_time + 600
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

            # showing trajectories recorded in 10 mins
            start_time += 600
            self.counter += len(trajs.trajectories)
            rospy.loginfo("Total trajectories: %d", self.counter)

            self._publish_rate.sleep()
            self._pub.publish(trajs)

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

    tp = TrajectoryManager(rospy.get_name())
    tp.publish_trajectories()

    rospy.spin()
