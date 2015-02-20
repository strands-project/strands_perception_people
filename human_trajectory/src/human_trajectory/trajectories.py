#!/usr/bin/env python

import rospy
import pymongo
import human_trajectory.msg
from human_trajectory.trajectory import Trajectory
from mongodb_store.message_store import MessageStoreProxy
from strands_navigation_msgs.msg import TopologicalMap


class Trajectories(object):

    def __init__(self, name, interval):
        self._traj = dict()
        self.map_info = ''
        self.start_secs = -1
        self.publish_rate = rospy.Rate(1 / float(interval))
        self.publish_interval = interval

        rospy.loginfo("Connecting to mongodb...")
        self._client = pymongo.MongoClient(rospy.get_param("datacentre_host"),
                                           rospy.get_param("datacentre_port"))
        self._store_client = MessageStoreProxy(collection="people_trajectory")
        rospy.loginfo("Connecting to topological_map...")
        self.sub = rospy.Subscriber("/topological_map", TopologicalMap,
                                    self.map_callback, None, 10)
        rospy.loginfo("Creating human_trajectory/trajectories topic...")
        self.pub = rospy.Publisher(name+'/trajectories',
                                   human_trajectory.msg.Trajectories,
                                   queue_size=10)

        self._retrieve_logs()
        self._validating_trajectories()
        rospy.loginfo("Data is ready...")

    def map_callback(self, msg):
        self.map_info = msg.map
        self.sub.unregister()

    def get_poses_persecond(self):
        average_poses = 0
        for uuid in self._traj:
            traj = self._traj[uuid]
            inner_counter = 1
            outer_counter = 1
            prev_sec = traj.secs[0]
            for i, sec in enumerate(traj.secs[1:]):
                if prev_sec == sec:
                    inner_counter += 1
                else:
                    prev_sec = sec
                    outer_counter += 1
            average_poses += round(inner_counter/outer_counter)
        return round(average_poses/len(self._traj))

    def _validating_trajectories(self):
        rospy.loginfo("Sorting data...")
        untraj = []
        mframe = self.get_poses_persecond() * 5
        for uuid in self._traj:
            self._traj[uuid].sort_pose()
            self._traj[uuid].calc_length()

            if self._traj[uuid].length < 0.1 and uuid not in untraj:
                untraj.append(uuid)
            if len(self._traj[uuid].pose) < mframe and uuid not in untraj:
                untraj.append(uuid)

        rospy.loginfo("Validating data...")
        for i, uuid in enumerate(untraj):
            del self._traj[uuid]

    def _retrieve_logs(self):
        rospy.loginfo("Retrieving data from mongodb...")
        logs = self._client.message_store.people_perception.find()

        for log in logs:
            for i, uuid in enumerate(log['uuids']):
                if uuid not in self._traj:
                    t = Trajectory(uuid)
                    t.append_pose(log['people'][i],
                                  log['header']['stamp']['secs'],
                                  log['header']['stamp']['nsecs'],
                                  log['robot'])
                    self._traj[uuid] = t
                else:
                    t = self._traj[uuid]
                    t.append_pose(log['people'][i],
                                  log['header']['stamp']['secs'],
                                  log['header']['stamp']['nsecs'],
                                  log['robot'])
                if self.start_secs == -1 or \
                        log['header']['stamp']['secs'] < self.start_secs:
                    self.start_secs = log['header']['stamp']['secs']

    def publish_trajectories(self, save_to_db=False):
        start_time = self.start_secs
        seq = 1
        published_traj = []
        counter = 0

        # While start time does not exceed the latest trajectory
        while len(published_traj) != len(self._traj):
            rospy.loginfo("Total trajectories from %d", start_time)
            trajs = human_trajectory.msg.Trajectories()
            trajs.header.seq = seq
            trajs.header.stamp = rospy.Time.now()
            trajs.header.frame_id = '/map'
            seq += 1
            for uuid, traj in self._traj.iteritems():
                end_time = start_time + self.publish_interval
                if traj.secs[len(traj.secs)-1] <= end_time \
                        and uuid not in published_traj:
                    trajs.trajectories.append(traj.get_trajectory_message())
                    published_traj.append(uuid)

            start_time += self.publish_interval
            counter += len(trajs.trajectories)
            rospy.loginfo("up to %d is %d",
                          start_time, len(trajs.trajectories))

            if save_to_db:
                meta = dict()
                meta["map"] = self.map_info
                self._store_client.insert(trajs, meta)
                rospy.sleep(1)
            else:
                self.pub.publish(trajs)
                self.publish_rate.sleep()

        rospy.loginfo("Total trajectories: %d", counter)
