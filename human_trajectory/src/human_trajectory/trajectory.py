#!/usr/bin/env python

import rospy
import pymongo
import human_trajectory.msg
from geometry_msgs.msg import PoseStamped, Pose, Point, Quaternion
from mongodb_store.message_store import MessageStoreProxy
from strands_navigation_msgs.msg import TopologicalMap

import sys
import json
import math


class Trajectory(object):

    def __init__(self, uuid):

        self.uuid = uuid
        self.pose = []
        self.secs = []
        self.nsecs = []
        self.robot_pose = []
        self.length = 0.0
        self.seq = 1

    def get_trajectory_message(self):
        traj = human_trajectory.msg.Trajectory()
        traj.header.seq = self.seq
        traj.header.stamp = rospy.Time.now()
        traj.header.frame_id = '/map'
        traj.uuid = self.uuid
        traj.start_time = rospy.Time(self.secs[0], self.nsecs[0])
        traj.end_time = rospy.Time(self.secs[len(self.secs)-1],
                                   self.nsecs[len(self.nsecs)-1])
        traj.trajectory_length = self.length
        traj.complete = True
        traj.sequence_id = self.seq
        for i in range(len(self.pose)):
            ps = PoseStamped()
            ps.header.seq = i + 1
            ps.header.stamp = rospy.Time(self.secs[i], self.nsecs[i])
            ps.header.frame_id = ''
            # Human pose
            ps.pose = Pose(Point(self.pose[i]['position']['x'],
                                 self.pose[i]['position']['y'],
                                 self.pose[i]['position']['z']),
                           Quaternion(self.pose[i]['orientation']['x'],
                                      self.pose[i]['orientation']['y'],
                                      self.pose[i]['orientation']['z'],
                                      self.pose[i]['orientation']['w']))
            traj.trajectory.append(ps)
            # robot pose
            pose = Pose(Point(self.robot_pose[i]['position']['x'],
                              self.robot_pose[i]['position']['y'],
                              self.robot_pose[i]['position']['z']),
                        Quaternion(self.robot_pose[i]['orientation']['x'],
                                   self.robot_pose[i]['orientation']['y'],
                                   self.robot_pose[i]['orientation']['z'],
                                   self.robot_pose[i]['orientation']['w']))
            traj.robot.append(pose)

        self.seq += 1
        return traj

    def append_pose(self, pose, secs, nsecs, robot_pose):
        self.pose.append(pose)
        self.secs.append(secs)
        self.nsecs.append(nsecs)
        self.robot_pose.append(robot_pose)

    def sort_pose(self):
        self.pose, self.secs, self.nsecs = self.__quick_sort(self.pose,
                                                             self.secs,
                                                             self.nsecs)

    # this function assumes that all objects have the same length
    # and the secs are the same
    def __validate_poses(self, pose, secs, nsecs):
        i = 0
        while i < len(nsecs):
            reduced_nsecs = nsecs[(i + 1):]
            if nsecs[i] in reduced_nsecs:
                index = reduced_nsecs.index(nsecs[i])
                pose_index = pose[i + 1 + index]

                prev_nsecs = next_nsecs = 1000000
                prev_pose = next_pose = pose[i]
                for j in range(len(nsecs)):
                    delta = nsecs[j] - nsecs[i]
                    if delta == 0:
                        continue
                    if delta > 0 and delta < next_nsecs:
                        next_nsecs = nsecs[j]
                        next_pose = pose[j]
                    if delta < 0 and delta < prev_nsecs:
                        prev_nsecs = nsecs[j]
                        prev_pose = pose[j]

                if prev_nsecs == 1000000 and next_nsecs == 1000000:
                    pose = [pose[i]]
                    secs = [secs[i]]
                    nsecs = [nsecs[i]]
                    break

                delta_i = abs(pose[i] - prev_pose) + abs(pose[i] - next_pose)
                delta_index = abs(pose_index - prev_pose) + \
                    abs(pose_index - next_pose)
                if delta_i < delta_index:
                    del nsecs[i + 1 + index]
                    del pose[i + 1 + index]
                    del secs[i + 1 + index]
                    i -= 1
                else:
                    del nsecs[i]
                    del pose[i]
                    del secs[i]
                    i -= 1
            i += 1
        return pose, secs, nsecs

    def __quick_sort(self, pose, secs, nsecs, secs_sorted=False):
        less_pose = []
        equal_pose = []
        greater_pose = []
        less_secs = []
        equal_secs = []
        greater_secs = []
        less_nsecs = []
        equal_nsecs = []
        greater_nsecs = []

        if len(secs) > 1:
            pivot = secs[0]
            for i, sec in enumerate(secs):
                if sec < pivot:
                    less_secs.append(sec)
                    less_pose.append(pose[i])
                    less_nsecs.append(nsecs[i])
                elif sec == pivot:
                    equal_secs.append(sec)
                    equal_pose.append(pose[i])
                    equal_nsecs.append(nsecs[i])
                else:
                    greater_secs.append(sec)
                    greater_pose.append(pose[i])
                    greater_nsecs.append(nsecs[i])

            less_pose, less_secs, less_nsecs = \
                self.__quick_sort(less_pose, less_secs, less_nsecs,
                                  secs_sorted)
            greater_pose, greater_secs, greater_nsecs = \
                self.__quick_sort(greater_pose, greater_secs, greater_nsecs,
                                  secs_sorted)
            if not secs_sorted:
                equal_pose, equal_secs, equal_nsecs = \
                    self.__validate_poses(equal_pose, equal_secs, equal_nsecs)
                equal_pose, equal_nsecs, equal_secs = \
                    self.__quick_sort(equal_pose, equal_nsecs, equal_secs,
                                      not secs_sorted)

            return less_pose + equal_pose + greater_pose, less_secs + \
                equal_secs + greater_secs, less_nsecs + equal_nsecs + \
                greater_nsecs

            pose = less_pose + equal_pose + greater_pose
            secs = less_secs + equal_secs + greater_secs
            nsecs = less_nsecs + equal_nsecs + greater_nsecs

        return pose, secs, nsecs

    def calc_length(self):
        length = 0.0
        if len(self.pose) < 2:
            return length
        for i in range(1, len(self.pose)):
            j = i - 1
            length += math.hypot(
                (self.pose[i]['position']['x'] - self.pose[j]['position']['x']),
                (self.pose[i]['position']['x'] - self.pose[j]['position']['x'])
            )
        self.length = length

    def to_JSON(self):
        return json.dumps(self, default=lambda o: o.__dict__,
                          sort_keys=True, indent=4)


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
        self._validating_trajectory()
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

    def _validating_trajectory(self):
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


if __name__ == '__main__':
    rospy.init_node('human_trajectories')

    if len(sys.argv) < 3:
        rospy.logerr("usage: trajectory publish_interval store_or_publish[1/0]")
        sys.exit(2)

    trajs = Trajectories(rospy.get_name(), int(sys.argv[1]))
    trajs.publish_trajectories(int(sys.argv[2]))
