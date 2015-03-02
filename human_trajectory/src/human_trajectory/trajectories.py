#!/usr/bin/env python

import random
import rospy
import pymongo
from geometry_msgs.msg import PoseStamped, Pose, Point, Quaternion
from std_msgs.msg import Header
from bayes_people_tracker.msg import PeopleTracker
from human_trajectory.trajectory import Trajectory


class Trajectories(object):

    def __init__(self):
        self.traj = dict()

    # delete trajs that appear less than 5 secs or have length less than 0.1
    def _validate_trajectories(self, traj):
        rospy.loginfo("Validating data...")
        will_be_deleted = []
        for uuid in traj:
            traj[uuid].validate_all_poses()
            if traj[uuid].length[-1] < 0.1 and uuid not in will_be_deleted:
                will_be_deleted.append(uuid)
            start_appear = traj[uuid].humrobpose[0][0].header.stamp
            end_appear = traj[uuid].humrobpose[-1][0].header.stamp
            delta_appear = end_appear - start_appear
            if delta_appear.secs < 3 and uuid not in will_be_deleted:
                will_be_deleted.append(uuid)

        rospy.loginfo("Deleting noisy data...")
        for i, uuid in enumerate(will_be_deleted):
            del traj[uuid]

        return traj


class OnlineTrajectories(Trajectories):

    def __init__(self):
        Trajectories.__init__(self)
        self._temp_traj = dict()
        self._first_seen = dict()
        self._last_seen = dict()
        self._checked = dict()
        self.complete = dict()
        self.robot_pose = Pose()
        rospy.Subscriber(
            "/people_tracker/positions", PeopleTracker,
            self.pt_callback, None, 30
        )
        rospy.Subscriber(
            "/robot_pose", Pose, self.pose_callback, None, 10
        )
        rospy.loginfo("Taking data from people_tracker/positions...")

    # get robot position
    def pose_callback(self, pose):
        self.robot_pose = pose

    # extract PeopleTracker message to obtain trajectories
    def pt_callback(self, msg):
        for i, uuid in enumerate(msg.uuids):
            if uuid not in self._temp_traj:
                t = Trajectory(uuid)
                t.append_ros_pose(msg.poses[i], msg.header, self.robot_pose)
                self._temp_traj[uuid] = t
                self._first_seen[uuid] = msg.header.stamp.secs
                self._checked[uuid] = False
            else:
                t = self._temp_traj[uuid]
                t.append_ros_pose(msg.poses[i], msg.header, self.robot_pose)
            self._last_seen[uuid] = msg.header.stamp.secs

        self.check_completeness()

    # check complete trajectories from _temp_traj and validate
    # the those complete trajectories before they are put in ready
    # trajectories (self.traj)
    def check_completeness(self):
        cur_time = rospy.Time.now()
        will_be_deleted = []
        for uuid, secs in self._first_seen.items():
            if (cur_time.secs - 5) >= secs:
                self._checked[uuid] = True

        self.traj.update({
            uuid:self._temp_traj[uuid]
            for uuid, val in self._checked.iteritems() if val
        })

        for uuid, secs in self._last_seen.items():
            if (cur_time.secs - 5) >= secs:
                temp = self._validate_trajectories({uuid:self._temp_traj[uuid]})
                if temp == {}:
                    self.complete[uuid] = False
                else:
                    self.complete[uuid] = True
                del self._temp_traj[uuid]
                del self._first_seen[uuid]
                del self._last_seen[uuid]
                del self._checked[uuid]


class OfflineTrajectories(Trajectories):

    def __init__(self):
        self.start_secs = -1
        # calling superclass
        Trajectories.__init__(self)

        self.from_people_trajectory = self._retrieve_logs()
        if not self.from_people_trajectory:
            self.traj = self._validate_trajectories(self.traj)
        rospy.loginfo("Data is ready...")

    # construct trajectories based on data from people_perception collection
    def _construct_from_people_perception(self, logs):
        rospy.loginfo("Constructing data from people perception...")
        for log in logs:
            for i, uuid in enumerate(log['uuids']):
                if uuid not in self.traj:
                    t = Trajectory(uuid)
                    t.append_pose(log['people'][i],
                                  log['header']['stamp']['secs'],
                                  log['header']['stamp']['nsecs'],
                                  log['robot'])
                    self.traj[uuid] = t
                else:
                    t = self.traj[uuid]
                    t.append_pose(log['people'][i],
                                  log['header']['stamp']['secs'],
                                  log['header']['stamp']['nsecs'],
                                  log['robot'])
                if self.start_secs == -1 or \
                        log['header']['stamp']['secs'] < self.start_secs:
                    self.start_secs = log['header']['stamp']['secs']

    # construct trajectories based on data from people_trajectory collection
    def _construct_from_people_trajectory(self, logs):
        rospy.loginfo("Constructing data from people trajectory...")
        for log in logs:
            t = Trajectory(str(log['uuid']))
            t.length = [0.0 for i in range(len(log['robot']))]
            t.length[-1] = log['trajectory_length']
            t.sequence_id = log['sequence_id']
            robot_pose = [
                Pose(
                    Point(i['position']['x'],
                          i['position']['y'],
                          i['position']['z']),
                    Quaternion(i['orientation']['x'],
                               i['orientation']['y'],
                               i['orientation']['z'],
                               i['orientation']['w']))
                for i in log['robot']
            ]
            human_pose = [
                PoseStamped(
                    Header(i['header']['seq'],
                           rospy.Time(i['header']['stamp']['secs'], i['header']['stamp']['nsecs']), i['header']['frame_id']), Pose(
                        Point(i['pose']['position']['x'],
                              i['pose']['position']['y'],
                              i['pose']['position']['z']),
                        Quaternion(i['pose']['orientation']['x'],
                                   i['pose']['orientation']['y'],
                                   i['pose']['orientation']['z'],
                                   i['pose']['orientation']['w'])))
                for i in log['trajectory']
            ]
            t.humrobpose = zip(human_pose, robot_pose)
            self.traj[log['uuid']] = t
            traj_start = log['trajectory'][0]['header']['stamp']['secs']
            if self.start_secs == -1 or traj_start < self.start_secs:
                self.start_secs = traj_start

    # retrieve trajectory from mongodb
    def _retrieve_logs(self):
        client = pymongo.MongoClient(
            rospy.get_param("datacentre_host", "localhost"),
            rospy.get_param("datacentre_port", 62345)
        )
        rospy.loginfo("Retrieving data from mongodb...")
        people_traj = client.message_store.people_trajectory.find()
        if people_traj.count() > 0:
            uuid = [
                people_traj[random.randint(0, people_traj.count()-1)]['uuid']
                for i in range(5)
            ]
            temp = [{"uuids": i} for i in uuid]
            logs = client.message_store.people_perception.find(
                {"$or": temp},
                {"uuids": 1}
            )
            if logs.count() > 0:
                self._construct_from_people_trajectory(people_traj)
                # if data comes from people_trajectory db then the data
                # has been validated
                return True

        logs = client.message_store.people_perception.find()
        self._construct_from_people_perception(logs)

        return False
