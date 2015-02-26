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
        self.start_secs = -1

    # delete trajs that appear less than 5 secs or have length less than 0.1
    def _validate_trajectories(self, traj):
        rospy.loginfo("Validating data...")
        untraj = []
        mframe = 0
        for uuid in traj:
            traj[uuid].validate_poses()
            mframe += traj[uuid].pps
            if traj[uuid].length < 0.1 and uuid not in untraj:
                untraj.append(uuid)

        mframe = round(mframe/len(traj)) * 5

        for uuid in traj:
            if len(traj[uuid].humrobpose) < mframe and uuid not in untraj:
                untraj.append(uuid)

        rospy.loginfo("Deleting noisy data...")
        for i, uuid in enumerate(untraj):
            del traj[uuid]

        return traj


class OnlineTrajectories(Trajectories):

    def __init__(self):
        Trajectories.__init__(self)

        self._temp_traj = dict()
        self._last_seen = dict()
        self.robot_pose = Pose()
        rospy.Subscriber(
            "/people_tracker/positions", PeopleTracker,
            self.pt_callback, None, 10
        )
        rospy.Subscriber(
            "/robot_pose", Pose, self.pose_callback, None, 10
        )

    def pose_callback(self, pose):
        self.robot_pose = pose

    def pt_callback(self, msg):
        for i, uuid in enumerate(msg.uuids):
            if uuid not in self._temp_traj:
                t = Trajectory(uuid)
                t.append_ros_pose(msg.poses[i], msg.header, self.robot_pose)
                self._temp_traj[uuid] = t
            else:
                t = self._temp_traj[uuid]
                t.append_ros_pose(msg.poses[i], msg.header, self.robot_pose)
            self._last_seen[uuid] = msg.header.stamp.secs

        self.check_completeness()

    def check_completeness(self):
        cur_time = rospy.Time.now()
        unvalidated_traj = dict()
        for uuid, secs in self._last_seen.iteritems():
            if (cur_time.secs - 3) >= secs:
                unvalidated_traj[uuid] = self._temp_traj[uuid]

        for uuid in unvalidated_traj:
            del self._temp_traj[uuid]
            del self._last_seen[uuid]

        if len(unvalidated_traj) > 0:
            self.traj.update(self._validate_trajectories(unvalidated_traj))


class OfflineTrajectories(Trajectories):

    def __init__(self):
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
            t.length = log['trajectory_length']
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
                           rospy.Time(i['header']['stamp']['secs'],
                                      i['header']['stamp']['nsecs']),
                           i['header']['frame_id']),
                    Pose(
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
