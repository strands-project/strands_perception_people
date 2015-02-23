#!/usr/bin/env python

import random
import rospy
import pymongo
from geometry_msgs.msg import PoseStamped, Pose, Point, Quaternion
from std_msgs.msg import Header
from human_trajectory.trajectory import Trajectory


class Trajectories(object):

    def __init__(self):
        self.traj = dict()
        self.start_secs = -1
        self.average_pps = 0
        self.from_people_trajectory = False
        self._client = pymongo.MongoClient(
            rospy.get_param("datacentre_host", "localhost"),
            rospy.get_param("datacentre_port", 62345)
        )
        self._retrieve_logs()
        if not self.from_people_trajectory:
            self._validate_trajectories()
        rospy.loginfo("Data is ready...")

    def _validate_trajectories(self):
        rospy.loginfo("Validating data...")
        untraj = []
        mframe = 0
        for uuid in self.traj:
            self.traj[uuid].validate_poses()
            mframe += self.traj[uuid].pps
            if self.traj[uuid].length < 0.1 and uuid not in untraj:
                untraj.append(uuid)

        mframe = round(mframe/len(self.traj))
        self.average_pps = mframe
        mframe *= 5

        for uuid in self.traj:
            if len(self.traj[uuid].humrobpose) < mframe and uuid not in untraj:
                untraj.append(uuid)

        rospy.loginfo("Deleting noisy data...")
        for i, uuid in enumerate(untraj):
            del self.traj[uuid]

    # construct trajectories based on data from people_perception collection
    def _construct_from_people_perception(self):
        rospy.loginfo("Constructing data from people perception...")
        logs = self._client.message_store.people_perception.find()
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
        rospy.loginfo("Retrieving data from mongodb...")
        people_traj = self._client.message_store.people_trajectory.find()
        if people_traj.count() > 0:
            uuid = [
                people_traj[random.randint(0, people_traj.count()-1)]['uuid']
                for i in range(5)
            ]
            temp = [{"uuids": i} for i in uuid]
            logs = self._client.message_store.people_perception.find(
                {"$or": temp},
                {"uuids": 1}
            )
            if logs.count() > 0:
                self._construct_from_people_trajectory(people_traj)
                self.from_people_trajectory = True
                return

        self._construct_from_people_perception()
