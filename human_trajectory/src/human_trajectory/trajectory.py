#!/usr/bin/env python

import rospy
import human_trajectory.msg
from geometry_msgs.msg import PoseStamped, Pose, Point, Quaternion

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
        # sorting secs
        self.pose, self.secs, self.nsecs, self.robot_pose = self.__insert_sort(
            self.pose, self.secs, self.nsecs, self.robot_pose)

        # sorting and validating nsecs
        i = 0
        pose = secs = nsecs = robot_pose = []
        while i < len(self.secs):
            start_sec = self.secs[i]
            j = i
            while j < len(self.secs):
                if self.secs[j] != start_sec:
                    tpose, tnsecs, tsecs, trobot = self.__insert_sort(
                        self.pose[i:j],
                        self.nsecs[i:j],
                        self.secs[i:j],
                        self.robot_pose[i:j]
                    )
                    tpose, tsecs, tnsecs, trobot = self.__validate_poses(
                        tpose, tsecs, tnsecs, trobot
                    )
                    pose = pose + tpose
                    secs = secs + tsecs
                    nsecs = nsecs + tnsecs
                    robot_pose = robot_pose + trobot
                    i = j
                    break
                j += 1
            if j == len(self.secs):
                tpose, tnsecs, tsecs, trobot = self.__insert_sort(
                    self.pose[i:j],
                    self.nsecs[i:j],
                    self.secs[i:j],
                    self.robot_pose[i:j]
                )
                tpose, tsecs, tnsecs, trobot = self.__validate_poses(
                    tpose, tsecs, tnsecs, trobot
                )
                pose = pose + tpose
                secs = secs + tsecs
                nsecs = nsecs + tnsecs
                robot_pose = robot_pose + trobot
                i = j

        self.pose = pose
        self.nsecs = nsecs
        self.secs = secs
        self.robot_pose = robot_pose

    def __insert_sort(self, pose, secs, nsecs, robot_pose):
        for i in range(1, len(secs)):
            j = i
            while j > 0 and secs[j - 1] > secs[j]:
                secs[j - 1], secs[j] = secs[j], secs[j - 1]
                nsecs[j - 1], nsecs[j] = nsecs[j], nsecs[j - 1]
                pose[j - 1], pose[j] = pose[j], pose[j - 1]
                robot_pose[j - 1], robot_pose[j] = \
                    robot_pose[j], robot_pose[j - 1]
                j -= 1
        return pose, secs, nsecs, robot_pose

    # this function assumes that all objects have the same length
    # and the secs are the same
    def __validate_poses(self, pose, secs, nsecs, robot_pose):
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
                    robot_pose = [robot_pose[i]]
                    break

                delta_i_x = abs(pose[i]['position']['x'] -
                                prev_pose['position']['x']) \
                    + abs(pose[i]['position']['x'] - next_pose['position']['x'])
                delta_i_y = abs(pose[i]['position']['y'] -
                                prev_pose['position']['y']) \
                    + abs(pose[i]['position']['y'] - next_pose['position']['y'])
                delta_index_x = abs(pose_index['position']['x'] -
                                    prev_pose['position']['x']) \
                    + abs(pose_index['position']['x'] -
                          next_pose['position']['x'])
                delta_index_y = abs(pose_index['position']['y'] -
                                    prev_pose['position']['y']) \
                    + abs(pose_index['position']['y'] -
                          next_pose['position']['y'])
                if (delta_i_x + delta_i_y) < (delta_index_x + delta_index_y):
                    del nsecs[i + 1 + index]
                    del pose[i + 1 + index]
                    del secs[i + 1 + index]
                    del robot_pose[i + 1 + index]
                    i -= 1
                else:
                    del nsecs[i]
                    del pose[i]
                    del secs[i]
                    del robot_pose[i]
                    i -= 1
            i += 1
        return pose, secs, nsecs, robot_pose

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
