#!/usr/bin/env python

import rospy
import human_trajectory.msg
from geometry_msgs.msg import PoseStamped, Pose, Point, Quaternion
from std_msgs.msg import Header

import json
import math


class Trajectory(object):

    def __init__(self, uuid):
        self.uuid = uuid
        self.humrobpose = []
        self.length = 0.0
        self.sequence_id = 1
        self._pose_seq = 1
        self.pps = 0            # poses per second

    # construct trajectory message based on Trajectory.msg
    def get_trajectory_message(self):
        traj = human_trajectory.msg.Trajectory()
        traj.header = Header(self.sequence_id, rospy.Time.now(), '/map')
        traj.uuid = self.uuid
        traj.start_time = self.humrobpose[0][0].header.stamp
        last = self.humrobpose[len(self.humrobpose)-1]
        traj.end_time = last[0].header.stamp
        traj.trajectory_length = self.length
        traj.complete = True
        traj.sequence_id = self.sequence_id
        for i in range(len(self.humrobpose)):
            (human, robot) = self.humrobpose[i]
            traj.trajectory.append(human)
            traj.robot.append(robot)
        self.sequence_id += 1
        return traj

    # transform pose, secs, nsecs info into PoseStamped
    # robot_pose into Pose and stored in tuple (PoseStamped, Pose)
    def append_pose(self, pose, secs, nsecs, robot_pose):
        human_pose = Pose(Point(pose['position']['x'],
                                pose['position']['y'],
                                pose['position']['z']),
                          Quaternion(pose['orientation']['x'],
                                     pose['orientation']['y'],
                                     pose['orientation']['z'],
                                     pose['orientation']['w']))
        header = Header(self._pose_seq, rospy.Time(secs, nsecs), '')
        robot_pose = Pose(Point(robot_pose['position']['x'],
                                robot_pose['position']['y'],
                                robot_pose['position']['z']),
                          Quaternion(robot_pose['orientation']['x'],
                                     robot_pose['orientation']['y'],
                                     robot_pose['orientation']['z'],
                                     robot_pose['orientation']['w']))
        self.humrobpose.append((PoseStamped(header, human_pose), robot_pose))
        self._pose_seq += 1

    # sort and validate poses (delete poses which have the same
    # time stamp), also calculate length
    def validate_poses(self):
        self.humrobpose = sorted(self.humrobpose,
                                 key=lambda pose: pose[0].header.stamp)
        i = 0
        humrobpose = []
        prev_pose = None
        while i < len(self.humrobpose):
            j = i + 1
            while j < len(self.humrobpose):
                if cmp(self.humrobpose[i][0].header.stamp,
                       self.humrobpose[j][0].header.stamp) != 0:
                    temp = self._delete_poses(self.humrobpose[i:j],
                                              prev_pose,
                                              self.humrobpose[j][0].pose)
                    humrobpose.append(temp)
                    prev_pose = temp[0].pose
                    break
                j += 1
            if j == len(self.humrobpose):
                temp = self._delete_poses(self.humrobpose[i:j], prev_pose)
                humrobpose.append(temp)
            i = j

        self.humrobpose = humrobpose
        if self.length == 0.0:
            self._calc_length()
        self._calc_pps()

    # for same poses at the same time, choose one that has minimal distance
    # with the previous pose and the next pose
    def _delete_poses(self, humrob, prev_pose=None, next_pose=None):
        i = 1
        result = humrob[0]
        while i < len(humrob):
            print "aaaaaa"
            ddist_prev1 = ddist_prev2 = 0
            ddist_next1 = ddist_next2 = 0
            if prev_pose is not None:
                ddist_prev1 = math.hypot(
                    result[0].pose.position.x - prev_pose.position.x,
                    result[0].pose.position.y - prev_pose.position.y,
                )
                ddist_prev2 = math.hypot(
                    humrob[i][0].pose.position.x - prev_pose.position.x,
                    humrob[i][0].pose.position.y - prev_pose.position.y,
                )

            if next_pose is not None:
                ddist_next1 = math.hypot(
                    result[0].pose.position.x - next_pose.position.x,
                    result[0].pose.position.y - next_pose.position.y,
                )
                ddist_next2 = math.hypot(
                    humrob[i][0].pose.position.x - next_pose.position.x,
                    humrob[i][0].pose.position.y - next_pose.position.y,
                )

            if (ddist_prev1 + ddist_next1) > (ddist_prev2 + ddist_next2):
                result = humrob[i]
            i += 1
        return result

    # calculate travel distance from initial point to the end point
    def _calc_length(self):
        length = 0.0
        if len(self.humrobpose) >= 2:
            for i in range(1, len(self.humrobpose)):
                j = i - 1
                ps1 = self.humrobpose[i][0]
                ps2 = self.humrobpose[j][0]
                length += math.hypot(
                    (ps1.pose.position.x - ps2.pose.position.x),
                    (ps1.pose.position.y - ps2.pose.position.y)
                )
        self.length = length

    # calculate average poses persecond
    def _calc_pps(self):
        inner_counter = outer_counter = 1
        prev_sec = self.humrobpose[0][0].header.stamp.secs
        for i, v in enumerate(self.humrobpose[1:]):
            sec = v[0].header.stamp.secs
            if prev_sec == sec:
                inner_counter += 1
            else:
                prev_sec = sec
                outer_counter += 1
        self.pps = inner_counter/outer_counter

    # create json for all data stored here
    def to_JSON(self):
        return json.dumps(self, default=lambda o: o.__dict__,
                          sort_keys=True, indent=4)
