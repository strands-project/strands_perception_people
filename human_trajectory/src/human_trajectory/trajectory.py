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
        self.length = [0.0]
        self.sequence_id = 1
        self._pose_seq = 1
        self.publish_index = -1

    # check consistency between a new pose with
    # all stored poses (based on time stamp)
    def _check_poses_consistency(self, pose):
        if self.humrobpose != []:
            humrobpose = self.humrobpose[-1][0]
            temp = cmp(pose[0].header.stamp, humrobpose.header.stamp)
            if temp is 0:
                rospy.logwarn(
                    "Traj with uuid %s has more than one pose at time %d, %d",
                    self.uuid, pose[0].header.stamp.secs,
                    pose[0].header.stamp.secs
                )
                prev_pose = None
                if len(self.humrobpose) >= 2:
                    prev_pose = self.humrobpose[-2][0]
                pose = self._delete_poses([humrobpose, pose], prev_pose)
            elif temp is -1:
                rospy.logwarn(
                    "Traj with uuid %s has unordered time stamp %d, %d",
                    self.uuid, pose[0].header.stamp.secs,
                    pose[0].header.stamp.secs
                )

        return pose

    # construct trajectory message based on Trajectory.msg
    def get_trajectory_message(self, interval=None):
        traj = human_trajectory.msg.Trajectory()
        traj.header = Header(self.sequence_id, rospy.Time.now(), '/map')
        traj.uuid = self.uuid

        to_index = len(self.humrobpose)
        if interval is not None:
            from_index = self.publish_index + 1
            for i, j in enumerate(self.humrobpose[from_index + 1:]):
                frm = self.humrobpose[from_index][0].header.stamp
                if (j[0].header.stamp - frm).secs >= interval:
                    to_index = i + from_index + 1
                    break
        else:
            from_index = 0

        traj.start_time = self.humrobpose[from_index][0].header.stamp
        traj.end_time = self.humrobpose[to_index-1][0].header.stamp
        if interval is not None:
            traj.complete = False
            length = self.length[to_index-1] - self.length[from_index]
            traj.trajectory_length = length
        else:
            traj.complete = True
            traj.trajectory_length = self.length[-1]

        traj.sequence_id = self.sequence_id
        humrobpose = self.humrobpose[from_index:to_index]
        for i in range(len(humrobpose)):
            (human, robot) = humrobpose[i]
            traj.trajectory.append(human)
            traj.robot.append(robot)
        self.sequence_id += 1
        if to_index < len(self.humrobpose) - 1:
            self.publish_index = to_index
        return traj

    def append_ros_pose(self, human_pose, header, robot_pose):
        pose = (PoseStamped(header, human_pose), robot_pose)
        self.humrobpose.append(self._check_poses_consistency(pose))
        self._calc_length(True)

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
    def validate_all_poses(self):
        if self._pose_seq is not 1:
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
            self._calc_length()

    # for same poses at the same time, choose one that has minimal distance
    # with the previous pose and the next pose
    def _delete_poses(self, poses, prev_pose=None, next_pose=None):
        result = poses[0]
        for j, i in enumerate(poses[1:]):
            ddist_prev1 = ddist_prev2 = 0
            ddist_next1 = ddist_next2 = 0
            if prev_pose is not None:
                ddist_prev1 = math.hypot(
                    result[0].pose.position.x - prev_pose.position.x,
                    result[0].pose.position.y - prev_pose.position.y,
                )
                ddist_prev2 = math.hypot(
                    i[0].pose.position.x - prev_pose.position.x,
                    i[0].pose.position.y - prev_pose.position.y,
                )

            if next_pose is not None:
                ddist_next1 = math.hypot(
                    result[0].pose.position.x - next_pose.position.x,
                    result[0].pose.position.y - next_pose.position.y,
                )
                ddist_next2 = math.hypot(
                    i.pose.position.x - next_pose.position.x,
                    i.pose.position.y - next_pose.position.y,
                )

            if (ddist_prev1 + ddist_next1) > (ddist_prev2 + ddist_next2):
                result = i
        return result

    # calculate travel distance from initial point to the end point
    def _calc_length(self, update=False):
        length = 0.0
        if len(self.humrobpose) >= 2:
            if update:
                ps1 = self.humrobpose[-2][0]
                ps2 = self.humrobpose[-1][0]
                self.length.append(
                    self.length[-1] + math.hypot(
                        (ps1.pose.position.x - ps2.pose.position.x),
                        (ps1.pose.position.y - ps2.pose.position.y)
                    )
                )
            else:
                for i in range(1, len(self.humrobpose)):
                    j = i - 1
                    ps1 = self.humrobpose[i][0]
                    ps2 = self.humrobpose[j][0]
                    length += math.hypot(
                        (ps1.pose.position.x - ps2.pose.position.x),
                        (ps1.pose.position.y - ps2.pose.position.y)
                    )
                    self.length.append(length)

    # create json for all data stored here
    def to_JSON(self):
        return json.dumps(self, default=lambda o: o.__dict__,
                          sort_keys=True, indent=4)
