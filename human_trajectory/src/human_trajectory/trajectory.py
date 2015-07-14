#!/usr/bin/env python

import math
import rospy
import human_trajectory.msg
from geometry_msgs.msg import PoseStamped
from nav_msgs.msg import Path
from std_msgs.msg import Header


class Trajectory(object):

    def __init__(self, uuid):
        self.uuid = uuid
        self.humrobpose = []
        self.length = [0.0]
        self.sequence_id = 0
        self.header_seq = 1
        self._publish_index = 1
        self.too_long = False

    # check consistency between a new pose with
    # all stored poses (based on time stamp)
    def _check_poses_consistency(self, pose):
        if self.humrobpose != []:
            humrobpose = self.humrobpose[-1]
            temp = cmp(pose[0].header.stamp, humrobpose[0].header.stamp)
            if temp == 0:
                rospy.logwarn(
                    "%s has two poses at time %s",
                    self.uuid, str(pose[0].header.stamp)
                )
                prev_pose = None
                if len(self.humrobpose) >= 2:
                    prev_pose = self.humrobpose[-2]
                pose = self._delete_poses([humrobpose, pose], prev_pose)
        return pose

    # construct trajectory message based on Trajectory.msg
    def get_trajectory_message(self, chunked=False):
        from_index = 0
        to_index = len(self.humrobpose)

        if len(self.humrobpose) > len(self.length):
            rospy.logwarn("The overall length is not updated yet, ignoring the last pose.")
            to_index = len(self.length)
        if chunked:
            from_index = self._publish_index - 1
            self._publish_index = to_index
            if from_index == to_index:
                return None
            self.sequence_id += 1

        traj = human_trajectory.msg.Trajectory()
        traj.header = Header(self.header_seq, rospy.Time.now(), '/map')
        traj.uuid = self.uuid
        traj.start_time = self.humrobpose[from_index][0].header.stamp
        traj.end_time = self.humrobpose[to_index-1][0].header.stamp
        traj.trajectory = [i[0] for i in self.humrobpose[from_index:to_index]]
        traj.robot = [i[1] for i in self.humrobpose[from_index:to_index]]
        traj.complete = not chunked
        traj.sequence_id = self.sequence_id
        traj.trajectory_length = self.length[to_index-1] - self.length[from_index]
        traj.trajectory_displacement = self.trajectory_displacement
        traj.displacement_pose_ratio = self.displacement_pose_ratio

        self.header_seq += 1
        return traj

    # construct nav_msg/Path for visualization
    def get_nav_message(self):
        header = Header(self.header_seq, rospy.Time.now(), '/map')
        self.header_seq += 1
        return Path(header, [i[0] for i in self.humrobpose])

    # append human pose and robot pose
    def append_pose(self, human_pose, header, robot_pose, calc_length=False):
        pose = (PoseStamped(header, human_pose), robot_pose)
        self.humrobpose.append(self._check_poses_consistency(pose))
        start_time = self.humrobpose[0][0].header.stamp
        current_time = self.humrobpose[-1][0].header.stamp
        self.too_long = (current_time - start_time).secs > 3600
        if calc_length:
            self._calc_length()

    # sort and validate poses (delete poses which have the same
    # time stamp), also calculate length
    def validate_all_poses(self):
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
                                              self.humrobpose[j])
                    humrobpose.append(temp)
                    prev_pose = temp
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
                    result[0].pose.position.x - prev_pose[0].pose.position.x,
                    result[0].pose.position.y - prev_pose[0].pose.position.y,
                )
                ddist_prev2 = math.hypot(
                    i[0].pose.position.x - prev_pose[0].pose.position.x,
                    i[0].pose.position.y - prev_pose[0].pose.position.y,
                )

            if next_pose is not None:
                ddist_next1 = math.hypot(
                    result[0].pose.position.x - next_pose[0].pose.position.x,
                    result[0].pose.position.y - next_pose[0].pose.position.y,
                )
                ddist_next2 = math.hypot(
                    i[0].pose.position.x - next_pose[0].pose.position.x,
                    i[0].pose.position.y - next_pose[0].pose.position.y,
                )

            if (ddist_prev1 + ddist_next1) > (ddist_prev2 + ddist_next2):
                result = i
        return result

    # calculate travel distance from initial point to the end point
    def _calc_length(self):
        if len(self.humrobpose) >= 2:
            diff = len(self.humrobpose) - len(self.length)
            for i in range(diff):
                ps1 = self.humrobpose[len(self.length)-1][0]
                ps2 = self.humrobpose[len(self.length)][0]
                self.length.append(
                    self.length[-1] + math.hypot(
                        (ps1.pose.position.x - ps2.pose.position.x),
                        (ps1.pose.position.y - ps2.pose.position.y)
                    )
                )
            ps1 = self.humrobpose[0][0]
            ps2 = self.humrobpose[-1][0]
            self.trajectory_displacement = math.hypot(
                (ps1.pose.position.x - ps2.pose.position.x),
                (ps1.pose.position.y - ps2.pose.position.y)
            )
            self.displacement_pose_ratio = self.trajectory_displacement/float(len(self.humrobpose))
