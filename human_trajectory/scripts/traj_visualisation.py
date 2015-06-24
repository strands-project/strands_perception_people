#!/usr/bin/env python

import argparse
import rospy
import interactive_markers.interactive_marker_server as ims
from visualization_msgs.msg import Marker, InteractiveMarkerControl
from geometry_msgs.msg import Point
from interactive_markers.interactive_marker_server import InteractiveMarker
from human_trajectory.trajectories import OfflineTrajectories


def trapezoidal_shaped_func(a, b, c, d, x):
    min_val = min(min((x - a)/(b - a), float(1.0)), (d - x)/(d - c))
    return max(min_val, float(0.0))


def r_func(x):
    a = -0.125
    b = 0.125
    c = 0.375
    d = 0.625
    x = 1.0 - x
    value = trapezoidal_shaped_func(a, b, c, d, x)
    return value


def g_func(x):
    a = 0.125
    b = 0.375
    c = 0.625
    d = 0.875
    x = 1.0 - x
    value = trapezoidal_shaped_func(a, b, c, d, x)
    return value


def b_func(x):
    a = 0.375
    b = 0.625
    c = 0.875
    d = 1.125
    x = 1.0 - x
    value = trapezoidal_shaped_func(a, b, c, d, x)
    return value


def average_velocity(traj):
    start = traj.humrobpose[0][0].header.stamp
    end = traj.humrobpose[-1][0].header.stamp
    delta = float((end-start).secs + 0.000000001 * (end-start).nsecs)
    avg_vel = 0.0
    if delta != 0.0:
        avg_vel = traj.length[-1] / delta
    return avg_vel


class TrajectoryVisualization(object):

    def __init__(self, marker_name):
        self._modulo = 3
        self.trajs = OfflineTrajectories()
        self._server = ims.InteractiveMarkerServer(marker_name)

    # choosing the visualisation you want, all trajectories, the longest,
    # shortest or average length of trajectories
    def visualize_trajectories(self, mode="all", avg_len=0, longest_len=0):
        counter = 0

        for traj in self.trajs.traj.itervalues():
            if len(traj.humrobpose) > 1:
                if mode == "average":
                    if abs(traj.length[-1]-avg_len) < (avg_len/10):
                        self.visualize_trajectory(traj)
                        counter += 1
                elif mode == "longest":
                    if abs(traj.length[-1]-longest_len) < (longest_len/10):
                        self.visualize_trajectory(traj)
                        counter += 1
                elif mode == "shortest":
                    if traj.length[-1] < 1:
                        self.visualize_trajectory(traj)
                        counter += 1
                else:
                    self.visualize_trajectory(traj)
                    counter += 1

        rospy.loginfo("Total Trajectories: " + str(len(self.trajs.traj)))
        rospy.loginfo("Printed trajectories: " + str(counter))

    def _update_cb(self, feedback):
        return

    def visualize_trajectory(self, traj):
        int_marker = self.create_trajectory_marker(traj)
        self._server.insert(int_marker, self._update_cb)
        self._server.applyChanges()

    def create_trajectory_marker(self, traj):
        # create an interactive marker for our server
        int_marker = InteractiveMarker()
        int_marker.header.frame_id = "/map"
        int_marker.name = traj.uuid

        int_marker.pose = traj.humrobpose[0][0].pose

        line_marker = Marker()
        line_marker.type = Marker.LINE_STRIP
        line_marker.scale.x = 0.05

        line_marker.color.r = r_func(average_velocity(traj))
        line_marker.color.g = g_func(average_velocity(traj))
        line_marker.color.b = b_func(average_velocity(traj))
        line_marker.color.a = 1.0

        line_marker.points = []
        while len(traj.humrobpose) / self._modulo > 5000:
            self._modulo += 1

        for i, pose in enumerate(traj.humrobpose):
            if i % self._modulo == 0:
                p = Point(
                    pose[0].pose.position.x - int_marker.pose.position.x,
                    pose[0].pose.position.y - int_marker.pose.position.y,
                    0.0
                )
                line_marker.points.append(p)

        self._modulo = 3

        # create a control which will move the box
        # this control does not contain any markers,
        # which will cause RViz to insert two arrows
        control = InteractiveMarkerControl()
        control.markers.append(line_marker)
        int_marker.controls.append(control)

        return int_marker

    def trajectory_visualization(self, mode):
        average_length = 0.0
        longest_length = -1.0
        short_trajectories = 0
        average_vel = 0.0
        highest_vel = -1.0

        for uuid, traj in self.trajs.traj.iteritems():
            average_length += float(traj.length[-1])
            avg_vel = average_velocity(traj)
            average_vel += average_velocity(traj)
            if traj.length[-1] < 1:
                short_trajectories += 1
            if longest_length < traj.length[-1]:
                longest_length = traj.length[-1]
            if highest_vel < avg_vel:
                highest_vel = avg_vel

        average_length /= float(len(self.trajs.traj))
        average_vel /= float(len(self.trajs.traj))
        rospy.loginfo("Average length of tracks is " + str(average_length))
        rospy.loginfo("Longest length of tracks is " + str(longest_length))
        rospy.loginfo("Short trajectories are " + str(short_trajectories))
        rospy.loginfo("Average velocity of tracks is " + str(average_vel))
        rospy.loginfo("Highest velocity of tracks is " + str(highest_vel))

        self.visualize_trajectories(mode, average_length, longest_length)


if __name__ == "__main__":
    mode = "all"
    parser = argparse.ArgumentParser(prog='trajectory')
    parser.add_argument("mode", help="[all | average | shortest | longest]")
    args = parser.parse_args()
    if args.mode != "":
        mode = args.mode

    rospy.init_node("human_trajectory_visualization")
    rospy.loginfo("Running Trajectory Visualization...")

    ta = TrajectoryVisualization('trajectory_visualization')
    ta.trajectory_visualization(mode)

    raw_input("Press 'Enter' to exit.")
