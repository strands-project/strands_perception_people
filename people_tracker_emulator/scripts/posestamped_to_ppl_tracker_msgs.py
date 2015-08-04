#!/usr/bin/env python
# -*- coding: utf-8 -*-
"""
Created on Tue Jul 28 17:52:31 2015

@author: cdondrup
"""

import rospy
from bayes_people_tracker.msg import PeopleTracker
from geometry_msgs.msg import PoseStamped, PoseArray
import tf
from visualization_msgs.msg import MarkerArray
import people_tracker_emulator.msg_creator as mc
from people_msgs.msg import People


class PeopleTrackerEmulator(object):
    _uuid = 'fred' # Only one person and no id so we make one up
    _prev_time = 0.0
    _prev_pose = None

    def __init__(self, name):
        rospy.loginfo("Starting %s ..." % name)
        rospy.Subscriber(rospy.get_param("~pose_in_topic","/human/transformed"), PoseStamped, self.callback)
        self.pospub = rospy.Publisher(rospy.get_param("~positions","/people_tracker/positions"), PeopleTracker, queue_size=10)
        self.markpub = rospy.Publisher(rospy.get_param("~marker","/people_tracker/marker_array"), MarkerArray, queue_size=10)
        self.pplpub = rospy.Publisher(rospy.get_param("~people","/people_tracker/people"), People, queue_size=10)
        self.posepub = rospy.Publisher(rospy.get_param("~pose","/people_tracker/pose"), PoseStamped, queue_size=10)
        self.poseapub = rospy.Publisher(rospy.get_param("~pose_array","/people_tracker/pose_array"), PoseArray, queue_size=10)
        self.tf = tf.TransformListener()
        self.target_frame = rospy.get_param("~target_frame", "/map")
        rospy.loginfo("... all done")

    def callback(self, msg):
        try:
            t = self.tf.getLatestCommonTime(self.target_frame, msg.header.frame_id)
            msg.header.stamp = t
            new_pose = self.tf.transformPose(self.target_frame, msg)
        except Exception as e:
            rospy.logwarn(e)
            return

        self.posepub.publish(new_pose)
        pa = PoseArray()
        pa.header = new_pose.header
        pa.poses.append(new_pose.pose)
        self.poseapub.publish(pa)
        p = mc.people_tracker_msg_from_posestamped(self._uuid, new_pose, self.tf)
        self.pospub.publish(p)
        marker_array = mc.marker_array_from_people_tracker_msg(p, self.target_frame)
        self.markpub.publish(marker_array)
        if self._prev_pose and self._prev_time:
            dt = msg.header.stamp.to_sec() - self._prev_time
            ppl = mc.people_msg_from_pose_stamped(self._uuid, new_pose, self._prev_pose, dt)
            self.pplpub.publish(ppl)
        self._prev_time = msg.header.stamp.to_sec()
        self._prev_pose = new_pose


if __name__ == "__main__":
    rospy.init_node("people_tracker_emulator")
    p = PeopleTrackerEmulator(rospy.get_name())
    rospy.spin()