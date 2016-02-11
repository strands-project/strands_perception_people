#!/usr/bin/env python
# -*- coding: utf-8 -*-
"""
Created on Tue Jul 28 17:52:31 2015

@author: cdondrup
"""

import rospy
from bayes_people_tracker.msg import PeopleTracker
from geometry_msgs.msg import PoseStamped, PoseArray, Vector3
import tf
from visualization_msgs.msg import MarkerArray
import people_tracker_emulator.msg_creator as mc
from people_msgs.msg import People
from std_srvs.srv import Empty, EmptyResponse
import uuid


class PeopleTrackerEmulator(object):
    _uuid = 'fred' # Only one person and no id so we make one up
    _prev_time = 0.0
    _prev_pose = None
    _vel = Vector3()

    def __init__(self, name):
        rospy.loginfo("Starting %s ..." % name)
        self.rate = rospy.Rate(30) # Ensure same speed as people tracker.
        rospy.Subscriber(rospy.get_param("~pose_in_topic","/human/transformed"), PoseStamped, self.callback, queue_size=1)
        self.pospub = rospy.Publisher(rospy.get_param("~positions","/people_tracker/positions"), PeopleTracker, queue_size=10)
        self.markpub = rospy.Publisher(rospy.get_param("~marker","/people_tracker/marker_array"), MarkerArray, queue_size=10)
        self.pplpub = rospy.Publisher(rospy.get_param("~people","/people_tracker/people"), People, queue_size=10)
        self.posepub = rospy.Publisher(rospy.get_param("~pose","/people_tracker/pose"), PoseStamped, queue_size=10)
        self.poseapub = rospy.Publisher(rospy.get_param("~pose_array","/people_tracker/pose_array"), PoseArray, queue_size=10)
        rospy.Service("~new_id", Empty, self.srv_cb)
        self.tf = tf.TransformListener()
        self.target_frame = rospy.get_param("~target_frame", "/map")
        rospy.loginfo("... all done")

    def srv_cb(self, req):
        self._uuid = uuid.uuid1().hex
        return EmptyResponse()

    def callback(self, msg):
        try:
            t = self.tf.getLatestCommonTime(self.target_frame, msg.header.frame_id)
            msg.header.stamp = t
            new_pose = self.tf.transformPose(self.target_frame, msg)
        except Exception as e:
            rospy.logwarn(e)
            return

        # Get velocity if previous pose and timestamp are available,
        # 0 otherwise
        if self._prev_pose and self._prev_time:
             dt = msg.header.stamp.to_sec() - self._prev_time
             self._vel = mc.get_velocity(self._vel, new_pose, self._prev_pose, dt)

        # Publish all the topics
        self.posepub.publish(new_pose)
        self.poseapub.publish(PoseArray(header=new_pose.header, poses=[new_pose.pose]))
        self.pospub.publish(mc.people_tracker_msg_from_posestamped(self._uuid, new_pose, self._vel, self.tf))
        self.markpub.publish(mc.marker_array_from_people_tracker_msg([new_pose.pose], self.target_frame))
        self.pplpub.publish(mc.people_msg_from_pose_stamped(self._uuid, new_pose, self._vel))

        # Update prev pose and time
        self._prev_time = msg.header.stamp.to_sec()
        self._prev_pose = new_pose

        # Ensure same rate as people tracker
        self.rate.sleep()


if __name__ == "__main__":
    rospy.init_node("people_tracker_emulator")
    p = PeopleTrackerEmulator(rospy.get_name())
    rospy.spin()