# -*- coding: utf-8 -*-

import rospy
from bayes_people_tracker.msg import PeopleTracker
from visualization_msgs.msg import MarkerArray
import math
import human_marker as hm
from people_msgs.msg import People, Person


def people_tracker_msg_from_posestamped(uuid, pose, vel, tf):
    try:
        t = tf.getLatestCommonTime("/base_link", pose.header.frame_id)
        pose.header.stamp = t
        base_pose = tf.transformPose("/base_link", pose)
    except Exception as e:
        rospy.logwarn(e)
        return PeopleTracker()
    p = PeopleTracker()
    p.uuids.append(uuid)
    p.poses.append(pose.pose)
    p.velocities.append(vel)
    p.header = pose.header
    p.distances.append(math.sqrt((base_pose.pose.position.x**2)+(base_pose.pose.position.y**2)))
    p.angles.append(math.atan2(base_pose.pose.position.y, base_pose.pose.position.x))
    p.min_distance = p.distances[0]
    p.min_distance_angle = p.angles[0]
    return p

def marker_array_from_people_tracker_msg(poses, target_frame):
    marker_array = MarkerArray()
    for i, pose in enumerate(poses):
        human = hm.createHuman(1, pose, target_frame)
        marker_array.markers.extend(human)
    return marker_array

def people_msg_from_pose_stamped(uuid, pose, vel):
    p = Person()
    p.name = uuid
    p.position.x=pose.pose.position.x
    p.position.y=pose.pose.position.y
    p.position.z=pose.pose.position.z

    p.velocity = vel
    p.reliability = 1.0

    ppl = People()
    ppl.header = pose.header
    ppl.people.append(p)
    return ppl

def get_velocity(current, pose, prev_pose, dt, A=.05):
    if dt > 0.0:
        current.x += (((pose.pose.position.x-prev_pose.pose.position.x)/dt)-current.x)*A
        current.y += (((pose.pose.position.y-prev_pose.pose.position.y)/dt)-current.y)*A
    return current
