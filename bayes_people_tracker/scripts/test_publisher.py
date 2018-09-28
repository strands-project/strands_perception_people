#!/usr/bin/env python

import rospy

from geometry_msgs.msg import PoseArray, Pose
from people_msgs.msg import People, Person
from math import pi, sin, cos


def talker():
    rospy.init_node('tracker_tester', anonymous=True)
    poses_pub = rospy.Publisher('/poses', PoseArray, queue_size=10)
    people_pub = rospy.Publisher('/people', People, queue_size=10)
    rate = rospy.Rate(10)
    angle = 0
    angle_step = .5
    scale = 2.0

    while not rospy.is_shutdown():
        angle += angle_step
        if angle > 360:
            angle = 0
        pose_array = PoseArray()
        pose_array.header.frame_id = '/map'
        pose_array.header.stamp = rospy.Time.now()

        current_pose = Pose()
        current_pose.position.x = cos(angle * pi / 180.0) * scale
        current_pose.position.y = sin(angle * pi / 180.0) * scale
        current_pose.orientation.w = 1.0
        pose_array.poses.append(current_pose)

        people = People()
        people.header.frame_id = '/base_link'
        people.header.stamp = rospy.Time.now()

        person = Person()
        person.position = current_pose.position
        person.name = 'hurga'
        person.reliability = 1.0
        people.people.append(person)

        poses_pub.publish(pose_array)
        people_pub.publish(people)
        rate.sleep()

if __name__ == '__main__':
    try:
        talker()
    except rospy.ROSInterruptException:
        pass
