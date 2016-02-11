#!/usr/bin/env python

import rospy
import geometry_msgs.msg
import sensor_msgs.msg

import numpy as np

def callback(scan):
    #Parse the laser data and
    data = scan.ranges

    wheelchair_poses =[]
    ########################################
    # TODO detect wheelchairs in this data #
    ########################################
    # Dummy detections follow

    wheelchair_count = np.random.randint(0,3)
    for w in xrange(wheelchair_count):
        wheelchair_pose = geometry_msgs.msg.Pose()
        wheelchair_pose.position.x = np.random.uniform(-1,1)
        wheelchair_pose.position.y = np.random.uniform(-1,1)
        wheelchair_pose.position.z = 0

        wheelchair_pose.orientation.x = 0
        wheelchair_pose.orientation.y = 0
        wheelchair_pose.orientation.z = 0
        wheelchair_pose.orientation.w = 1
        wheelchair_poses.append(wheelchair_pose)

    # TODO END
    ########################################

    #Publish the detections
    detections = geometry_msgs.msg.PoseArray()
    detections.header = scan.header
    detections.poses = wheelchair_poses

    publisher.publish(detections)

def detector():
    rospy.init_node('wheelchair_detector')

    #Parse parameters.
    ns = rospy.get_name() + '/'
    laser_topic     = rospy.get_param(ns + 'laser_topic', '/scan')
    detection_topic = rospy.get_param(ns + 'detection_topic', '/wheelchair_detections')

    #Create a publisher for the detections
    global publisher
    publisher = rospy.Publisher(detection_topic, geometry_msgs.msg.PoseArray, queue_size=10)

    #Subscribe to the laser scanner.
    rospy.Subscriber(laser_topic, sensor_msgs.msg.LaserScan, callback)
    print('Wheelchair detector started.\nATTENTION! Funtionality is still missing.\nCurrently only random detections are published.')

    rospy.spin()

if __name__ == '__main__':
    detector()