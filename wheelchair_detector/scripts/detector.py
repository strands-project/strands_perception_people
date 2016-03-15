#!/usr/bin/env python

import rospy
import geometry_msgs.msg
import sensor_msgs.msg

import numpy as np

import net
import laser_utils


def callback(scan):
    #Check if we have any subscribers
    subscribed = False
    for p in publishers:
        if p.get_num_connections():
            subscribed = True
            break

    if subscribed:
        #Parse the laser data and
        scan_data = scan.ranges

        #Generate the cut outs from the scan
        cut_outs = laser_utils.generate_cut_outs(scan_data)

        #Push them through the network
        conf, off = network.forward(cut_outs)

        #Generate the actual predictions
        laserFoV = scan.angle_max - scan.angle_min
        predictions = laser_utils.pred2det_comb(scan_data, conf, off, thresh=threshold, fov=laserFoV)

        #flip the prediction axis to the ros coordinate system
        for c,preds in enumerate(predictions):

            detections = geometry_msgs.msg.PoseArray()
            detections.header = scan.header
            detections.poses = []

            for det in preds:
                pose = geometry_msgs.msg.Pose()
                pose.position.x = det[1]
                pose.position.y = -det[0]
                pose.position.z = 0

                pose.orientation.x = 0
                pose.orientation.y = 0
                pose.orientation.z = 0
                pose.orientation.w = 1
                detections.poses.append(pose)

            #Publish the detections
            publishers[c].publish(detections)





def detector():
    rospy.init_node('wheelchair_detector')

    #Parse parameters.
    ns = rospy.get_name() + '/'
    laser_topic = rospy.get_param(ns + 'laser_topic', '/scan')
    wheelchair_detection_topic = rospy.get_param(ns + 'wheelchair_detection_topic', '/wheelchair_detections')
    walker_detection_topic = rospy.get_param(ns + 'walker_detection_topic', '/walker_detections')
    class_agnostic_detection_topic = rospy.get_param(ns + 'class_agnostic_detection_topic', '/mobility_aid_detections')
    global threshold
    threshold = rospy.get_param(ns + 'detection_threshold', 0.5)

    #Load the network
    global network
    use_cudnn = rospy.get_param(ns + 'use_cudnn', True)
    network_file = rospy.get_param(ns + 'network_param_file')
    network = net.load_net_from_file(network_file, cudnn=use_cudnn)

    #Create a publisher for the detections
    global publishers
    publishers = []
    #The order here matters, it should be agnostic first and then in the order the network was trained. Default is: agnostic, wheelchair, walker
    publishers.append(rospy.Publisher(class_agnostic_detection_topic, geometry_msgs.msg.PoseArray, queue_size=10)) #class agnostic
    publishers.append(rospy.Publisher(wheelchair_detection_topic, geometry_msgs.msg.PoseArray, queue_size=10)) #Wheelchairs
    publishers.append(rospy.Publisher(walker_detection_topic, geometry_msgs.msg.PoseArray, queue_size=10)) #walkers

    #Subscribe to the laser scanner.
    rospy.Subscriber(laser_topic, sensor_msgs.msg.LaserScan, callback, queue_size=1)

    rospy.spin()

if __name__ == '__main__':
    detector()