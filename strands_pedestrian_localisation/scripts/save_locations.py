#!/usr/bin/env python

import rospy
from ros_datacentre.message_store import MessageStoreProxy
import strands_perception_people_msgs.msg
import geometry_msgs.msg

def callback(pl):
    meta = {}
    meta["people"] = dataset_name
    msg_store.insert(pl,meta)

def amcl_callback(pl):
    meta = {}
    meta["amcl"] = dataset_name
    msg_store.insert(pl,meta)

if __name__ == '__main__':
    rospy.init_node('save_people_locations')

    dataset_name = "locations"
    msg_store = MessageStoreProxy(collection="people_tracks")
    rospy.Subscriber("/pedestrian_localisation/localisations", strands_perception_people_msgs.msg.PedestrianLocations, callback, None, 10)
    rospy.Subscriber("/amcl_pose", geometry_msgs.msg.PoseWithCovarianceStamped, amcl_callback, None, 10)
    rospy.spin()
