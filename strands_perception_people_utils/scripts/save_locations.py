#!/usr/bin/env python

import rospy
from ros_datacentre.message_store import MessageStoreProxy
import strands_perception_people_msgs.msg
import geometry_msgs.msg


class SaveLocations():
    def __init__(self):
        rospy.loginfo("Intialising logging")
        self.robot_pose = geometry_msgs.msg.Pose()
        self.dataset_name = "locations"
        self.msg_store = MessageStoreProxy(collection="people_perception")
        rospy.Subscriber(
            "/pedestrian_localisation/localisations",
            strands_perception_people_msgs.msg.PedestrianLocations,
            self.people_callback,
            None,
            10
        )
        rospy.Subscriber(
            "/robot_pose",
            geometry_msgs.msg.Pose,
            self.pose_callback,
            None,
            10
        )

    def people_callback(self, pl):
        rospy.loginfo("people")
        if len(pl.distances) == 0:
            return
        rospy.loginfo("found")
        meta = {}
        meta["people"] = self.dataset_name
        insert = strands_perception_people_msgs.msg.Logging()
        insert.header = pl.header
        insert.ids = pl.ids
        insert.people = pl.poses
        insert.robot = self.robot_pose
        insert.scores = pl.scores
        insert.distances = pl.distances
        insert.angles = pl.angles
        insert.min_distance = pl.min_distance
        insert.min_distance_angle = pl.min_distance_angle
        self.msg_store.insert(pl, meta)

    def pose_callback(self, pose):
        self.pose = pose
        rospy.loginfo("robot_pose")

if __name__ == '__main__':
    rospy.init_node('save_people_locations')
    sl = SaveLocations()
    rospy.spin()
