#!/usr/bin/env python

import rospy
from ros_datacentre.message_store import MessageStoreProxy
import strands_perception_people_msgs.msg
import geometry_msgs.msg
import message_filters


class SaveLocations():
    def __init__(self):
        rospy.loginfo("Intialising logging")
        self.robot_pose = geometry_msgs.msg.Pose()
        self.dataset_name = "locations"
        self.msg_store = MessageStoreProxy(collection="people_perception")
        locations = message_filters.Subscriber(
            "/pedestrian_localisation/localisations",
            strands_perception_people_msgs.msg.PedestrianLocations,
        )
        pedestrian = message_filters.Subscriber(
            "/pedestrian_tracking/pedestrian_array",
            strands_perception_people_msgs.msg.PedestrianTrackingArray,
        )
        upper = message_filters.Subscriber(
            "/upper_body_detector/detections",
            strands_perception_people_msgs.msg.UpperBodyDetector,
        )
        rospy.Subscriber(
            "/robot_pose",
            geometry_msgs.msg.Pose,
            self.pose_callback,
            None,
            10
        )
        ts = message_filters.TimeSynchronizer([locations, pedestrian, upper], 100)
        ts.registerCallback(self.people_callback)

    def people_callback(self, pl, pt, up):
        rospy.loginfo("CALLED")
        if len(pl.distances) == 0:
            return
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
        insert.pedestrian_tracking = pt.pedestrians
        insert.upper_body_detections = up
        self.msg_store.insert(insert, meta)

    def pose_callback(self, pose):
        self.robot_pose = pose

if __name__ == '__main__':
    rospy.init_node('save_people_locations')
    sl = SaveLocations()
    rospy.spin()
