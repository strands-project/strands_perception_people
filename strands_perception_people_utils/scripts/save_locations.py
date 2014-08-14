#!/usr/bin/env python

import rospy
from mongodb_store.message_store import MessageStoreProxy
import strands_perception_people_msgs.msg
import geometry_msgs.msg
import message_filters
import tf


class SaveLocations():
    def __init__(self):
        rospy.logdebug("Intialising logging")
        self.robot_pose = geometry_msgs.msg.Pose()
        self.tfl = tf.TransformListener()
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
        ts = message_filters.TimeSynchronizer(
            [locations, pedestrian, upper],
            100
        )
        ts.registerCallback(self.people_callback)

    def transform(self, source_frame, target_frame):
        rospy.logdebug("Looking up transform: %s -> %s", source_frame, target_frame)
        transform = geometry_msgs.msg.Transform()
        if self.tfl.frameExists(source_frame[1:]) \
                and self.tfl.frameExists(target_frame[1:]):
            try:
                t = self.tfl.getLatestCommonTime(
                    source_frame,
                    target_frame
                )
                translation, rotation = self.tfl.lookupTransform(
                    target_frame,
                    source_frame,
                    t
                )
                transform.translation.x = translation[0]
                transform.translation.y = translation[1]
                transform.translation.z = translation[2]
                transform.rotation.x = rotation[0]
                transform.rotation.y = rotation[1]
                transform.rotation.z = rotation[2]
                transform.rotation.w = rotation[3]
            except (
                tf.Exception,
                tf.ConnectivityException,
                tf.LookupException,
                tf.ExtrapolationException
            ) as e:
                rospy.logwarn(e)
        return transform

    def people_callback(self, pl, pt, up):
        if len(pl.distances) == 0:
            return
        meta = {}
        meta["people"] = self.dataset_name
        rospy.logdebug("Person detected. Logging to people_perception collection.")
        insert = strands_perception_people_msgs.msg.Logging()
        insert.header = pl.header
        insert.ids = pl.ids
        insert.uuids = pl.uuids
        insert.people = pl.poses
        insert.robot = self.robot_pose
        insert.scores = pl.scores
        insert.pedestrian_locations = pl
        insert.pedestrian_tracking = pt.pedestrians
        insert.upper_body_detections = up
        insert.target_frame = self.transform(pt.header.frame_id, pl.header.frame_id)
        if pl.header.frame_id == '/base_link':
            insert.base_link = insert.target_frame
        else:
            insert.base_link = self.transform(pt.header.frame_id, '/base_link')
        self.msg_store.insert(insert, meta)

    def pose_callback(self, pose):
        self.robot_pose = pose

if __name__ == '__main__':
    rospy.init_node('save_people_locations')
    sl = SaveLocations()
    rospy.spin()
