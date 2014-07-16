#!/usr/bin/env python

import rospy
from ros_datacentre.message_store import MessageStoreProxy
import strands_perception_people_msgs.msg
import geometry_msgs.msg
import message_filters
import tf
import thread


class SaveLocations():
    def __init__(self):
        rospy.logdebug("Intialising logging")
        target_frame = rospy.get_param("~target_frame", "/base_link")
        self.fps = rospy.Rate(25)
        self.source_frame = ""
        self.robot_pose = geometry_msgs.msg.Pose()
        self.transform = None
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
        thread.start_new_thread(self.tf_thread, (target_frame,))

    def tf_thread(self, target_frame):
        rospy.logdebug("tf thread started")
        while not rospy.is_shutdown():
            if self.tfl.frameExists(self.source_frame[1:]) \
                    and self.tfl.frameExists(target_frame[1:]):
                try:
                    t = self.tfl.getLatestCommonTime(
                        self.source_frame,
                        target_frame
                    )
                    translation, rotation = self.tfl.lookupTransform(
                        target_frame,
                        self.source_frame,
                        t
                    )
                    if not self.transform:
                        rospy.logdebug("First transform received")
                        self.transform = geometry_msgs.msg.Transform()
                    self.transform.translation.x = translation[0]
                    self.transform.translation.y = translation[1]
                    self.transform.translation.z = translation[2]
                    self.transform.rotation.x = rotation[0]
                    self.transform.rotation.y = rotation[1]
                    self.transform.rotation.z = rotation[2]
                    self.transform.rotation.w = rotation[3]
                except (
                    tf.Exception,
                    tf.ConnectivityException,
                    tf.LookupException,
                    tf.ExtrapolationException
                ) as e:
                    rospy.logwarn(e)
            self.fps.sleep()

    def people_callback(self, pl, pt, up):
        if not self.source_frame:
            self.source_frame = pt.header.frame_id
            rospy.logdebug(
                "Setting frame source frame to: %s",
                self.source_frame
            )
        if len(pl.distances) == 0 or not self.transform:
            return
        meta = {}
        meta["people"] = self.dataset_name
        rospy.logdebug("Person detected. Logging to people_perception collection.")
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
        insert.tf = self.transform
        self.msg_store.insert(insert, meta)

    def pose_callback(self, pose):
        self.robot_pose = pose

if __name__ == '__main__':
    rospy.init_node('save_people_locations')
    sl = SaveLocations()
    rospy.spin()
