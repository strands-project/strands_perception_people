#!/usr/bin/env python

import rospy
from mongodb_store.message_store import MessageStoreProxy
from bayes_people_tracker.msg import PeopleTracker
#from mdl_people_tracker.msg import MdlPeopleTrackerArray, MdlPeopleTracker
#from upper_body_detector.msg import UpperBodyDetector
from bayes_people_tracker_logging.msg import Logging
import geometry_msgs.msg
#import message_filters
import tf


class SaveLocations():
    def __init__(self):
        rospy.logdebug("Intialising logging")
        self.robot_pose = geometry_msgs.msg.Pose()
        self.tfl = tf.TransformListener()
        self.dataset_name = "locations"
        self.msg_store = MessageStoreProxy(collection="people_perception")
        rospy.Subscriber(
            "/people_tracker/positions",
            PeopleTracker,
            self.people_callback
        )
#        locations = message_filters.Subscriber(
#            "/people_tracker/positions",
#            PeopleTracker,
#        )
#        people = message_filters.Subscriber(
#            "/mdl_people_tracker/people_array",
#            MdlPeopleTrackerArray,
#        )
#        upper = message_filters.Subscriber(
#            "/upper_body_detector/detections",
#            UpperBodyDetector,
#        )
        rospy.Subscriber(
            "/robot_pose",
            geometry_msgs.msg.Pose,
            self.pose_callback,
            None,
            10
        )
#        ts = message_filters.ApproximateTimeSynchronizer(
#            0.5,
#            [locations, people, upper],
#            10
#        )
#        ts.registerCallback(self.people_callback)

    def transform(self, source_frame, target_frame, time):
        rospy.logdebug(
            "Looking up transform: %s -> %s",
            source_frame,
            target_frame
        )
        transform = geometry_msgs.msg.Transform()
        if self.tfl.frameExists(source_frame[1:]) \
                and self.tfl.frameExists(target_frame[1:]):
            try:
                self.tfl.waitForTransform(
                    target_frame,
                    source_frame,
                    time,
                    rospy.Duration(0.1)
                )
                translation, rotation = self.tfl.lookupTransform(
                    target_frame,
                    source_frame,
                    time
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

#    def people_callback(self, pl, pt, up):
    def people_callback(self, pl):
        if len(pl.distances) == 0:
            return
        meta = {}
        meta["people"] = self.dataset_name
        rospy.logdebug(
            "Person detected. "
            "Logging to people_perception collection."
        )
        insert = Logging()
        insert.header = pl.header
        insert.uuids = pl.uuids
        insert.people = pl.poses
        insert.robot = self.robot_pose
        insert.people_tracker = pl
#        insert.mdl_people_tracker = pt.people
#        insert.upper_body_detections = up
#        insert.target_frame = self.transform(
#            pt.header.frame_id,
#            pl.header.frame_id,
#            pt.header.stamp
#        )
#        if pl.header.frame_id == '/base_link':
#            insert.base_link = insert.target_frame
#        else:
#            insert.base_link = self.transform(
#                pt.header.frame_id,
#                '/base_link',
#                pt.header.stamp
#            )
        self.msg_store.insert(insert, meta)

    def pose_callback(self, pose):
        self.robot_pose = pose

if __name__ == '__main__':
    rospy.init_node('save_people_locations')
    sl = SaveLocations()
    rospy.spin()
