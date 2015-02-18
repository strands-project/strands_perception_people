#!/usr/bin/env python

import rospy
import roslib
import rostopic
import geometry_msgs.msg
import genpy
from bson import Binary


class ToPoseArray():
    """docstring for ToPoseArray"""
    def __init__(self):
        detectors = rospy.get_param("~detectors")
        subscribers = []
        for elem in detectors:
            topic = detectors[elem]["topic"]
            point = detectors[elem]["point_name"]
            rospy.loginfo(
                "Found detector '%s' with topic '%s'. "
                "Waiting for topic type...",
                elem,
                topic
            )
            type = rostopic.get_topic_type(topic, True)[0]
            rospy.loginfo("Got topic type: %s.", type)
            pub = rospy.Publisher("~"+elem, geometry_msgs.msg.PoseArray, queue_size=0)
            subscribers.append(
                rospy.Subscriber(
                    topic,
                    roslib.message.get_message_class(type),
                    callback=self.callback,
                    callback_args={
                        "detector": elem,
                        "point_name": point,
                        "publisher": pub
                    }
                )
            )

    def callback(self, msg, args):
        pose_array = geometry_msgs.msg.PoseArray()
        msg = self.msg_to_document(msg)
        headers = []
        # Finding a header that contains a frame_id
        self.find_key(msg, "header", headers)
        for elem in headers:
            if not elem["frame_id"] == '':
                pose_array.header.seq = elem["seq"]
                pose_array.header.frame_id = elem["frame_id"]
                pose_array.header.stamp.secs = elem["stamp"]["secs"]
                pose_array.header.stamp.nsecs = elem["stamp"]["nsecs"]
        positions = []
        # Find x and y values for the given position key
        self.find_key(msg, args["point_name"], positions)
        for elem in positions:
            pose = geometry_msgs.msg.Pose()
            pose.position.x = elem['x']
            pose.position.y = elem['y']
            pose.position.z = 0.0     # Project to ground plane
            pose.orientation.w = 1.0  # Currently no orientation
            pose_array.poses.append(pose)
        args["publisher"].publish(pose_array)

    def find_key(self, dictionary, key, result_list):
        for elem in dictionary:
            if str(elem) == key:
                result_list.append(dictionary[elem])
            else:
                if isinstance(elem, dict):
                    self.find_key(elem, key, result_list)
                else:
                    if hasattr(dictionary[elem], '__iter__'):
                        try:
                            self.find_key(dictionary[elem], key, result_list)
                        except TypeError:
                            pass

    def msg_to_document(self, msg):
        """
        Given a ROS message, turn it into a (nested) dictionary.

        >>> from geometry_msgs.msg import Pose
        >>> msg_to_document(Pose())
        {'orientation': {'w': 0.0, 'x': 0.0, 'y': 0.0, 'z': 0.0},
        'position': {'x': 0.0, 'y': 0.0, 'z': 0.0}}

        :Args:
            | msg (ROS Message): An instance of a ROS message to convert
        :Returns:
            | dict : A dictionary representation of the supplied message.
        """

        d = {}

        slot_types = []
        if hasattr(msg, '_slot_types'):
            slot_types = msg._slot_types
        else:
            slot_types = [None] * len(msg.__slots__)

        for (attr, type) in zip(msg.__slots__, slot_types):
            d[attr] = self.sanitize_value(attr, getattr(msg, attr), type)

        return d

    def sanitize_value(self, attr, v, type):
        """
        De-rosify a msg.

        Internal function used to convert ROS messages into dictionaries.

        :Args:
            | attr(str): the ROS message slot name the value came from
            | v: the value from the message's slot to make into a MongoDB able type
            | type (str): The ROS type of the value passed, as given by the ressage slot_types member.
        :Returns:
            | A sanitized version of v.
        """

        if isinstance(v, str) and type == 'uint8[]':
            v = Binary(v)

        if isinstance(v, rospy.Message):
            return self.msg_to_document(v)
        elif isinstance(v, genpy.rostime.Time):
            return self.msg_to_document(v)
        elif isinstance(v, genpy.rostime.Duration):
            return self.msg_to_document(v)
        elif isinstance(v, list):
            result = []
            for t in v:
                if hasattr(t, '_type'):
                    result.append(self.sanitize_value(None, t, t._type))
                else:
                    result.append(self.sanitize_value(None, t, None))
            return result
        else:
            return v

if __name__ == '__main__':
    rospy.init_node('to_pose_array')
    tpa = ToPoseArray()
    rospy.spin()
