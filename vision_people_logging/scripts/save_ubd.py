#!/usr/bin/env python

import rospy
from mongodb_store.message_store import MessageStoreProxy
from topological_logging_manager.msg import LoggingManager
from upper_body_detector.msg import UpperBodyDetector
from vision_people_logging.msg import LoggingUBD
import geometry_msgs.msg
import sensor_msgs.msg
import message_filters
from cv_bridge import CvBridge
import tf
import std_msgs.msg


class SaveLocations():
    def __init__(self, collname="upper_bodies"):
        self.collname = collname
        rospy.loginfo("Intialising UBD logging")
        self.robot_pose = geometry_msgs.msg.Pose()
        self.tfl = tf.TransformListener()
        self.msg_store = MessageStoreProxy(collection=collname)
        self.counter = 0

        # Always get the robot's position.
        rospy.Subscriber("/robot_pose", geometry_msgs.msg.Pose, self.posecb, None, 1)

        subs = [
            message_filters.Subscriber(rospy.get_param("~ubd", "/upper_body_detector/detections"), UpperBodyDetector),
            message_filters.Subscriber(rospy.get_param("~ubd_cent", "/upper_body_detector/bounding_box_centres"), geometry_msgs.msg.PoseArray),
            message_filters.Subscriber(rospy.get_param("~ubd_rgb", "/head_xtion/rgb/image_rect_color"), sensor_msgs.msg.Image),
            message_filters.Subscriber(rospy.get_param("~ubd_d", "/head_xtion/depth/image_rect_meters"), sensor_msgs.msg.Image),
        ]

        # Potentially also make use of the manager to tell us whether
        # we are allowed to record or not.
        mgr_topic = rospy.get_param("~manager_topic", "")
        if not mgr_topic == '':
            subs += [message_filters.Subscriber(mgr_topic, LoggingManager)]

        ts = message_filters.ApproximateTimeSynchronizer(subs, queue_size=5, slop=0.15)
        ts.registerCallback(self.cb)


    def posecb(self, pose):
        self.robot_pose = pose


    def cb(self, ubd, ubd_cent, rgb, d, *mgr):
        # Check for perission, if necessary:
        if len(mgr) and not mgr[0].log:
            return

        # UBD publishes an empty message even when there's no detection.
        if len(ubd_cent.poses) == 0:
            return

        self.counter += len(ubd_cent.poses)
        rospy.logdebug("{} Upper Body/ies detected. Logging to {} collection.".format(len(ubd_cent.poses), self.collname))

        log = LoggingUBD()
        log.header = ubd.header
        log.robot = self.robot_pose
        log.ubd_pos = self.to_world_all(ubd_cent)

        log.ubd = ubd
        log.ubd_rgb = list(self.cut_all(ubd, rgb))
        log.ubd_d = list(self.cut_all(ubd, d))

        self.msg_store.insert(log, meta={"people": "upper_bodies"})


    def cut_all(self, ubd, image, hfact=3):
        b = CvBridge()
        img = b.imgmsg_to_cv2(image)
        for x, y, w, h in zip(ubd.pos_x, ubd.pos_y, ubd.width, ubd.height):
            # Need to be careful for negative indices in conjunction with
            # numpy's (and thus OpenCV's) wrap-around.
            y2, x2 = y+hfact*h, x+w
            y1, x1 = max(y, 0), max(x, 0)
            tmp = img[y1:y2, x1:x2]
            rospy.logdebug("Before: {}x{}@{},{} ; After: {}x{}".format(w,hfact*h, x, y, tmp.shape[1], tmp.shape[0]))
            yield b.cv2_to_imgmsg(img[y1:y2, x1:x2])

    def to_world_all(self, pose_arr):
        transformed_pose_arr = list()
        try:
            fid = pose_arr.header.frame_id
            for cpose in pose_arr.poses:
                ctime = self.tfl.getLatestCommonTime(fid, "/map")
                pose_stamped = geometry_msgs.msg.PoseStamped(
                    std_msgs.msg.Header(1, ctime, fid), cpose
                )
                # Get the translation for this camera's frame to the world.
                # And apply it to all current detections.
                tpose = self.tfl.transformPose("/map", pose_stamped)
                transformed_pose_arr.append(tpose.pose.position)
        except tf.Exception as e:
            rospy.logwarn(e)
            # In case of a problem, just give empty world coordinates.
            return []

        return transformed_pose_arr

if __name__ == '__main__':
    rospy.init_node('save_ubd')
    sl = SaveLocations()
    rospy.spin()
    rospy.loginfo("Stored a total of {} UBDs to database.".format(sl.counter))
