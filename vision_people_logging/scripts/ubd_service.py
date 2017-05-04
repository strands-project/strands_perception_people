#!/usr/bin/env python

import rospy
import pymongo
from vision_people_logging.msg import LoggingUBD
from vision_people_logging.save_ubd import SaveUBD
from mongodb_store.message_store import MessageStoreProxy
from vision_people_logging.srv import FindUBD, FindUBDResponse
from vision_people_logging.srv import DeleteUBD, DeleteUBDResponse
from vision_people_logging.srv import CaptureUBD, CaptureUBDResponse


class VisionLoggingService(object):

    def __init__(self, name):
        rospy.loginfo("Creating a delete service under %s/delete" % name)
        self.del_srv = rospy.Service(name+'/delete', DeleteUBD, self.del_srv_cb)
        rospy.loginfo("Creating a find service under %s/find" % name)
        self.find_srv = rospy.Service(name+'/find', FindUBD, self.find_srv_cb)
        rospy.loginfo("Creating a capture service under %s/capture" % name)
        self.cptr_srv = rospy.Service(name+'/capture', CaptureUBD, self.capture_srv_cb)
        rospy.loginfo("Connecting to mongodb message_store - upper_bodies collection...")
        self._ubd_db = pymongo.MongoClient(
            rospy.get_param("mongodb_host", "localhost"),
            rospy.get_param("mongodb_port", 62345)
        ).message_store.upper_bodies
        self._msg_store = MessageStoreProxy(collection="upper_bodies")
        self._pub = rospy.Publisher(name+'/log', LoggingUBD, queue_size=10)
        self.save_ubd = SaveUBD(is_stored=False)

    def publish_captured_log(self):
        while not rospy.is_shutdown():
            self._pub.publish(self.save_ubd.log)
            rospy.sleep(0.1)

    def capture_srv_cb(self, srv):
        rospy.loginfo("Got a request to capture a snapshot of UBD")
        count = len(self.save_ubd.obj_ids)
        self.save_ubd.is_stored = True
        st = rospy.Time.now()
        et = rospy.Time.now()
        while len(self.save_ubd.obj_ids) <= count and (et - st).secs < 5:
            rospy.loginfo("Trying to capture a snapshot of UBD...")
            rospy.sleep(0.05)
            et = rospy.Time.now()
        self.save_ubd.is_stored = False
        if len(self.save_ubd.obj_ids) - count > 1:
            rospy.logwarn(
                "%d snapshots have been captured" % (len(self.save_ubd.obj_ids) - count)
            )
        elif len(self.save_ubd.obj_ids) - count == 0:
            rospy.loginfo("No snapshot has been captured")
        rospy.sleep(0.1)
        return CaptureUBDResponse(
            self.save_ubd.obj_ids[count:len(self.save_ubd.obj_ids)]
        )

    def find_srv_cb(self, srv):
        rospy.loginfo("Got a request to query UBD from db...")
        logs = list()
        if len(srv.obj_ids) > 0:
            for _id in srv.obj_ids:
                log = self._msg_store.query_id(_id, LoggingUBD._type)
                if log is not None:
                    logs.append(log[0])
        elif srv.start_time is not None or srv.stop_time is not None:
            query = {
                "header.stamp.secs": {
                    "$gte": srv.start_time.secs,
                    "$lt": srv.stop_time.secs
                }
            }
            logs = self._msg_store.query(LoggingUBD._type, query)
            rospy.loginfo("Found %d entry(ies)..." % len(logs))
            logs = [log[0] for log in logs]
        rospy.sleep(0.1)
        return FindUBDResponse(logs)

    def del_srv_cb(self, srv):
        rospy.loginfo("Got a request to delete UBD entries from db...")
        if len(srv.obj_ids) > 0:
            count = list()
            for _id in srv.obj_ids:
                count.append(self._msg_store.delete(_id))
            count = [i for i in count if i]
            response = True
            if len(count) > 0:
                rospy.logwarn(
                    "%d entry(ies) have been deleted based on %s" % (len(count), str(srv.obj_ids))
                )
            else:
                rospy.loginfo(
                    "No entry has been found, nothing to delete"
                )
                response = False
        elif srv.start_time is not None or srv.stop_time is not None:
            query = {
                "header.stamp.secs": {
                    "$gte": srv.start_time.secs,
                    "$lt": srv.stop_time.secs
                }
            }
            count = self._ubd_db.find(query).count()
            response = True
            if count > 0:
                rospy.logwarn(
                    "%d entry(ies) from upper_bodies collection will be deleted" % count
                )
                self._ubd_db.remove(query)
            else:
                rospy.loginfo(
                    "No entry has been found, nothing to delete"
                )
                response = False
        rospy.sleep(0.1)
        return DeleteUBDResponse(response)


if __name__ == '__main__':
    rospy.init_node('vision_logging_service')
    vls = VisionLoggingService(rospy.get_name())
    vls.publish_captured_log()
    rospy.spin()
