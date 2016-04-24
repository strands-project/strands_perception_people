#!/usr/bin/env python

import rospy
import pymongo
import datetime
from vision_people_logging.srv import DeleteUBD, DeleteUBDResponse


class VisionLoggingService(object):

    def __init__(self, name):
        self.del_srv = rospy.Service(name+'/delete', DeleteUBD, self.del_srv_cb)
        self._ubd_db = pymongo.MongoClient(
            rospy.get_param("mongodb_host", "localhost"),
            rospy.get_param("mongodb_port", 62345)
        ).message_store.upper_bodies

    def del_srv_cb(self, srv):
        query ={
            "header.stamp.secs": {
                "$gte": srv.start_time.secs,
                "$lt": srv.stop_time.secs
            }
        }
        count = self._ubd_db.find(query).count()
        response = True
        if count > 0:
            rospy.logwarn(
                "%d entries from upper_bodies collection will be deleted" % count
            )
            self._ubd_db.remove(query)
        else:
            rospy.loginfo(
                "No entries have been found, nothing to delete"
            )
            response = False
        rospy.sleep(0.1)
        return DeleteUBDResponse(response)


if __name__ == '__main__':
    rospy.init_node('vision_logging_service')
    VisionLoggingService(rospy.get_name())
    rospy.spin()
