#!/usr/bin/env python

"""Queries people_trajectory message store and re-uploads."""

__author__      = "Paul Duckworth"
__copyright__   = "Copyright 2015, University of Leeds"

import rospy
import os, sys
import pymongo
import datetime, time
from human_trajectory.trajectories import OfflineTrajectories, Trajectories
from mongodb_store.message_store import MessageStoreProxy


class Importer(object):
    def __init__(self):
        rospy.loginfo("Connecting to mongodb...")
        self._client = pymongo.MongoClient(rospy.get_param("mongodb_host"),
                                           rospy.get_param("mongodb_port"))

        self._store_client = MessageStoreProxy(collection=\
                "people_trajectory_new")


if __name__ == "__main__":
    rospy.init_node('people_trajectory_updater')

    # Loop over multiple dates.
    # Leave a sensible gap between t1 and t2.
    t1_st = datetime.datetime(2015, 5, 01, 00, 0, 0, 000000)
    t2_st = datetime.datetime(2015, 5, 02, 00, 0, 0, 000000)
    days = 1
    hrs = 24
    loops = days*(24/float(hrs))

    for i in range(int(loops)):
        t1 = t1_st + datetime.timedelta(hours=hrs*i)
        t2 = t2_st + datetime.timedelta(hours=hrs*i)
        print "Dates queried: %s, until: %s" %(t1, t2)

        #query_start_seconds = (t1-datetime.datetime(1970,1,1)).total_seconds()
        #query_end_seconds = (t2-datetime.datetime(1970,1,1)).total_seconds()
        date_query = {"_meta.inserted_at": {"$gte": t1, "$lt": t2}}

        st = time.time()
        ot = OfflineTrajectories(date_query)
        print "time taken to return = ", time.time() - st
        if ot.from_people_trajectory == None: continue

        i = Importer()
        st = time.time()
        print "started upload"
        for k, v in ot.traj.items():
            traj_msg = v.get_trajectory_message()
            p_id = i._store_client.update(message=traj_msg, \
                message_query={"uuid" : str(k)}, \
                meta = v._meta, upsert=True)

        print "time taken to upload = ", time.time()-st
