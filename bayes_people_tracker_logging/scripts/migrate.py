#!/usr/bin/env python
# -*- coding: utf-8 -*-
"""
Created on Tue Oct 27 10:04:57 2015

@author: cdondrup
"""

from __future__ import print_function

import rospy
import pymongo
from bayes_people_tracker.msg import PeopleTracker
from bayes_people_tracker_logging.msg import PeopleTrackerLogging
from mongodb_store.message_store import MessageStoreProxy
import mongodb_store.util as mdb_util
import genpy.dynamic
import numpy as np


COLLECTION = "people_perception"
PEOPLE_TRACKER = "people_tracker"

new_types = genpy.dynamic.generate_dynamic(PeopleTrackerLogging._type, PeopleTrackerLogging._full_text)

def check_for_update(new_message, client):
    to_update=[]
    db=client.message_store
    collection=db[COLLECTION]
    available = collection.find().distinct("_id")
    #for every people in the db
    for _id in available:
        #get one message
        search = {"_id": _id}
        aa = collection.find_one(search)
        a = aa[PEOPLE_TRACKER].keys()
        #count the difference
        c = len(list(set(new_message).difference(set(a))))
        if c > 0:
            #add to update list
            to_update.append(_id)
    return to_update, available

def create_proper_msg(message):
    size = len(message.uuids) # Assuming that uuids is always there. Bit hacky but there should be no other cases
    for slot, slot_type in zip(message.__slots__, message._slot_types):
        s = getattr(message, slot)
        if isinstance(s, list):
            new = []
            t = slot_type if not slot_type.endswith('[]') else slot_type[:-2]
            if t in ("string", "float64"):
                continue # Ignoring uuids and others
            t = new_types[t]
            for e in s:
                if not isinstance(e, dict):
                    continue
                ne = {}
                ne["_meta"] = {}
                ne["msg"] = e
                _, msg = mdb_util.document_to_msg_and_meta(ne,t)
                new.append(msg)
            if len(new) != size:
                new.extend([t()]*(size-len(new)))
            setattr(message, slot, new)
    return message

def update_people_tracker_msg(to_update, client):
    db=client.message_store
    collection=db[COLLECTION]

    msg_store = MessageStoreProxy(collection=COLLECTION)
    increment = float(len(to_update)) / 100.
    del_ids = []
    print ("Updating entries")
    for idx, _id in enumerate(to_update):
        search = {"_id": _id}
        entry = collection.find(search)[0] # only one ppl per unique id
        new_entry = {"msg": {}}
        if not entry.has_key("msg"):
            new_entry["_meta"] = entry["_meta"]
            entry.pop("_meta")
            new_entry["_id"] = entry["_id"]
            entry.pop("_id")
            for k,v in entry.items():
                new_entry["msg"][k] = v
        else:
            new_entry = entry

        meta, new_message = mdb_util.document_to_msg_and_meta(new_entry, PeopleTrackerLogging)

        new_message = create_proper_msg(new_message)
        new_message.people_tracker = create_proper_msg(new_message.people_tracker)
        del_ids.append(_id)
        msg_store.insert(new_message,meta)
        print(str(idx+1)+'/'+str(len(to_update))+':'+"[" + "=" * np.floor(float(idx) / increment) + ">" + " " * np.floor(((float(len(to_update)) - float(idx))/ increment)) + "]"+' %.0f %%'%np.floor(float(idx+1)/float(len(to_update))*100.), end='\r')
    print('')
    print ("done")

    increment = float(len(del_ids)) / 100.
    print ("Deleting updated entries")
    for idx, i in enumerate(del_ids):
        msg_store.delete(str(i))
        print(str(idx+1)+'/'+str(len(del_ids))+':'+"[" + "=" * np.floor(float(idx) / increment) + ">" + " " * np.floor(((float(len(del_ids)) - float(idx))/ increment)) + "]"+' %.0f %%'%np.floor(float(idx+1)/float(len(del_ids))*100.), end='\r')
    print('')
    print ("done")


if __name__ == '__main__':

    rospy.init_node('bayes_people_tracker_logging_migration')
    host = rospy.get_param("mongodb_host")
    port = rospy.get_param("mongodb_port")
    client = pymongo.MongoClient(host, port)

    to_pop=['_id','_meta']

    new_message = PeopleTracker().__slots__
    new_message_types = PeopleTracker()._slot_types

    print ('========= Current bayes people tracker definition ===========')
    print (new_message)
    print (new_message_types)


    to_update, available = check_for_update(new_message, client)

    print ('========= The following number of entries need to be updated ===========')
    print (str(len(to_update))+"/"+str(len(available))+": %.0f%%" %np.floor((float(len(to_update))/float(len(available))*100.)))

    print ('========= Updating database ===========')
    update_people_tracker_msg(to_update, client)
