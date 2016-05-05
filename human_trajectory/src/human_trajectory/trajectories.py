#!/usr/bin/env python

import tf
import rospy
import math
import pymongo
import message_filters
from std_msgs.msg import Header
import scipy.spatial.distance as dist_calc
from bayes_people_tracker.msg import PeopleTracker
from human_trajectory.trajectory import Trajectory
from geometry_msgs.msg import PoseStamped, Pose, Point, Quaternion, PoseArray


class OnlineTrajectories(object):

    def __init__(self, tracker_topic, ubd_topic):
        self.traj = dict()
        self.ubd_uuids = list()
        self._temp_traj = dict()
        self.complete_traj = dict()
        self.robot_pose = Pose()
        self._tfl = tf.TransformListener()
        # self.subs = rospy.Subscriber(
        #     topic, PeopleTracker, self.pt_callback, None, 30
        # )
        subs = [
            message_filters.Subscriber(tracker_topic, PeopleTracker),
            message_filters.Subscriber(ubd_topic, PoseArray),
        ]
        message_filters.ApproximateTimeSynchronizer(
            subs, queue_size=5, slop=0.15
        ).registerCallback(self.cb)

        rospy.Subscriber(
            "/robot_pose", Pose, self.pose_callback, None, 10
        )
        rospy.loginfo("Taking data from %s, validating with %s..." % (tracker_topic, ubd_topic))

    # delete trajs that appear fewer than in 20 frames or have length less than
    # 10 cm unless they have ubd
    def _validate_trajectories(self, traj):
        for uuid, t in traj.items():
            t.validate_all_poses()
            length_ratio = t.length[-1] / float(len(t.humrobpose)) 
            # 0.03 based on how fast a person moves within frames
            if (length_ratio < 0.03 and uuid not in self.ubd_uuids):
                del traj[uuid]
        return traj

    # get people position and ubd uuid
    def cb(self, people, ubd_cent):
        ubd_poses = self.pose_of_map_frame(ubd_cent)
        for i, uuid in enumerate(people.uuids):
            if uuid not in self._temp_traj:
                self._temp_traj[uuid] = Trajectory(uuid)
            self._temp_traj[uuid].append_pose(
                people.poses[i], people.header, self.robot_pose, True
            )
            if uuid not in self.ubd_uuids:
                for ubd_pose in ubd_poses:
                    dist = dist_calc.euclidean(
                        [ubd_pose.x, ubd_pose.y],
                        [people.poses[i].position.x, people.poses[i].position.y]
                    )
                    if dist < 0.3:
                        self.ubd_uuids.append(uuid)
                        break

        self.traj.update({
            uuid: traj
            for uuid, traj in self._temp_traj.iteritems()
            if len(traj.humrobpose) >= 20
        })
        self.ubd_uuids = [uuid for uuid in self.ubd_uuids if uuid in self.traj.keys()]
        self.check_completeness()

    def pose_of_map_frame(self, pose_arr):
        transformed_pose_arr = list()
        try:
            fid = pose_arr.header.frame_id
            for cpose in pose_arr.poses:
                ctime = self._tfl.getLatestCommonTime(fid, "/map")
                pose_stamped = PoseStamped(Header(1, ctime, fid), cpose)
                # Get the translation for this camera's frame to the world.
                # And apply it to all current detections.
                tpose = self._tfl.transformPose("/map", pose_stamped)
                transformed_pose_arr.append(tpose.pose.position)
        except tf.Exception:
            rospy.logwarn("Transformation from %s to /map can not be done at the moment" % pose_arr.header.frame_id)
            # In case of a problem, just give empty world coordinates.
            return []
        return transformed_pose_arr

    # get robot position
    def pose_callback(self, pose):
        self.robot_pose = pose

    # extract PeopleTracker message to obtain trajectories
    def pt_callback(self, msg):
        for i, uuid in enumerate(msg.uuids):
            if uuid not in self._temp_traj:
                self._temp_traj[uuid] = Trajectory(uuid)
            self._temp_traj[uuid].append_pose(
                msg.poses[i], msg.header, self.robot_pose, True
            )

        self.traj.update({
            uuid: traj
            for uuid, traj in self._temp_traj.iteritems()
            if len(traj.humrobpose) >= 20}
        )
        self.check_completeness()

    # check complete trajectories from _temp_traj and validate
    # those complete trajectories
    def check_completeness(self):
        cur_time = rospy.Time.now()
        for uuid, t in self.traj.items():
            delta = cur_time - t.humrobpose[-1][0].header.stamp
            if delta.secs >= 5:
                temp = self._validate_trajectories({uuid: t})
                if temp != {}:
                    self.complete_traj[uuid] = temp[uuid]

                del self.traj[uuid]
                del self._temp_traj[uuid]


class OfflineTrajectories(object):

    def __init__(self, query=None, size=10000):
        self.traj = dict()
        self.query = query
        self.size = size
        self.start_secs = -1
        self.client = pymongo.MongoClient(
            rospy.get_param("mongodb_host", "localhost"),
            rospy.get_param("mongodb_port", 62345)
        )
        self.from_people_trajectory = self._retrieve_logs()

        if self.from_people_trajectory is None:
            rospy.loginfo("Too much data. Needs re-running.")

        elif not self.from_people_trajectory:
            rospy.loginfo("Validating data...")
            self.traj = self._validate_trajectories(self.traj)
            rospy.loginfo("Data is ready...")

    # delete trajs that appear less than 5 secs or have length less than 0.1
    def _validate_trajectories(self, traj):
        for uuid, t in traj.items():
            t.validate_all_poses()
            if t.length[-1] < 0.1 or len(t.humrobpose) < 20:
                del traj[uuid]
        return traj

    # construct trajectories based on data from people_perception collection
    def _construct_from_people_perception(self, logs):
        rospy.loginfo("Constructing data from people perception...")
        for log in logs:
            for i, uuid in enumerate(log['uuids']):
                if uuid not in self.traj:
                    self.traj[uuid] = Trajectory(uuid)
                header = Header(
                    log['header']['seq'],
                    rospy.Time(log['header']['stamp']['secs'],
                               log['header']['stamp']['nsecs']),
                    log['header']['frame_id']
                )
                human_pose = Pose(
                    Point(log['people'][i]['position']['x'],
                          log['people'][i]['position']['y'],
                          log['people'][i]['position']['z']),
                    Quaternion(log['people'][i]['orientation']['x'],
                               log['people'][i]['orientation']['y'],
                               log['people'][i]['orientation']['z'],
                               log['people'][i]['orientation']['w'])
                )
                robot_pose = Pose(
                    Point(log['robot']['position']['x'],
                          log['robot']['position']['y'],
                          log['robot']['position']['z']),
                    Quaternion(log['robot']['orientation']['x'],
                               log['robot']['orientation']['y'],
                               log['robot']['orientation']['z'],
                               log['robot']['orientation']['w']))
                self.traj[uuid].append_pose(human_pose, header, robot_pose)

                if self.start_secs == -1 or \
                        log['header']['stamp']['secs'] < self.start_secs:
                    self.start_secs = log['header']['stamp']['secs']

    # construct trajectories based on data from people_trajectory collection
    def _construct_from_people_trajectory(self, logs):
        rospy.loginfo("Constructing data from people trajectory...")
        for log in logs:
            t = Trajectory(str(log['uuid']))
            t.length = [0.0 for i in range(len(log['robot']))]
            t.length[-1] = log['trajectory_length']
            t.sequence_id = log['sequence_id']
            t._meta = log['_meta']

            robot_pose = [
                Pose(
                    Point(i['position']['x'],
                          i['position']['y'],
                          i['position']['z']),
                    Quaternion(i['orientation']['x'],
                               i['orientation']['y'],
                               i['orientation']['z'],
                               i['orientation']['w']))
                for i in log['robot']
            ]
            human_pose = [
                PoseStamped(
                    Header(i['header']['seq'],
                           rospy.Time(i['header']['stamp']['secs'],
                                      i['header']['stamp']['nsecs']),
                           i['header']['frame_id']),
                    Pose(
                        Point(i['pose']['position']['x'],
                              i['pose']['position']['y'],
                              i['pose']['position']['z']),
                        Quaternion(i['pose']['orientation']['x'],
                                   i['pose']['orientation']['y'],
                                   i['pose']['orientation']['z'],
                                   i['pose']['orientation']['w'])))
                for i in log['trajectory']
            ]

            t.trajectory_displacement = math.hypot(
                (human_pose[0].pose.position.x - human_pose[-1].pose.position.x),
                (human_pose[0].pose.position.y - human_pose[-1].pose.position.y)
                )
            t.displacement_pose_ratio = t.trajectory_displacement / float(len(human_pose))

            t.humrobpose = zip(human_pose, robot_pose)
            self.traj[log['uuid']] = t
            traj_start = log['trajectory'][0]['header']['stamp']['secs']
            if self.start_secs == -1 or traj_start < self.start_secs:
                self.start_secs = traj_start

    # retrieve trajectory from mongodb
    def _retrieve_logs(self):
        rospy.loginfo("Getting trajectories from database with query %s" % self.query)
        total_traj = self.client.message_store.people_trajectory.find(self.query).count()
        rospy.loginfo("Number of trajs returned = %s " % total_traj)
        if int(total_traj) > self.size:
            rospy.logwarn("Total trajectories retrieved is greater than %d" % self.size)
            rospy.loginfo("Limiting the retrieved trajectories to %d..." % self.size)
        people_traj = self.client.message_store.people_trajectory.find(self.query).limit(self.size)

        if people_traj.count() > 0:
            self._construct_from_people_trajectory(people_traj)
            # if data comes from people_trajectory db then the data
            # has been validated
            return True

        rospy.loginfo("No data in people trajectory collection, looking data in people perception collection...")
        total_poses = self.client.message_store.people_perception.find(self.query).count()
        if int(total_poses) > self.size * 100:
            rospy.logwarn("Total poses retrieved is greater than %d" % (self.size * 100))
            rospy.loginfo("Limiting the retrieved poses to %d..." % (self.size * 100))
        logs = self.client.message_store.people_perception.find(self.query).limit(self.size * 100)

        self._construct_from_people_perception(logs)
        return False
