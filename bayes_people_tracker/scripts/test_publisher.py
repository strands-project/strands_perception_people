#!/usr/bin/env python

import rospy

from geometry_msgs.msg import PoseArray, Pose
from people_msgs.msg import People, Person
from math import pi, sin, cos, sqrt
from threading import Thread, RLock
from random import randint, random
from numpy import random as nprnd
from collections import defaultdict

class Walker(Thread):
    def __init__(
        self, name,
        start_coords=[0, 0],
        arena=[-10, -10, 10, 10]
    ):
        Thread.__init__(self, name=name)
        self.start_coords = start_coords
        self.rate = 50.0
        self.position = list(self.start_coords)
        self.arena = arena
        self.lock = RLock()
        self.pub = rospy.Publisher(
            'tracker_tester/ground_truth/%s' % name,
            PoseArray,
            queue_size=10
            )

    def _crop(self):
        self.position[0] = (
            self.position[0]
            if self.position[0] <= self.arena[2]
            else self.arena[2]
        )
        self.position[1] = (
            self.position[1]
            if self.position[1] <= self.arena[3]
            else self.arena[3]
        )
        self.position[0] = (
            self.position[0]
            if self.position[0] >= self.arena[0]
            else self.arena[0]
        )
        self.position[1] = (
            self.position[1]
            if self.position[1] >= self.arena[1]
            else self.arena[1]
        )

    def _update(self):
        pass

    def run(self):
        rate = rospy.Rate(self.rate)
        while not rospy.is_shutdown():

            with self.lock:
                self._update()
                self._crop()

            person = self.get_pose()
            ppl = PoseArray()
            ppl.header.frame_id = '/base_link'
            ppl.header.stamp = rospy.Time.now()
            ppl.poses.append(person)
            self.pub.publish(ppl)
            rate.sleep()

    def get_pose(self):
        pose = Pose()
        with self.lock:
            pose.position.x = self.position[0]
            pose.position.y = self.position[1]
        pose.orientation.w = 1.0
        return pose

    def get_person(self):
        person = Person()
        person.name = self.name
        with self.lock:
            person.position.x = self.position[0]
            person.position.y = self.position[1]
        return person


class CircularWalker(Walker):
    def __init__(
        self, name,
        start_coords=[0, 0],
        arena=[-5, -5, 5, 5],
        omega_deg=5, scale=[2, 1]
    ):
        Walker.__init__(
            self, name, start_coords, arena)
        self.omega = omega_deg / self.rate
        self.angle = randint(0, 359)
        self.scale = scale

    def _update(self):
        self.angle += self.omega
        if self.angle >= 360:
            self.angle = 0
        self.position[0] = (
            self.start_coords[0] +
            cos(self.angle * pi / 180.0) * self.scale[0])
        self.position[1] = (
            self.start_coords[1] +
            sin(self.angle * pi / 180.0) * self.scale[1])


class LinearWalker(Walker):
    def __init__(
        self, name,
        start_coords=[0, 0],
        arena=[-10, -10, 10, 10],
        speed=.4, dirchange_prob=.1
    ):
        Walker.__init__(
            self, name, start_coords, arena)
        self.speed = speed / self.rate
        self.dirchange_prob = dirchange_prob / self.rate
        self._newdir()

    def _newdir(self):
        dir = [random() - .5, random() - .5]
        l = sqrt((dir[0] ** 2) + (dir[1] ** 2))

        self.direction = [
            dir[0] / l,
            dir[1] / l
        ]

    def _update(self):
        if (random() <= self.dirchange_prob):
            rospy.loginfo("%s changes direction" % self.name)
            self._newdir()

        self.position[0] += self.direction[0] * self.speed
        self.position[1] += self.direction[1] * self.speed


class Sensor(Thread):
    def __init__(
        self, name, walkers,
        anonymous=True,
        noise=[.4, .4],
        avg_rate=10, std_rate=2,
        prob_switch_visible=0.0,
        prob_visible=1.0,
        delayed_start=0.0
    ):
        Thread.__init__(self, name=name)
        self.noise = noise
        self.avg_rate = avg_rate
        self.std_rate = std_rate
        self.anonymous = anonymous
        self.prob_switch_visible = prob_switch_visible / self.avg_rate
        self.prob_visible = prob_visible
        self.visible = defaultdict(lambda: random() < self.prob_visible)
        self.walkers = walkers
        self.delayed_start = delayed_start
        self.pub_poses = rospy.Publisher(
            'tracker_tester/sensor/pose_array/%s' % name,
            PoseArray,
            queue_size=10
            )
        self.pub_people = rospy.Publisher(
            'tracker_tester/sensor/people/%s' % name,
            People,
            queue_size=10
            )

    def run(self):
        self.start_time = rospy.Time.now()
        while not rospy.is_shutdown():
            if (
                rospy.Time.now() - self.start_time <
                rospy.Duration(self.delayed_start)
            ):
                rospy.sleep(0.1)
                continue
            poses = []
            persons = []

            for w in self.walkers:
                if random() < self.prob_switch_visible:
                    self.visible[w.name] = (random() < self.prob_visible)
                    if self.visible[w.name]:
                        rospy.loginfo(
                            "switch sensor %s to see %s" % (self.name, w.name))
                    else:
                        rospy.loginfo(
                            "switch sensor %s to NOT see %s" % (self.name, w.name))
                if self.visible[w.name]:
                    ps = w.get_pose()
                    ps.position.x += nprnd.randn() * self.noise[0]
                    ps.position.y += nprnd.randn() * self.noise[1]
                    poses.append(ps)
                    p = w.get_person()
                    p.position.x = ps.position.x
                    p.position.y = ps.position.y
                    if self.anonymous:
                        p.name = ''
                    persons.append(p)

            pa = PoseArray()
            pa.header.frame_id = '/base_link'
            pa.header.stamp = rospy.Time.now()
            pa.poses = poses
            self.pub_poses.publish(pa)

            ppl = People()
            ppl.header.frame_id = '/base_link'
            ppl.header.stamp = rospy.Time.now()
            ppl.people = persons
            self.pub_people.publish(ppl)

            sleep_rate = (
                self.avg_rate + nprnd.randn() * self.std_rate)
            sleep_rate = sleep_rate if sleep_rate > .1 else .1
            rospy.sleep(1.0 / sleep_rate)


def talker():
    rospy.init_node('tracker_tester', anonymous=True)

    walkers = [
        CircularWalker('marc', start_coords=[5, 5], omega_deg=10),
        LinearWalker('vicky', start_coords=[0, 0]),
        LinearWalker('greg', start_coords=[2, 2])
    ]

    for w in walkers:
        w.start()

    Sensor(
        'marvelmind', walkers[:2],          # not all walkers of this sensor
        anonymous=False,                    # this sensor provides IDs
        noise=[.2, .2],                     # cartesian noise
        delayed_start=0,                    # start only after N seconds
        prob_visible=.9,                    # probability of sensor being
                                            # on when switched
        prob_switch_visible=.2,             # prob of switching (per sec)
        avg_rate=1, std_rate=.1).start()   # avg/std of publish rate
    Sensor(
        'serhan', walkers,
        anonymous=True,
        noise=[.04, .04],
        delayed_start=0,
        avg_rate=20, std_rate=2,
        prob_visible=.2,
        prob_switch_visible=.2).start()
    Sensor(
        'gps', walkers,
        anonymous=False,
        noise=[1, 1],
        delayed_start=0,
        avg_rate=1, std_rate=.2,
        prob_visible=.5,
        prob_switch_visible=.1).start()

    rospy.spin()

    # rate = rospy.Rate(10)
    # angle = 0
    # angle_step = .5
    # scale = 2.0

    # while not rospy.is_shutdown():
    #     angle += angle_step
    #     if angle > 360:
    #         angle = 0
    #     pose_array = PoseArray()
    #     pose_array.header.frame_id = '/map'
    #     pose_array.header.stamp = rospy.Time.now()

    #     current_pose = Pose()
    #     current_pose.position.x = cos(angle * pi / 180.0) * scale
    #     current_pose.position.y = sin(angle * pi / 180.0) * scale
    #     current_pose.orientation.w = 1.0
    #     pose_array.poses.append(current_pose)

    #     people = People()
    #     people.header.frame_id = '/base_link'
    #     people.header.stamp = rospy.Time.now()

    #     person = Person()
    #     person.position = current_pose.position
    #     person.name = 'hurga'
    #     person.reliability = 1.0
    #     people.people.append(person)

    #     poses_pub.publish(pose_array)
    #     people_pub.publish(people)
    #     rate.sleep()

if __name__ == '__main__':
    try:
        talker()
    except rospy.ROSInterruptException:
        pass
