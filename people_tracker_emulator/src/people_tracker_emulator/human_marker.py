# -*- coding: utf-8 -*-

import rospy
from geometry_msgs.msg import Vector3, Point, Pose
from std_msgs.msg import ColorRGBA
import math
from tf.transformations import euler_from_quaternion
from visualization_msgs.msg import Marker
from copy import deepcopy


_marker_seq = 0

def generate_extremity_position(centre, dx, dy, z):
    angle = euler_from_quaternion(
        [
            centre.orientation.x,
            centre.orientation.y,
            centre.orientation.z,
            centre.orientation.w
        ]
    )[2] + math.pi/2
    centre.position.z = z
    return Pose(position=generate_position(centre.position, angle, dx, dy))

def generate_position(centre, angle, dx, dy):
  s = math.sin(angle)
  c = math.cos(angle)

  # rotate point
  res = Point()
  res.x = dx * c - dy * s
  res.y = dx * s + dy * c

  # translate point back:
  res.x += centre.x
  res.y += centre.y
  res.z  = centre.z
  return res

def createMarker(m_id, m_type, action, pose, scale, color, target_frame):
    marker = Marker()
    marker.header.frame_id = target_frame
    marker.header.stamp = rospy.Time.now()
    marker.header.seq = _marker_seq
    global _marker_seq
    _marker_seq += 1
    marker.ns = "people_tracker"
    marker.id = m_id
    marker.type = m_type
    marker.action = action
    marker.pose = pose
    marker.scale = scale
    marker.color = color
    marker.lifetime = rospy.Duration(1)
    return marker


def createHead(m_id, action, pose, target_frame):
    pose = deepcopy(pose)
    scale = Vector3()
    scale.x = 0.3
    scale.y = 0.3
    scale.z = 0.3
    color = ColorRGBA()
    color.a = 1.0
    color.r = 233.0/255.0
    color.g = 150.0/255.0
    color.b = 122.0/255.0
    pose.position.z = 1.6
    return createMarker(m_id, Marker.SPHERE, action, pose, scale, color, target_frame)


def createBody(m_id, action, pose, target_frame, color=None):
    pose = deepcopy(pose)
    scale = Vector3()
    scale.x = 0.35
    scale.y = 0.35
    scale.z = 0.7
    if color == None:
        color = ColorRGBA()
        color.a = 1.0
        color.r = 139.0/255.0
        color.g = 0.0/255.0
        color.b = 0.0/255.0
    pose.position.z = 1.1
    return createMarker(m_id, Marker.CYLINDER, action, pose, scale, color, target_frame)

def createLegs(m_idl, m_idr, action, pose, target_frame):
    pose = deepcopy(pose)
    legs = []
    scale = Vector3()
    scale.x = 0.15
    scale.y = 0.2
    scale.z = 0.8
    color = ColorRGBA()
    color.a = 1.0
    color.r = 0.0/255.0
    color.g = 0.0/255.0
    color.b = 139.0/255.0
    legs.append(
        createMarker(
            m_idl,
            Marker.CYLINDER,
            action,
            generate_extremity_position(pose, 0.1, 0.0, 0.4),
            scale,
            color,
            target_frame
        )
    )
    legs.append(
        createMarker(
            m_idr,
            Marker.CYLINDER,
            action,
            generate_extremity_position(pose, -0.1, 0.0, 0.4),
            scale,
            color,
            target_frame
        )
    )
    return legs

def createArms(m_idl, m_idr, action, pose, target_frame, color=None):
    pose = deepcopy(pose)
    arms = []
    scale = Vector3()
    scale.x = 0.1
    scale.y = 0.1
    scale.z = 0.7
    if color == None:
        color = ColorRGBA()
        color.a = 1.0
        color.r = 139.0/255.0
        color.g = 0.0/255.0
        color.b = 0.0/255.0
    arms.append(
        createMarker(
            m_idl,
            Marker.CYLINDER,
            action,
            generate_extremity_position(pose, 0.2, 0.0, 1.1),
            scale,
            color,
            target_frame
        )
    )
    arms.append(
        createMarker(
            m_idr,
            Marker.CYLINDER,
            action,
            generate_extremity_position(pose, -0.2, 0.0, 1.1),
            scale,
            color,
            target_frame
        )
    )
    return arms;

def createHuman(m_id, pose, target_frame, color=None):
    human = []
    m_id += 1
    human.append(createHead(m_id, Marker.ADD, pose, target_frame))
    m_id += 1
    human.append(createBody(m_id, Marker.ADD, pose, target_frame, color=color))
    legs = createLegs(m_id+1, m_id+2, Marker.ADD, pose, target_frame)
    m_id += 3
    human.extend(legs)
    arms = createArms(m_id+1, m_id+2, Marker.ADD, pose, target_frame, color=color)
    human.extend(arms)
    return human
