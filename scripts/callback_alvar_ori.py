#!/usr/bin/env python
import collections
import rospy
from ar_track_alvar_msgs.msg import AlvarMarker
from tf.transformations import euler_from_quaternion, quaternion_from_euler
from math import degrees

global roll, pitch, yaw


def callback_alvmarker(AlvarMsg):
    markers = {}
    markers[0] = {'x': 0.0, 'y': 0.0}
    markers[22] = {'x': 0.0, 'y': 0.0}

    for m in AlvarMsg.markers:
        marker_id = m.id
        if marker_id <= 30:
            marker_pose = m.pose.pose
            pos = marker_pose.position
            ori = marker_pose.orientation
            ori_list = [ori.y, ori.z, ori.x, ori.w]

            #transform orientation to euler
            (roll, pitch, yaw) = euler_from_quaternion(ori_list)
            markers[marker_id] = {'x': pos.y, 'y': pos.z, 'roll': roll, 'pitch': pitch, 'yaw': yaw}
            markers = collections.OrderedDict(sorted(markers.items())) #sort dict by markers no.

    (drone_x, drone_y) = markers[0]['x'], markers[0]['y']
    (goal_x, goal_y) = markers[22]['x'], markers[22]['y']


    return drone_x, drone_y, goal_x, goal_y

