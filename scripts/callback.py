#!/usr/bin/env python
# -*- coding: utf-8 -*-
"""

@author: keykeerati
@date: 2021/12/06

"""

import collections
import rospy
from tf.transformations import euler_from_quaternion
from ar_track_alvar_msgs.msg import AlvarMarkers

global roll, pitch, yaw


def get_drone_location(AlvarMsg):

    #initialize drones
    drone = {}

    for m in AlvarMsg.markers:
        marker_id = m.id
        if marker_id <= 5:
            marker_pose = m.pose.pose
            pos = marker_pose.position
            ori = marker_pose.orientation
            ori_list = [ori.y, ori.z, ori.x, ori.w]

            #transform orientation to euler
            (roll, pitch, yaw) = euler_from_quaternion(ori_list)
            drone[marker_id] = {'x': pos.y, 'y': pos.z, 'roll': roll, 'pitch': pitch, 'yaw': yaw}
            markers = collections.OrderedDict(sorted(drone.items())) #sort dict by markers no.

    return drone

def get_goal_location(AlvarMsg):

    markers = {}
    goal_x = []
    goal_y = []
    markers[22] = {'x': 0.0, 'y': 0.0} 
    markers[24] = {'x': 0.0, 'y': 0.0}
    markers[26] = {'x': 0.0, 'y': 0.0}

    for m in AlvarMsg.markers:
        marker_id = m.id
        if marker_id >= 4:
            marker_pose = m.pose.pose
            pos = marker_pose.position

            markers[marker_id] = {'x': pos.y, 'y': pos.z}
            markers = collections.OrderedDict(sorted(markers.items())) #sort dict by markers no.
    
    goal_x  = [markers[22]['x'], markers[24]['x'], markers[26]['x']]
    goal_y  = [markers[22]['y'], markers[24]['y'], markers[26]['y']]

    return goal_x, goal_y

