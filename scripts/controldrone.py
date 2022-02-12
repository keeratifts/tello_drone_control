#!/usr/bin/env python
# -*- coding: utf-8 -*-
"""

@author: keykeerati
@date: 2021/12/06

"""

import rospy
import socket
import threading
import time
import numpy as np
from math import sqrt, pow
import csv


#Coordinate matching
K_x = -93.00
K_y = 100.00

Vel = 10

GOAL_DIST_THRESHOLD = 20 #cm

local1_address = ('', 9010 )

sock1 = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)

sock1.bind(local1_address)



def send(message, delay, drone):
  # Try to send the message otherwise print the exception
  try:
    sock1.sendto(message.encode(), drone)
    

    print("Sending message: " + message)
  except Exception as e:
    print("Error sending: " + str(e))

  # Delay for a user-defined period of time
  time.sleep(delay)

def receive():
  # Continuously loop and listen for incoming messages
  while True:
    # Try to receive the message otherwise print the exception
    try:
      response1, ip_address = sock1.recvfrom(128)
      #response2, ip_address = sock2.recvfrom(128)

      print("Received message: from Tello EDU: " + response1.decode(encoding='utf-8'))
      #print("Received message: from Tello EDU #2: " + response2.decode(encoding='utf-8'))

    except Exception as e:
      # If there's an error close the socket and break out of the loop
      sock1.close()
     # sock2.close()

      print("Error receiving: " + str(e))
      break


def move_xy(goal_x, goal_y, drone_x, drone_y, drone):
    move_x = int(K_x * (goal_x - drone_x))
    move_y = int(K_y * (goal_y - drone_y))

    if abs(move_x) <= GOAL_DIST_THRESHOLD and abs(move_y) <= GOAL_DIST_THRESHOLD:
        
        if move_x > 10:
          send("forward 20 ", 1, drone)
          if move_y > 10:
            send("right 20 ", 1, drone)
          elif move_y < -10:
            send("left 20 ", 1, drone)
        elif move_x < -10:
          send("back 20 ", 1, drone)
          if move_y > 10:
            send("right 20 ", 1, drone)
          elif move_y < -10:
            send("left 20 ", 1, drone)
        
        
        status = 'Goal Position reached'

    else:
        status = 'Goal Position not reached'
        move_x = int(K_x * (goal_x - drone_x))
        move_y = int(K_y * (goal_y - drone_y))
        print ("%s go (x: %3.3f, y: %3.3f)" % (drone, move_x, move_y))
        send("go " + str(move_x) + " " + str(move_y) + " 0 " + str(Vel), 0, drone)
  
    return status

def set_goal_dict(log_file):
    dict = {}
    for i in range(len(log_file)):
        goal, x, y = ([], [], [])
        for row in csv.reader(log_file[i], delimiter = ','):
            goal.append(row)
        for j in range(len(goal)):
            x.append(round(float(goal[j][0]), 4))
            y.append(round(float(goal[j][1]), 4))
        dict[i] = {'x': x, 'y': y}
    
    return dict

def choose_goal(goal, robot):
    all_d = np.empty((0, len(robot)), float)
    for i in range(len(goal)):
        distance = []
        for j in range(100):
            if j in robot:
                distance.append(sqrt(pow((goal[i]['x'][0] - robot[j]['x']), 2) + pow((goal[i]['y'][0] - robot[j]['y']), 2)))
        all_d = np.append(all_d, [distance], axis = 0)

    while not np.isnan(all_d).all():
        result = np.where(all_d == np.nanmin(all_d))
        listofCordinates = list(zip(result[0], result[1]))
        #print (listofCordinates)
        for cord in listofCordinates:
            k = {'goal': goal[cord[0]]}
            robot[cord[1]].update(k)
            all_d[cord[0]] = np.NaN
            all_d[:,cord[1]] = np.NaN

    return robot

'''
def move(goal, pose, drone):
  DATA_PATH = '/home/robolab/tello_drone/src/tello_drone_control/Data/'
  goal = pd.read_csv(DATA_PATH + goal + '.csv')
  
  for x, y in zip(goal.x, goal.y):
    move_x = int(K_x * (x - pose['x']))
    move_y = int(K_y * (y - pose['y']))

    while abs(move_x) >= GOAL_DIST_THRESHOLD and abs(move_y) >= GOAL_DIST_THRESHOLD:
      status = 'Goal Position not reached'
      send("go " + str(move_x) + " " + str(move_y) + " 0 " + str(Vel), 1, drone)
      move_x = int(K_x * (x - pose['x']))
      move_y = int(K_y * (y - pose['y']))

    else:
      status = 'Goal reached'
  
  status = 'Goal Position reached'

  return status
'''
  










