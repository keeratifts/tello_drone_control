#!/usr/bin/env python
import rospy
import socket
import threading
import time
import pandas as pd

'''
import sys
sys.path.insert(0, '/home/robolab/tello_drone/src/tello_drone/scripts')

from original import send, receive

from ar_track_alvar_msgs.msg import AlvarMarker
'''

from callback_alvar import *


#Coordinate matching
K_x = -93.00
K_y = 100.00

Vel = 10

GOAL_DIST_THRESHOLD = 20 #cm

tello1_address = ('192.168.11.33', 8889)
tello2_address = ('192.168.11.40', 8889)
tello3_address = ('192.168.11.23', 8889)


local1_address = ('', 9010 )
local2_address = ('', 9011 )
local3_address = ('', 9012 )


sock1 = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)
sock2 = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)
sock3 = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)


sock1.bind(local1_address)
sock2.bind(local2_address)
sock3.bind(local3_address)




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
      response2, ip_address = sock2.recvfrom(128)
      response3, ip_address = sock2.recvfrom(128)

      

      print("Received message: from Tello EDU #1: " + response1.decode(encoding='utf-8'))
      print("Received message: from Tello EDU #2: " + response2.decode(encoding='utf-8'))
      print("Received message: from Tello EDU #3: " + response3.decode(encoding='utf-8'))

    except Exception as e:
      # If there's an error close the socket and break out of the loop
      sock1.close()
      sock2.close()
      sock3.close()

      print("Error receiving: " + str(e))
      break


def move_xy(goal_x, goal_y, drone_x, drone_y, drone):
    move_x = int(K_x * (goal_x - drone_x))
    move_y = int(K_y * (goal_y - drone_y))

    if abs(move_x) <= GOAL_DIST_THRESHOLD and abs(move_y) <= GOAL_DIST_THRESHOLD:
        status = 'Goal Position reached'

    else:
        status = 'Goal Position not reached'
        move_x = int(K_x * (goal_x - drone_x))
        move_y = int(K_y * (goal_y - drone_y))
    
    print ("go (x: %3.3f, y: %3.3f)" % (move_x, move_y))
    send("go " + str(move_x) + " " + str(move_y) + " 0 " + str(Vel), 1, drone)
  
    return status

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
  










