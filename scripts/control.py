#!/usr/bin/env python
import rospy
import socket
import threading
import time

'''
import sys
sys.path.insert(0, '/home/robolab/tello_drone/src/tello_drone/scripts')

from original import send, receive

from ar_track_alvar_msgs.msg import AlvarMarker
'''

from callback_alvar import *


#Coordinate matching
K_x = -78.00
K_y = 78.00

Vel = 10

GOAL_DIST_THRESHOLD = 20 #cm


tello1_address = ('192.168.11.40', 8889)
#tello1_address = ('192.168.11.33', 8889)

local1_address = ('', 9010 )

sock1 = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)

sock1.bind(local1_address)


def send(message, delay):
  # Try to send the message otherwise print the exception
  try:
    sock1.sendto(message.encode(), tello1_address)
   # sock2.sendto(message.encode(), tello2_address)
    

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

      print("Received message: from Tello EDU #1: " + response1.decode(encoding='utf-8'))
      #print("Received message: from Tello EDU #2: " + response2.decode(encoding='utf-8'))

    except Exception as e:
      # If there's an error close the socket and break out of the loop
      sock1.close()
     # sock2.close()

      print("Error receiving: " + str(e))
      break


def move_xy(goal_x, goal_y, drone_x, drone_y):
    move_x = int(K_x * (goal_x - drone_x))
    move_y = int(K_y * (goal_y - drone_y))

    if abs(move_x) <= GOAL_DIST_THRESHOLD and abs(move_y) <= GOAL_DIST_THRESHOLD:
        status = 'Goal Position reached'

    else:
        status = ' Goal Position not reached'
        move_x = int(K_x * (goal_x - drone_x))
        move_y = int(K_y * (goal_y - drone_y))
    
    print ("go (x: %3.3f, y: %3.3f)" % (move_x, move_y))
    send("go " + str(move_x) + " " + str(move_y) + " 0 " + str(Vel), 4)
  
    return status






