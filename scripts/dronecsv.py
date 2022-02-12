#! /usr/bin/env python3
# -*- coding: utf-8 -*-
"""

@author: keykeerati
@date: 2021/12/06

"""

import rospy
from time import time, sleep
from datetime import datetime
from ar_track_alvar_msgs.msg import AlvarMarkers
from controldrone import *
from callback import get_drone_location

N = 4

goal = dict()
for i in range(0, N):
    goal[i] = open('/home/robolab/Desktop/kk/Coverage control/Trajectory_Log/goal_'+str(i+1)+'.csv','r')

if __name__ == '__main__':
    try:
        rospy.init_node('control_node', anonymous= False)
        rate = rospy.Rate(10)
        d = dict()
        d[0] = ('192.168.11.39', 8889)
        d[1] = ('192.168.11.49', 8889)
        d[2] = ('192.168.11.41', 8889)
        d[3] = ('192.168.11.31', 8889)
        goal = set_goal_dict(goal)

        sleep(1)
        receiveThread = threading.Thread(target=receive)
        receiveThread.daemon = True
        receiveThread.start()

        for i in range(0, N):
            send("command", 0, d[i])
        sleep(3)

        for i in range(0, N):
            send("takeoff", 0, d[i])
        sleep(8)

        AlvarMsg = rospy.wait_for_message('/ar_pose_marker', AlvarMarkers)

        send("down 20", 0, d[1])
        send("up 30", 0, d[2])
        sleep(3)

        robot_in_pos = False

        j = 0
        count = 0
        
        while not rospy.is_shutdown():
            if count == 100: 
                for i in range(0, N):
                    send("land", 0, d[i])
                sleep(5)
                sock1.close()
                rospy.signal_shutdown('End of testing')
                pass
            else:
                status = dict()
                AlvarMsg = rospy.wait_for_message('/ar_pose_marker', AlvarMarkers)
                drone = get_drone_location(AlvarMsg)

                if not robot_in_pos:
                    AlvarMsg = rospy.wait_for_message('/ar_pose_marker', AlvarMarkers)
                    drone = get_drone_location(AlvarMsg)

                    if len(drone) == N:

                        (goalx_d, goaly_d) = (dict(), dict())
                        drone = choose_goal(goal, drone)
                        print('\r\nInitiail drone Position:')
                        for i in range(0, N):
                            print('drone_'+str(i+1)+' (x, y) = (%.2f, %.2f)' % (drone[i]['x'], drone[i]['y']))
                            goalx_d[i] = drone[i]['goal']['x']
                            goaly_d[i] = drone[i]['goal']['y']
                        print('')

                        sleep(1)
                        robot_in_pos = True

                    else:

                        robot_in_pos = False
                        count += 1
                        print (count)
                        print (len(drone))

                else:
                    if j >= len(goalx_d[1]):
                        if all(value=='Goal Position reached' for value in status.values()):
                            robot_in_pos = False
                            for i in range(0, N):
                                send("land", 0, d[i])
                            sleep(5)
                            print("Mission completed successfully!")
                            sock1.close()
                            rospy.signal_shutdown('End of testing')
                            pass
                        else:
                            if len(drone) == N:
                                for i in range(0, N):
                                    status[i] = move_xy(goalx_d[i][len(goalx_d[i]-1)], goaly_d[i][len(goaly_d[i]-1)], drone[i]['x'], drone[i]['y'], d[i])
                                sleep(2)
                    else:
                        if len(drone) == N:
                            status = dict()
                            for i in range(0, N):
                                status[i] = move_xy(goalx_d[i][j], goaly_d[i][j], drone[i]['x'], drone[i]['y'], d[i])
                                if j == 0:
                                    if all(value=='Goal Position reached' for value in status.values()):
                                        j += 10
                                    else:
                                        j = 0
                                else:
                                    j += 10
                        else:
                            count += 1
                            print (count)

    except rospy.ROSInterruptException:
        for i in range(0, N):
            send("land", 0, d[i])
        sleep(5)
        sock1.close()
        print('Simulation terminated')
        pass
