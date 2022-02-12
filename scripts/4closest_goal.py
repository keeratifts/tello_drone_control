#! /usr/bin/env python3

import rospy
from time import time, sleep
from datetime import datetime
from ar_track_alvar_msgs.msg import AlvarMarkers
from control import *
from callback import get_drone_location

N = 4

(goalx_d, goaly_d, d) = (dict(), dict(), dict())
goal = {}
goal[0] = {'x': [0.59, 0.07], 'y': [-0.016, 0.477]}
goal[1] = {'x': [0.30, -0.407], 'y': [-0.016,0.027]}
goal[2] = {'x': [0.00, 0.012], 'y': [-0.016, 0.003]}
goal[3] = {'x': [-0.59, 0.012], 'y': [-0.016, -0.41]}


if __name__ == '__main__':
    try:
        rospy.init_node('control_node', anonymous= False)
        rate = rospy.Rate(10)

        d[0] = ('192.168.11.39', 8889)
        d[1] = ('192.168.11.49', 8889)
        d[2] = ('192.168.11.41', 8889)
        d[3] = ('192.168.11.31', 8889)


        AlvarMsg = rospy.wait_for_message('/ar_pose_marker', AlvarMarkers)

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
        send("up 30", 0, d[0])
        sleep(3)
    

        robot_in_pos = False

        j = 0
        count = 0
        
        while not rospy.is_shutdown():
            if count == 200:
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
                        drone = choose_goal(goal, drone)
                        print('\r\nDrone Position:')
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
                else:
                    if j >= len(goalx_d[0]):
                        if all(value=='Goal Psition reached' for value in status.values()):
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
                            sleep(2)
                            if all(value == 'Goal Position reached' for value in status.values()):
                                print(" ")
                                print ("Mission completed successfully!")
                                print("")
                                j += 1
                                sleep(3)
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

