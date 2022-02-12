#! /usr/bin/env python

import rospy
from time import time, sleep
from datetime import datetime
from ar_track_alvar_msgs.msg import AlvarMarkers
from control import *
from callback import get_drone_location

N = 3
(goalx_d, goaly_d, d) = (dict(), dict(), dict())

if __name__ == '__main__':
    try:
        rospy.init_node('control_node', anonymous= False)
        rate = rospy.Rate(10)
        
        goalx_d[0] = [0.59, 0.07]
        goaly_d[0] = [-0.016, 0.477]
        goalx_d[1] = [0.00, -0.407]
        goaly_d[1] = [-0.016, 0.003]
        goalx_d[2] = [-0.59, 0.012]
        goaly_d[2] = [-0.016, -0.41]
        
        # goalx_d1 = [0.590, 0.070]
        # goaly_d1 = [-0.016, 0.477]

        # goalx_d2 = [0.00, -0.407]
        # goaly_d2 = [-0.016, 0.003]

        # goalx_d3 = [-0.590, 0.012]
        # goaly_d3 = [-0.016, -0.410]
        
        d[0] = ('192.168.11.40', 8889)
        d[1] = ('192.168.11.23', 8889)
        d[2] = ('192.168.11.33', 8889)

        AlvarMsg = rospy.wait_for_message('/ar_pose_marker', AlvarMarkers)

        sleep(1)
        receiveThread = threading.Thread(target=receive)
        receiveThread.daemon = True
        receiveThread.start()

        send("command", 0, drone_1)
        send("command", 0, drone_2)
        send("command", 3, drone_3)
        send("takeoff", 0, drone_1)
        send("takeoff", 0, drone_2)
        send("takeoff", 8, drone_3)

        AlvarMsg = rospy.wait_for_message('/ar_pose_marker', AlvarMarkers)

        send("down 20", 0, drone_2)
        send("up 20", 4, drone_3)

        robot_in_pos = False

        i = 0
        count = 0
        
        while not rospy.is_shutdown():
            if count == 40:
                send("land", 0, drone_1)
                send("land", 0, drone_2)
                send("land", 5, drone_3)
                sock1.close()
                rospy.signal_shutdown('End of testing')
                pass
            else:
                if not robot_in_pos:
                    AlvarMsg = rospy.wait_for_message('/ar_pose_marker', AlvarMarkers)
                    drone = get_drone_location(AlvarMsg)

                    if drone[5] != {'x': 0.0, 'y': 0.0} and drone[4] != {'x': 0.0, 'y': 0.0} and drone[2] != {'x': 0.0, 'y': 0.0}:
                        print('\r\nDrone Position:')
                        print('drone_1 (x, y) = (%.2f, %.2f)' % (drone[5]['x'], drone[5]['y']))
                        print('drone_2 (x, y) = (%.2f, %.2f)' % (drone[4]['x'], drone[4]['y']))
                        print('drone_3 (x, y) = (%.2f, %.2f)' % (drone[2]['x'], drone[2]['y']))
                        print('')
                        sleep(1)
                        robot_in_pos = True
                    else:
                        robot_in_pos = False
                        count += 1
                        print (count)
                else:
                    drone = get_drone_location(AlvarMsg)
                    d1_status = move_xy(goalx_d1[i], goaly_d1[i], drone[5]['x'], drone[5]['y'], drone_1)
                    d2_status = move_xy(goalx_d2[i], goaly_d2[i], drone[4]['x'], drone[4]['y'], drone_2)
                    d3_status = move_xy(goalx_d3[i], goaly_d3[i], drone[2]['x'], drone[2]['y'], drone_3)
                    sleep(2)
                    if d1_status == 'Goal Position reached' and d2_status == 'Goal Position reached' and d3_status == 'Goal Position reached':
                        print(" ")
                        print("Mission completed successfully!")
                        print(" ")
                        robot_in_pos = False

                        # send("land", 0, drone_1)
                        # send("land", 0, drone_2)
                        # send("land", 5, drone_3)
                        # print("Mission completed successfully!")
                        # sock1.close()
                        # rospy.signal_shutdown('End of testing')

                        
                        i += 1

                        if i == 2:
                            robot_in_pos = False
                            send("land", 0, drone_1)
                            send("land", 0, drone_2)
                            send("land", 5, drone_3)
                            print("Mission completed successfully!")
                            sock1.close()
                            rospy.signal_shutdown('End of testing')
                    else:
                        drone[5] = {'x': 0.0, 'y': 0.0}
                        drone[4] = {'x': 0.0, 'y': 0.0}
                        drone[2] = {'x': 0.0, 'y': 0.0}
                        robot_in_pos = False
                        sleep(3)

    except rospy.ROSInterruptException:
        send("land", 0, drone_1)
        send("land", 0, drone_2)
        send("land", 5, drone_3)
        sock1.close()
        print('Simulation terminated')
        pass

