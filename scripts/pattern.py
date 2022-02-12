#! /usr/bin/env python

import rospy
from time import time, sleep
from datetime import datetime
from ar_track_alvar_msgs.msg import AlvarMarkers
from control import *
from callback import *



if __name__ == '__main__':
    try:
        rospy.init_node('control_node', anonymous= False)
        rate = rospy.Rate(10)

        AlvarMsg = rospy.wait_for_message('/ar_pose_marker', AlvarMarkers)

        sleep(1)
        
        receiveThread = threading.Thread(target=receive)
        receiveThread.daemon = True
        receiveThread.start()

        send("command", 3) 
        send("takeoff", 8)
        sleep(1)

        count = 0
        robot_in_pos = False
        i = 0

        sleep(1)

        while not rospy.is_shutdown():
            if count == 30:
                send("land", 5)
                print("Mission failed")
                sock1.close()
                rospy.signal_shutdown('End of testing')
            else:
                if not robot_in_pos:
                    AlvarMsg = rospy.wait_for_message('/ar_pose_marker', AlvarMarkers)
                    drone = get_drone_location(AlvarMsg)
                    if drone[0]['x'] != 0.0 and drone[1]['x'] != 0.0 and drone[2]['x'] != 0 and drone[3]['x'] != 0:
                        #rotate drone to initial angle
                        print('\r\nDrone Position:')
                        print('Drone_M[0] = (x: %.2f, y: %.2f)' % (drone[0]['x'], drone[0]['y']))
                        print('Drone_M[1] = (x: %.2f, y: %.2f)' % (drone[1]['x'], drone[1]['y']))
                        print('Drone_M[2] = (x: %.2f, y: %.2f)' % (drone[2]['x'], drone[2]['y']))
                        print('Drone_M[3] = (x: %.2f, y: %.2f)' % (drone[3]['x'], drone[3]['y']))
                        print('')
                        sleep(1) 
                        robot_in_pos = True
                    else:
                        robot_in_pos = False
                        count += 1
                else:
                    #update the drone's current position
                    AlvarMsg = rospy.wait_for_message('/ar_pose_marker', AlvarMarkers)
                    drone = get_drone_location(AlvarMsg)
                    print ("drone x: %3.3f , drone y: %3.3f" % (drone_x, drone_y))
                    status = move_xy(goal_x[i], goal_y[i], drone_x, drone_y)
                    sleep(1)
                    if status == 'Goal Position reached':
                        print("Mission completed successfully!")
                        robot_in_pos = False
                        i += 1
                        if i == 3: #No. of goals
                            robot_in_pos = False
                            send("land", 5) 
                            print("Mission completed successfully!")
                            sock1.close()
                            rospy.signal_shutdown('End of testing')
                    else:
                        count += 1
                        drone_x, drone_y = (0.0, 0.0)
                        goal_x[i], goal_y[i] = (0.0, 0.0)
                        robot_in_pos = False

    except rospy.ROSInterruptException:
        send("land", 5)
        sock1.close()
        print('Simulation terminated')
        pass