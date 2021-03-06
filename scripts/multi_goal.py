#! /usr/bin/env python

import rospy
from time import time, sleep
from datetime import datetime
from ar_track_alvar_msgs.msg import AlvarMarkers
from control import *
from callback_alvar import *



if __name__ == '__main__':
    try:
        rospy.init_node('control_node', anonymous= False)
        rate = rospy.Rate(10)
        goal_x1 = [0.012, 0.012, 0.012]
        goal_y1 = [-0.223, 0.036, 0.326]

        drone_1 = ('192.168.11.40', 8889)
        drone_2 = ('192.168.11.33', 8889)

        AlvarMsg = rospy.wait_for_message('/ar_pose_marker', AlvarMarkers)

        sleep(1)
        
        receiveThread = threading.Thread(target=receive)
        receiveThread.daemon = True
        receiveThread.start()
        send("command", 3, drone_1)
        send("takeoff", 8, drone_1)
        sleep(1)
        count = 0
        robot_in_pos = False
        i = 0

        sleep(1)
    
        while not rospy.is_shutdown():
            if count == 40:
                send("land", 5, drone_1)
                print("Mission failed")
                sock1.close()
                rospy.signal_shutdown('End of testing')

            else:
                if not robot_in_pos:
                    AlvarMsg = rospy.wait_for_message('/ar_pose_marker', AlvarMarkers)
                    (drone_x, drone_y, goal_x, goal_y) = callback_alvmarker(AlvarMsg)
                    if drone_x != 0.0 and drone_y != 0.0 and goal_x1[i] != 0.0 and goal_y1[i] != 0.0:
                        #rotate drone to initial angle
                        print('\r\nDrone Position:')
                        print('x = %.2f' % drone_x)
                        print('y = %.2f' % drone_y)
                        print('')
                        sleep(1) 
                        robot_in_pos = True
                    elif drone_x != 0.0 and drone_y != 0.0 and goal_x1[i] == 0.0 and goal_y[i] == 0.0:
                        print("Mission completed successfully!")
                        robot_in_pos = False
                        i += 1
                        if i == 3:
                            robot_in_pos = False
                            send("land", 5, drone_1) 
                            print("Mission completed successfully!")
                            sock1.close()
                            rospy.signal_shutdown('End of testing')
                    else:
                        robot_in_pos = False
                        count += 1
                else:
                    #update the drone's current position
                    (drone_x, drone_y, goal_x, goal_y) = callback_alvmarker(AlvarMsg)
                    #print ("drone x: %3.3f , drone y: %3.3f" % (drone_x, drone_y))
                    status = move_xy(goal_x1[i], goal_y1[i], drone_x, drone_y, drone_1)
                    sleep(1)
                    if status == 'Goal Position reached':
                        print("Mission completed successfully!")
                        robot_in_pos = False
                        i += 1
                        if i == 3: #No. of goals
                            robot_in_pos = False
                            send("land", 5, drone_1) 
                            print("Mission completed successfully!")
                            sock1.close()
                            rospy.signal_shutdown('End of testing')
                    else:
                        count += 1
                        drone_x, drone_y = (0.0, 0.0)
                        goal_x[i], goal_y[i] = (0.0, 0.0)
                        robot_in_pos = False

    except rospy.ROSInterruptException:
        send("land", 5, drone_1)
        sock1.close()
        print('Simulation terminated')
        pass