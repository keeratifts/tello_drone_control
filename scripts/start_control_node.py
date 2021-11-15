#! /usr/bin/env python

import rospy
from time import time, sleep
from datetime import datetime
from ar_track_alvar_msgs.msg import AlvarMarkers
from control import *
from callback_alvar_ori import *



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

        sleep(1)

        while not rospy.is_shutdown():
            if count == 20:
                send("land", 5)
                print("Mission failed")
                sock1.close()
                rospy.signal_shutdown('End of testing')

            else:
                if not robot_in_pos:
                    AlvarMsg = rospy.wait_for_message('/ar_pose_marker', AlvarMarkers)
                    (drone_x, drone_y, goal_x, goal_y) = callback_alvmarker(AlvarMsg)
                    if drone_x != 0.0 and drone_y != 0.0 and goal_x != 0.0 and goal_y != 0.0:
                        #rotate drone to initial angle
                        print('\r\nInitial position:')
                        print('x = %.2f' % drone_x)
                        print('y = %.2f' % drone_y)
                        print('')
                        sleep(1)
                        robot_in_pos = True
                    elif drone_x != 0.0 and drone_y != 0.0 and goal_x == 0.0 and goal_y == 0.0:
                        robot_in_pos = False
                        send("land", 5) 
                        print("Mission completed successfully!")
                        sock1.close()
                        rospy.signal_shutdown('End of testing')
                    else:
                        robot_in_pos = False
                        count += 1
                else:
                    #update the drone's current position
                    (drone_x, drone_y, goal_x, goal_y) = callback_alvmarker(AlvarMsg)
                    print ("drone x: %3.3f , drone y: %3.3f" % (drone_x, drone_y))
                    status = move_xy(goal_x, goal_y, drone_x, drone_y)
                    sleep(1)
                    if status == 'Goal Position reached':
                        send("land", 5) 
                        print("Mission completed successfully!")
                        sock1.close()
                        rospy.signal_shutdown('End of testing')
                    else:
                        count += 1
                        drone_x, drone_y = (0.0, 0.0)
                        goal_x, goal_y = (0.0, 0.0)
                        robot_in_pos = False

    except rospy.ROSInterruptException:
        send("land", 5)
        sock1.close()
        print('Simulation terminated')
        pass