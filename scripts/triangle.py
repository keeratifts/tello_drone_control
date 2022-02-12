#! /usr/bin/env python

import rospy
from time import time, sleep
from datetime import datetime
from ar_track_alvar_msgs.msg import AlvarMarkers
from controlmulti import *
from callback_alvar import *



if __name__ == '__main__':
    # try:
        rospy.init_node('control_node', anonymous= False)
        rate = rospy.Rate(10)

        drone_1 = ('192.168.11.40', 8889)
        drone_2 = ('192.168.11.33', 8889)
        drone_3 = ('192.168.11.23', 8889)

        drone = [drone_1, drone_2, drone_3]
        #goal key for calling markers' goal
        goal = ["front22","left26", "right24"]


        # drone_4 = ('192.168.11.57', 8889)


        AlvarMsg = rospy.wait_for_message('/ar_pose_marker', AlvarMarkers)

        sleep(1)
        
        receiveThread = threading.Thread(target=receive)
        receiveThread.daemon = True
        receiveThread.start()
        #enter sdk mode
        send("command", 0, drone_1)
        send("command", 0, drone_2)
        send("command", 0, drone_3)
        sleep(3)



        #takeoff
        send("takeoff", 0, drone_1)
        send("takeoff", 0, drone_2)
        send("takeoff", 0, drone_3)
        sleep(8)

        #form a line to a non AR (manual)
        #use marker first to get the position 
        send("go -100 -20 60 10", 0, drone_1)
        send("go 70 20 100 10 ", 0, drone_2)
        send("go 70 30 60 10", 0, drone_3)
        sleep(1)


    
def feedbackcontrol(drone,goal):
        count = 0
        robot_in_pos = False
        i = 0

        while not rospy.is_shutdown():
            for N in drone:
                if count == 40:
                    send("land", 0, drone[N])
                    # send("land", 0, drone_2)
                    # send("land", 0, drone_3)

                    print("Mission failed")
                    sock1.close()
                    rospy.signal_shutdown('End of testing')

                else:
                    if not robot_in_pos:
                        AlvarMsg = rospy.wait_for_message('/ar_pose_marker', AlvarMarkers)
                        (drone_x, drone_y, goal_x, goal_y) = callback_alvmarker(AlvarMsg)
                        if drone_x != 0.0 and drone_y != 0.0 and goal_x[i] != 0.0 and goal_y[i] != 0.0:
                            #rotate drone to initial angle
                            print('\r\nDrone Position:')
                            print('x = %.2f' % drone_x)
                            print('y = %.2f' % drone_y)
                            print('')
                            sleep(1) 
                            robot_in_pos = True
                        elif drone_x != 0.0 and drone_y != 0.0 and goal_x[i] == 0.0 and goal_y[i] == 0.0:
                            print("Mission completed successfully!")
                            robot_in_pos = False
                            i += 1
                            if i == 1:
                                robot_in_pos = False
                                send("land", 0, drone[N])
                                # send("land", 0, drone_2)
                                # send("land", 0, drone_3)   
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
                        for j in drone:
                            status = move_xy(goal_x[i], goal_y[i], drone_x, drone_y, drone[j])
                            # sleep(1)
                            # status = move_xy(goal_x[i], goal_y[i], drone_x, drone_y, drone_2)
                            sleep(1)
                            if status == 'Goal Position reached':
                                print("Mission completed successfully!")
                                robot_in_pos = False
                                i += 1
                                if i == 1: #number of goals
                                    robot_in_pos = False
                                    send("land", 0, drone)
                                    # send("land", 0, drone_2)  
                                    print("Mission completed successfully!")
                                    sock1.close()
                                    rospy.signal_shutdown('End of testing')
                            else:
                                count += 1
                                drone_x, drone_y = (0.0, 0.0)
                                goal_x[i], goal_y[i] = (0.0, 0.0)
                                robot_in_pos = False

feedbackcontrol(drone[0], "front22")
feedbackcontrol(drone[1], "left26")
feedbackcontrol(drone[2], "right24")

    # except rospy.ROSInterruptException:
    #     send("land", 0, drone_1)
    #     send("land", 0, drone_2)
    #     sock1.close()
    #     print('Simulation terminated')
    #     pass