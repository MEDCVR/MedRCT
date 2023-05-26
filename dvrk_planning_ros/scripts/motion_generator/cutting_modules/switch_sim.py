#!/usr/bin/env python3

import rospy
from sensor_msgs.msg import Joy


def command():
    while not rospy.is_shutdown():
        print("T for teleop")
        print("M for motion generator")
        choice = input()
        temp = 5
        try:
            temp = list(choice)
            if temp[0] == 't' or temp[0] == 'T':
                print("disabling motion generator and switching to teleop.......")
                print (" ")
                temp=0
            elif temp[0] == 'm' or temp[0] == 'M':
                print("disabling teleop and switching to motion generator.......")
                print (" ")
                temp=1
            else: 
                command()
            dat = Joy()
            dat.buttons = [temp]
            ToggleStatePublisher.publish (dat)
        except:
            print ("something went wrong, try again ......")
            command()

            
if __name__ == '__main__':
    
    rospy.init_node('Switch_sim', anonymous= True)
    ToggleStatePublisher = rospy.Publisher('/toggle/mode', Joy, queue_size = 1)
    command()
    try:
        rospy.spin()
    except KeyboardInterrupt:
        print("shutting down")
