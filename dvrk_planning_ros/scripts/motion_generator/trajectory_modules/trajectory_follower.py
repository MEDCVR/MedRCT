#!/usr/bin/env python3

from sensor_msgs.msg import JointState
import time
import numpy as np
import rospy

import math
from dvrk_planning.kinematics.psm import PsmKinematicsSolver, LND400006, RTS470007

###############################################
#config
###############################################
mode = ""

max_open = 40
max_close = 5
distance_close_open = 0.0015
point_ratio  = 20
###############################################


jaw_trajectory = []

def jaw_cutting(traj, jaw_position):
    global jaw_trajectory, max_open, max_close, distance_close_open, point_ratio
    print ("trajectory", len(traj))
    jaw_trajectory = []

    max_open = max_open * (3.14/180)
    max_close = max_close * (3.14/180)
    
    
    
    inc_points = int((len(traj)-1)/point_ratio)
    #print (inc_points)

    points = inc_points
    points = points + 1
    p = PsmKinematicsSolver(RTS470007())

    mode = 0
    curr = jaw_position
    l=0
    while (l+inc_points) < (len(traj)):
        start = traj[l]
        l= l+ inc_points
        #print(l)
        end = traj[l]

        # print(start)
        # print(end)
        start = p.compute_fk(start)
        end = p.compute_fk(end)

        # print(start)
        # print(end)
        start = start.getA()
    
        end = end .getA()

        distance = math.sqrt( math.pow((start[0][3]-end[0][3]),2) +math.pow((start[1][3]-end[1][3]),2) + math.pow((start[2][3]-end[2][3]),2) )
        increment_points = points / (distance/ distance_close_open)
        #print (increment_points)
        increment = (max_open - max_close )/increment_points
        # print (distance)
        # print (distance_close_open)
        # print (points)
        # print (increment_points)
        # print (increment)

        
        # print (curr)
        for i in range (0,points):
            if curr >= max_open:
                mode = 1
            elif curr <=max_close:
                mode = 0

            if mode == 0:
                jaw_trajectory.append(curr + increment)
                curr = curr + increment
            elif mode ==1 :
                jaw_trajectory.append(curr - increment)
                curr = curr - increment
    #print (jaw_trajectory)
    print ("jaw trajectory", len(jaw_trajectory))


follower_interrupt_flag = [1]
class Follower:
    
    def set_follower_status(self,msg):
        global follower_interrupt_flag
        follower_interrupt_flag = msg

    def follow_trajectory(self, traj, JointStatePublisher, durations, interpolated_points, name, JawStatePublisher=[], jaw_position=[]):
        global mode
        trajectory = traj.copy()
        mode = name
        print (mode)
        print("Following the trajectory ........")
        # print (durations)
        # print (interpolated_points)
        rate = durations[0]/interpolated_points[0]
        durations_index = 1
        points_index = 0
        curr = 0
        #print (rate)
        # print(len(trajectory))
        # print(trajectory)
        # print(len(durations))
        # print(len(interpolated_points))
        # print (interpolated_points[0])
        
        if mode == "scissor":
            # print ("index?")
            # print(0+ interpolated_points[0]-1)
            # print (interpolated_points[points_index])
            jaw_cutting(trajectory[0:interpolated_points[points_index]-1],jaw_position)
        for i in range(0,len(trajectory)):
            if rospy.is_shutdown():
                return
            if follower_interrupt_flag[0] == 0:
                print("Interupting follwer")
                return
            dat = JointState()
            dat.position = trajectory[i]
            #print(dat)
            JointStatePublisher.publish(dat)

            if mode == "scissor":
                dat = JointState()
                dat.position = [jaw_trajectory[i]]
                JawStatePublisher.publish(dat)

                    

            if ((i-curr) >= interpolated_points[points_index]):
                points_index = points_index +1
                rate = durations[durations_index]/interpolated_points[points_index]
                curr = i
                durations_index = durations_index + 1
                
                # print (durations[durations_index])
                # print (interpolated_points)
                # print(rate)
                #print (rate)
                if mode == "scissor":
                    jaw_cutting(trajectory[0:interpolated_points[points_index]-1],jaw_position)

            time.sleep(rate)
        print("done")

    def follow_jaw_trajectory(self, trajectory, JointStatePublisher, duration, interpolated_points):
        global follower_interrupt_flag
        # print (durations)
        # print (interpolated_points)
        if len(trajectory)>0:
            print("Following the jaw trajectory ........")
            rate = duration/interpolated_points
            #print (trajectory)
            #print (rate)
            
            for i in range(0,len(trajectory)):
                if rospy.is_shutdown():
                    return
                if follower_interrupt_flag[0] == 0:
                    return
                dat = JointState()
                dat.position = [trajectory[i]]
                JointStatePublisher.publish(dat)
                time.sleep(rate)
            print("done")
        else:
            print ("Already here....")

   