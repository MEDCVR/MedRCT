#!/usr/bin/env python3

from sensor_msgs.msg import JointState
import time
import numpy as np

curr_pos = [0.0,0.0,0.0,0.0,0.0,0.0]
update_flag = 0
class Follower:
    
    def follow_trajectory(self, trajectory, JointStatePublisher, durations, interpolated_points):
        print("Following the trajectory ........")
        # print (durations)
        # print (interpolated_points)
        rate = durations[0]/interpolated_points[0]
        durations_index = 1
        points_index = 0
        curr = 0
        #print (rate)
        
        for i in range(0,len(trajectory)):
            dat = JointState()
            dat.position = trajectory[i]
            #print(dat)
            JointStatePublisher.publish(dat)
            if ((i-curr) >= interpolated_points[points_index]):
                points_index = points_index +1
                rate = durations[durations_index]/interpolated_points[points_index]
                curr = i
                durations_index = durations_index + 1
                
                # print (durations[durations_index])
                # print (interpolated_points)
                # print(rate)
                #print (rate)
            time.sleep(rate)
        print("done")

   