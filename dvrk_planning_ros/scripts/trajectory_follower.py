#!/usr/bin/env python3

from sensor_msgs.msg import JointState
import time
import numpy as np

curr_pos = [0.0,0.0,0.0,0.0,0.0,0.0]
update_flag = 0
class Follower:
    
    def follow_trajectory(self, trajectory, JointStatePublisher):
        print("Following the trajectory ........")
        for i in range(0,len(trajectory)):
            dat = JointState()
            dat.position = trajectory[i]
            #print(dat)
            JointStatePublisher.publish(dat)
            time.sleep(0.1)
        print("done")

   