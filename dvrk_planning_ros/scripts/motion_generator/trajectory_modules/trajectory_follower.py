#!/usr/bin/env python3

from sensor_msgs.msg import JointState
import time
import numpy as np
import rospy
import math
from dvrk_planning.kinematics.psm import PsmKinematicsSolver, LND400006, RTS470007
from dvrk_planning_msgs.msg import TrajectoryStatus


mode = ""

max_open = 40
max_close = 5
distance_close_open = 0.0015
point_ratio  = 15
jaw_trajectory = []
kinematics = LND400006()                #default kinematics, change from config
offset_update_flag = 0
offset = []
follower_interrupt_flag = [1]


def set_config(config):
    global max_close, max_open, distance_close_open, kinematics
    kinematics = eval(config['motion_generation_parameters']['kinematics']['tool']+ "()")
    max_open = config['cutting_parameters']['max_open']
    max_close = config['cutting_parameters']['max_close']
    distance_close_open = config['cutting_parameters']['distance_close_open']

def set_offset(data):
        global offset_update_flag, offset
        offset = data
        offset_update_flag = 1


def jaw_cutting(traj, jaw_position):
    global jaw_trajectory, max_open, max_close, distance_close_open, point_ratio
    #print ("trajectory", len(traj))
    max_open = max_open * (3.14/180)
    max_close = max_close * (3.14/180)
    inc_points = int((len(traj)-1)/point_ratio)
    #print (inc_points)
    points = inc_points
    points = points +1
    p = PsmKinematicsSolver(kinematics)
    mode = 0
    curr = jaw_position
    l=0
    while (l+inc_points) < (len(traj)):
        start = traj[l]
        l= l+ inc_points
        end = traj[l]
        start = p.compute_fk(start)
        end = p.compute_fk(end)
        start = start.getA()
        end = end .getA()
        distance = math.sqrt( math.pow((start[0][3]-end[0][3]),2) +math.pow((start[1][3]-end[1][3]),2) + math.pow((start[2][3]-end[2][3]),2) )
        increment_points = points / (distance/ distance_close_open)
        increment = (max_open - max_close )/increment_points
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
    #print ("jaw trajectory", len(jaw_trajectory))

def offset_update_function(i, trajectory):
    global offset_update_flag
    while i< len(trajectory):
        p = PsmKinematicsSolver(kinematics)
        point = p.compute_fk(trajectory[i])
        point = point.getA()
        point[0][3] = point[0][3] + offset[0]
        point[1][3] = point[1][3] + offset[1]
        point[2][3] = point[2][3] + offset[2]

        trajectory[i] = p.compute_ik(np.matrix(point))

        i = i+1

    offset_update_flag = 0
    return trajectory

class Follower:
    
    def set_follower_status(self,msg):
        global follower_interrupt_flag
        follower_interrupt_flag = msg

    def follow_trajectory(self, traj, JointStatePublisher, durations, interpolated_points, name, StatusPublisher,  JawStatePublisher=[], jaw_position=[]):
        global mode
        trajectory = traj.copy()
        mode = name
        print ("Name:", mode)
        print("Following the trajectory ........")
        rate = durations[0]/interpolated_points[0]
        durations_index = 1
        points_index = 0
        curr = 0
        
        if mode == "scissor":
            jaw_cutting(trajectory[0:interpolated_points[points_index]-1],jaw_position)
        for i in range(0,len(trajectory)):
            if rospy.is_shutdown():
                return
            if follower_interrupt_flag[0] == 0:
                print("Interupting follwer")
                return

            if offset_update_flag == 1:
                trajectory = offset_update_function(i, trajectory)

            dat = JointState()
            dat.position = trajectory[i]
            JointStatePublisher.publish(dat)

            statusDat = TrajectoryStatus()
            statusDat.trajectory_name = mode
            statusDat.percentage_completed = (i/len(trajectory))*100
            statusDat.completed_trajectory_points = i
            statusDat.total_trajectory_points = len(trajectory)
            StatusPublisher.publish(statusDat)
            time.sleep(rate/2)
            if mode == "scissor":
                dat = JointState()
                dat.position = [jaw_trajectory[i]]
                JawStatePublisher.publish(dat)
            if ((i-curr) >= interpolated_points[points_index]):
                points_index = points_index +1
                rate = durations[durations_index]/interpolated_points[points_index]
                curr = i
                durations_index = durations_index + 1

                if mode == "scissor":
                    jaw_cutting(trajectory[0:interpolated_points[points_index]-1],jaw_position)

            time.sleep(rate/2)
        print("done")

    def follow_jaw_trajectory(self, trajectory, JointStatePublisher, duration, interpolated_points):
        global follower_interrupt_flag
        if len(trajectory)>0:
            print("Following the jaw trajectory ........")
            rate = duration/interpolated_points
            
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

   