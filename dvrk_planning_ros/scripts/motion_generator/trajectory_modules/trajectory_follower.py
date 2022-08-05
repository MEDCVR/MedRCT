#!/usr/bin/env python3

from sensor_msgs.msg import JointState
import time
import numpy as np
import rospy
import math
from dvrk_planning.kinematics.psm import PsmKinematicsSolver, LND400006, RTS470007
from dvrk_planning_msgs.msg import TrajectoryStatus
from threading import Thread
from std_msgs.msg import Float64MultiArray

mode = ""

max_open = 40
max_close = 0
distance_close_open = 0.0015
point_ratio  = 15
jaw_trajectory = []
kinematics = LND400006()                #default kinematics, change from config
offset_update_flag = 0
offset = []
follower_interrupt_flag = [1]
current_trajectory = []
plot_message = []
plot_message_update_flag = 0
back_flag = 0
experiment_name = "test"
total_offset = [0,0,0]
shared_clicks = 0
shared_invoked_index = []

def set_config(config):
    global max_close, max_open, distance_close_open, kinematics, experiment_name
    kinematics = eval(config['motion_generation_parameters']['kinematics']['tool']+ "()")
    max_open = config['cutting_parameters']['max_open']
    max_close = config['cutting_parameters']['max_close']
    distance_close_open = config['cutting_parameters']['distance_close_open']
    experiment_name = config['experiment_name']

def set_offset(data):
        global offset_update_flag, offset
        offset = data
        offset_update_flag = 1
trajectory_edit_status =0
def plot():
    global plot_message, plot_message_update_flag
    trajectory = current_trajectory
    msg = []
    for i in range(0, len(trajectory)):
        p = PsmKinematicsSolver(kinematics)
        point = p.compute_fk(trajectory[i])
        point = point.getA()
        msg.append(point[0][3]) 
        msg.append(point[1][3]) 
        msg.append(point[2][3])
    plot_message = msg
    plot_message_update_flag = 1 
    #if trajectory_edit_status==1:
    f = open(experiment_name, "a")
    f.write("\n")
    f.write(str(msg))
    f.close()

def publish_plot_message(RvizTrajPublisher):
    global plot_message, plot_message_update_flag
    dat = Float64MultiArray()
    dat.data = plot_message
    RvizTrajPublisher.publish(dat)
    plot_message_update_flag = 0


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
        #print("increment",increment)
        increment = 0.06
        max_open = 1
        max_close = 0
        for i in range (0,points+5):
            if curr > max_open:
                mode = 1
            elif curr <max_close:
                mode = 0

            if mode == 0:
                jaw_trajectory.append(curr + increment)
                curr = curr + increment
            elif mode ==1 :
                jaw_trajectory.append(curr - increment)
                curr = curr - increment
    #print ("jaw trajectory", len(jaw_trajectory))

def offset_update_function(i, traj, cart_trajectory):
    import time
    # get the start time
    st = time.time()
    temp = traj.copy()

    global shared_clicks, total_offset

    global offset_update_flag, back_flag, trajectory_edit_status
    global shared_invoked_index
    total_offset[0] = total_offset[0] + offset[0]
    
    if offset[3]==1:
        back_flag =1
    else:
        total_offset[1] = total_offset[1] + offset[1]
        total_offset[2] = total_offset[2] + offset[2]
        shared_clicks = shared_clicks +1
        trajectory_edit_status = 1
        shared_invoked_index.append(i)
        while i< len(cart_trajectory):
            p = PsmKinematicsSolver(kinematics)
            # point = p.compute_fk(trajectory[i])
            # point = point.getA()
            point = cart_trajectory[i]
            #print (point)
            point[0][3] = point[0][3] + offset[0]
            point[1][3] = point[1][3] + offset[1]
            point[2][3] = point[2][3] + offset[2]
            

            temp[i] = p.compute_ik(np.matrix(point))

            i = i+1
        

    et = time.time()
    # get the execution time
    elapsed_time = et - st
    print('Execution time:', elapsed_time, 'seconds')

    offset_update_flag = 0
    return temp

class Follower:
    
    def set_follower_status(self,msg):
        global follower_interrupt_flag
        follower_interrupt_flag = msg

    def follow_trajectory(self, traj, JointStatePublisher, durations, interpolated_points, name, cartesian_traj, StatusPublisher, RvizTrajPublisher, JawStatePublisher=[], jaw_position=[]):
        global mode, current_trajectory, back_flag
        trajectory = traj.copy()
        current_trajectory = trajectory.copy()
        cartesian_trajectory = cartesian_traj.copy()
        Thread(target = plot).start()
        mode = name
        print ("Name:", mode)
        print("Following the trajectory ........")
        rate = durations[0]/interpolated_points[0]
        durations_index = 1
        points_index = 0
        curr = 0

        import time
        # get the start time
        st = time.time()
        
        if mode == "scissor":
            jaw_cutting(trajectory[0:interpolated_points[points_index]-1],jaw_position)
        i=0
        while i<len(trajectory):
            if rospy.is_shutdown():
                return
            if follower_interrupt_flag[0] == 0:
                print("Interupting follwer")
                return

            if offset_update_flag == 1:
                trajectory = offset_update_function(i, trajectory, cartesian_trajectory).copy()
                current_trajectory = trajectory.copy()
                Thread(target = plot).start()

            if plot_message_update_flag == 1:
                publish_plot_message(RvizTrajPublisher)

            if back_flag == 1:
                if i>15:
                    i = i - 15 
                else: 
                    i = 0
                back_flag=0
                #print("recd")
            
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
            i=i+1
            
            global shared_clicks, shared_invoked_index, total_offset
        if mode == "scissor":
            et = time.time()
            # get the execution time
            elapsed_time = et - st
            f = open("time-"+experiment_name, "a")
            f.write("\n")
            f.write("generated time: " + str(durations[0]))
            f.write("\n")
            f.write("actual time: " + str(elapsed_time))
            f.write("\n")
            f.write("number of times shared autonomy is invoked: " + str(shared_clicks))
            f.write("\n")
            f.write("total offset: " + str(total_offset))
            f.write("\n")
            f.write("trajectory indeces that shared was invoked at: " + str(shared_invoked_index))
            f.close()
            
            shared_clicks =0
            shared_invoked_index = []
            total_offset = [0,0,0]
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

   