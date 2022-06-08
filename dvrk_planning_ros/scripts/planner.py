#!/usr/bin/env python3

import rospy
from geometry_msgs.msg import TwistStamped
from sensor_msgs.msg import JointState
import time
import numpy as np
import roboticstoolbox as rtb

import toppra as ta
import toppra.constraint as constraint
import toppra.algorithm as algo
import matplotlib.pyplot as plt

current_position = [0.0,0.0,0.0,0.0,0.0,0.0]
updated = 0
goal_js = [0.0,0.0,0.0,0.0,0.0,0.0]
trajectory = []

class Constraints:
    velocity_constraints = []
    accln_constraints = []

class CartesianGoal:
    goals = []
    waypoints = []
    rotation_goals = []
    rotation_waypoints = []

def position_callback(msg):
    global current_position, updated
    current_position = msg.position
    #print("updated")
    updated = 1

def follow_trajectory():
    global trajectory
    for i in range(0,len(trajectory)):
        dat = JointState()
        dat.position = trajectory[i]
        print(dat)
        JointStatePublisher.publish(dat)
        time.sleep(0.1)
    print("done")


def generate_trajectory_rtb():
    global current_position, goal_js, trajectory
    traj = rtb.jtraj(current_position, goal_js, 100)
    trajectory = traj.q
    
    
    fig, axs = plt.subplots(3)
    axs[0].plot(list(range(1, 101)), traj.q)
    axs[1].plot(list(range(1, 101)), traj.qd)
    axs[2].plot(list(range(1, 101)), traj.qd)

    axs[2].set_xlabel("Time (s)")
    axs[0].set_ylabel("Position (rad)")
    axs[1].set_ylabel("Velocity (rad/s)")
    axs[2].set_ylabel("Acceleration (rad/s2)")
    #plt.show()

    generate_trajectory_toppra()

    #follow_trajectory()


def generate_trajectory_toppra():
        
    global current_position, goal_js, trajectory


    # Parameters
    N_samples = 2
    dof = 6
    way_pts = np.array([current_position,goal_js])
    print(way_pts)
    
    ss = np.linspace(0, 1, N_samples)

    vlims = 10 + np.random.rand(dof) * 20
    alims = 10 + np.random.rand(dof) * 2
    

    path = ta.SplineInterpolator(ss, way_pts)
    pc_vel = constraint.JointVelocityConstraint(vlims)
    pc_acc = constraint.JointAccelerationConstraint(alims)
    instance = algo.TOPPRA([pc_vel, pc_acc], path, parametrizer="ParametrizeConstAccel")
    jnt_traj = instance.compute_trajectory()
    ts_sample = np.linspace(0, jnt_traj.duration, 100)
    qs_sample = jnt_traj(ts_sample)
    qds_sample = jnt_traj(ts_sample, 1) 
    qdds_sample = jnt_traj(ts_sample, 2)

    print(qds_sample)
    fig, axs = plt.subplots(3, 1, sharex=True)
    for i in range(path.dof):
        # plot the i-th joint trajectory
        axs[0].plot(ts_sample, qs_sample[:, i], c="C{:d}".format(i))
        axs[1].plot(ts_sample, qds_sample[:, i], c="C{:d}".format(i))
        axs[2].plot(ts_sample, qdds_sample[:, i], c="C{:d}".format(i))
    axs[2].set_xlabel("Time (s)")
    axs[0].set_ylabel("Position (rad)")
    axs[1].set_ylabel("Velocity (rad/s)")
    axs[2].set_ylabel("Acceleration (rad/s2)")
    plt.show()

    trajectory = qs_sample
    #follow_trajectory()



def accept_goal():
    global goal_js
    goal_js = [0.39965444803237915, -0.5209582448005676, 0.13364960253238678, -0.24901601672172546, 1.0345871448516846, -1.196233332157135]
    while not updated:
        time.sleep(0.1)
    if updated==1:
        #generate_trajectory_toppra()
        generate_trajectory_rtb()

if __name__ == '__main__':
    rospy.init_node('MotionGenerator', anonymous= True)
    rospy.Subscriber("/PSM2/measured_js",JointState,position_callback)
    JointStatePublisher = rospy.Publisher('/PSM2/servo_jp', JointState, queue_size = 1)
    accept_goal()
    try:
        rospy.spin()
    except KeyboardInterrupt:
        print("shutting down")













# max_velocity_x = 2.034007847401901e-05
# max_velocity_y = 2.034007847401901e-05
# max_velocity_z = 2.034007847401901e-05

# current_velocity_x = 0.0
# current_velocity_y = 0.0
# current_velocity_z = 0.0

# time_to_max = 0.0
# start_time = 0.0

# def vel_func(limit, time_elapsed, max_vel,mode):
#     global time_to_max
#     if limit > max_vel:
#         limit = max_vel

#     #trapezoidal    
#     m=0.000025
#     if mode==0:
#         result = m * time_elapsed
#     elif mode == -1:
#         result = 0
#     else:
#         result = -m * time_elapsed + ( m* ( time_elapsed+time_to_max))
    
#     if result > limit:
#         result = limit
#         if time_to_max == 0.0:
#             time_to_max = time_elapsed

#     #print(result)

#     return result

# def velocity_mode(time, vel, x):
#     global time_to_max
#     distance_travelled = time * vel
#     mode = 0
#     if distance_travelled >= (x/2):
#         mode = 1
#     if time_to_max>0.0:
#         if x-distance_travelled <= (0.5*time_to_max*vel):
#             mode = 1
#     if x-distance_travelled < 0.00002:
#         mode = -1

#     print(distance_travelled )
#     return mode
   
# def velocity_calc(x,y,z):
#     global start_time, current_velocity_x, current_velocity_y, current_velocity_z
#     time_elapsed = time.time() - start_time
    
#     current_velocity_x = vel_func((0.7*x), time_elapsed, max_velocity_x, velocity_mode(time_elapsed,current_velocity_x,x))
#     current_velocity_y = vel_func((0.7*y), time_elapsed, max_velocity_y, velocity_mode(time_elapsed,current_velocity_y,y))
#     current_velocity_z = vel_func((0.7*z), time_elapsed, max_velocity_z, velocity_mode(time_elapsed,current_velocity_z,z))

#     msg = TwistStamped()
#     msg.twist.linear.x = current_velocity_x
#     msg.twist.linear.y = current_velocity_y
#     msg.twist.linear.z = current_velocity_z
#     pub = rospy.Publisher('/keyboard/twist', TwistStamped, queue_size = 1)
#     pub.publish(msg)


# def go_to_goal(goal, start):
#     x = goal[0] - start[0]
#     y = goal[1] - start[1]
#     z = goal[2] - start[2]
    
#     while True:
#         velocity_calc(x,y,z)


# #if __name__=='__main__':
# def start():
#     rospy.init_node('planner')
#     pub = rospy.Publisher('/keyboard/twist', TwistStamped, queue_size = 1)
    
#     goal = [0.000215,0.000215,0.0]
#     start = [0,0,0]
#     global current_position, start_time
#     current_position = start
#     start_time = time.time() 
#     go_to_goal (np.array(goal),np.array(start))

# #start()
from dvrk_planning.kinematics.psm import PsmKinematicsSolver, LND400006
#from timer import Timer

import numpy as np

np.set_printoptions(precision = 4, suppress = True)

#t = Timer(timer_repeat_times = 100)
p = PsmKinematicsSolver(LND400006())

def test_fk_ik(joint_pos, test_time = False):
    # if(test_time):
    #     output_fk = t.time_average(lambda : p.compute_fk(joint_pos))
    #     fk_avg_time = t.str_average()
    #     output_jp = t.time_average(lambda : p.compute_ik(output_fk))
    #     ik_avg_time = t.str_average()
    # else:
    output_fk = p.compute_fk(joint_pos)
    output_jp = p.compute_ik(output_fk)  
      
    # Print results
    print("Input joint pos: \n", joint_pos)
    print("FK: \n", output_fk)
    print("IK: \n", np.array(output_jp))
    # if(test_time):
    #     print("FK time: ", fk_avg_time)
    #     print("IK time: ", ik_avg_time)

    print("-------------------------------------")

joint_change = 0.1

joint_pos = [0.0, 0.0, joint_change, 0.0, 0.0, 0.0]
test_fk_ik(joint_pos)

joint_pos[1] = -joint_change
test_fk_ik(joint_pos)

joint_pos[0] = joint_change
test_fk_ik(joint_pos)

joint_pos[3] = joint_change
joint_pos =  [0.27342814207077026, -0.5036982893943787, 0.11685218662023544, 0.011314896866679192, 0.5170695781707764, -0.27725905179977417]

test_fk_ik(joint_pos)

joint_pos[4] = joint_change
joint_pos = [0.0007754408870823681, 0.07288364320993423, 0.10093997418880463, -1.1464053386589512e-05, -0.05122923105955124, -2.1644532353093382e-05]
test_fk_ik(joint_pos)

joint_pos[5] = joint_change

joint_pos = [0.0007719629211351275, -0.5215796828269958, 0.11377278715372086, 5.5424960009986535e-05, 0.5348151326179504, -6.279001536313444e-05]

test_fk_ik(joint_pos, test_time= True)

# import roboticstoolbox as rtb

# puma = rtb.models.DH.Puma560()

# traj = rtb.jtraj([0.0,0.0,0,0,0,0], joint_pos, 100)

# print(traj.q.shape)
# #rtb.tools.trajectory.Trajectory.qplot(traj.q)
# print(traj.q)



 
###############################################################################################################################################################


########################################################################################################################################################################
    # while True:
    #     msg = TwistStamped()
    #     msg.twist.linear.y = -0.01
    #     pub.publish(msg)

# import numpy as np
# # Should remove this if kinematics don't use
# from PyKDL import Frame, Rotation, Vector

# from dvrk_planning.controller.teleop_controller import TeleopController, InputType, ControllerType
# from dvrk_planning.utilities import convert_frame_to_mat

#  np.array([])



# def _update_output_tf(self, input_diff_tf, output_tf):
#         input_diff_rotated_p = self._rotation_adjustment(input_diff_tf)
#         input_diff_rot, _ = get_rot_and_p(input_diff_tf)
#         cur_output_rot, cur_output_p = get_rot_and_p(output_tf)

#         cur_output_p = cur_output_p + input_diff_rotated_p
#         cur_output_rot = np.matmul(cur_output_rot, input_diff_rot)

#         output_tf[0:3, 0:3] = cur_output_rot
#         output_tf[0:3, 3] = cur_output_p

#     def update(self, *args):
#         if(not self._update()):
#             return False

#         absolute_output_tf = self._update_impl(args)
#         output_js = self.kinematics_solver.compute_ik(absolute_output_tf)
#         self.output_callback(output_js)
#         return True