#!/usr/bin/env python3

import numpy as np
import roboticstoolbox as rtb
import toppra as ta
import toppra.constraint as constraint
import toppra.algorithm as algo
import math
from scipy.spatial.transform import Rotation as R
from scipy.spatial.transform import Slerp
from dvrk_planning.kinematics.psm import PsmKinematicsSolver, LND400006


###############################################
#config
###############################################
interpolating_distance = 0.01
no_of_points_jp = 50

dof = 6
vlims = 10 + np.random.rand(dof) * 20
alims = 10 + np.random.rand(dof) * 2

###############################################



def linear_interpolator_function( start_point, end_point):
    global interpolating_distance
    distance = math.sqrt( math.pow((start_point[0]-end_point[0]),2) +math.pow((start_point[1]-end_point[1]),2) + math.pow((start_point[2]-end_point[2]),2) )
    #print (distance)
    n = interpolating_distance
    waypoints = [start_point]
    while n<distance:
        factor = n/distance
        x = start_point[0] + (factor * (end_point[0]-start_point[0]) )
        y = start_point[1] + (factor * (end_point[1]-start_point[1]) )
        z = start_point[2] + (factor * (end_point[2]-start_point[2]) )
        n = interpolating_distance+n
        #print (n)
        waypoints.append([x,y,z])
    waypoints.append(end_point)
    #print (waypoints)


    return waypoints

def rotational_interpolator ( no_of_points, start_orientation, end_rotation):
    np.set_printoptions(precision = 4, suppress = True)
    
    key_rots = R.from_matrix( [start_orientation, end_rotation] )

    key_times = [0, 1]

    slerp = Slerp(key_times, key_rots)

    #print (key_rots.as_matrix())

    times = np.array(list(range(0, no_of_points)))
    times = times / (no_of_points-1) 

    #print (no_of_points)

    interp_rots = slerp(times)
    #print (interp_rots.as_euler('xyz', degrees=True))
    
    return interp_rots.as_matrix()

def toppra(current_position=[], goal_js=[], way_pts = [0.005]):    
        
        global vlims, alims, no_of_points_jp

        
        if way_pts[0] == 0.01:
            way_pts = [current_position,goal_js]
        
        N_samples = len(way_pts)
        ss = np.linspace(0, 1, N_samples)


        path = ta.SplineInterpolator(ss, way_pts)
        pc_vel = constraint.JointVelocityConstraint(vlims)
        pc_acc = constraint.JointAccelerationConstraint(alims)
        instance = algo.TOPPRA([pc_vel, pc_acc], path, parametrizer="ParametrizeConstAccel")
        jnt_traj = instance.compute_trajectory()
        ts_sample = np.linspace(0, jnt_traj.duration, no_of_points_jp)
        qs_sample = jnt_traj(ts_sample)
        qds_sample = jnt_traj(ts_sample, 1) 
        qdds_sample = jnt_traj(ts_sample, 2)
        trajectory = qs_sample
        return trajectory



    
def generate_waypoints_jp ( current_jp, goal):
    p = PsmKinematicsSolver(LND400006())
    current_postion_cartesian = p.compute_fk(current_jp)
    
    #print (current_postion_cartesian)
    current_postion_cartesian=current_postion_cartesian.getA()
    #print (current_postion_cartesian)
    curr_xyz = []
    curr_xyz.append (current_postion_cartesian[0][3])
    curr_xyz.append (current_postion_cartesian[1][3])
    curr_xyz.append (current_postion_cartesian[2][3])
    goal_xyz = []
    goal_xyz.append (goal[0][3])
    goal_xyz.append (goal[1][3])
    goal_xyz.append (goal[2][3])
    
    

    rot_curr = [current_postion_cartesian[0][:3], current_postion_cartesian[1][:3], current_postion_cartesian[2][:3]]
    rot_goal = [goal[0][:3], goal[1][:3], goal[2][:3]]

    waypoints_xyz = linear_interpolator_function(curr_xyz, goal_xyz)
    waypoints_rot = rotational_interpolator (len(waypoints_xyz), rot_curr, rot_goal)
    waypoints_jp = []
    for i in range(0,len(waypoints_xyz)):
        temp = waypoints_rot[i].tolist()
        #print (temp)
        for j in range(0,3):
            temp[j].append (waypoints_xyz[i][j])
        temp.append ([0,0,0,1])
        #np.append(np.array(temp),np.array([0,0,0,1]))
        # print ("wait")
        # print(temp)
        # print("crossed")
        temp_jp = p.compute_ik(np.matrix(temp))
        #print(temp_jp)
        waypoints_jp.append(temp_jp)
    waypoints = toppra (current_jp, waypoints_jp[len(waypoints_jp)-1], waypoints_jp)
    
    return waypoints

class Trajectories:
    
    def generate_traj(self, current_jp, goals = []):
        waypoints = generate_waypoints_jp ( current_jp, goals[0]).tolist()
        if len(goals)>1:
            for i in range (1, len(goals)):
                temp = generate_waypoints_jp ( waypoints[len(waypoints)-1], goals[i]).tolist()
                waypoints = waypoints + temp

        return waypoints



        

def quintic_traj(current_position, goal_js):
    print (goal_js)
    print (current_position)
    traj = rtb.jtraj(current_position, goal_js, 100)
    trajectory = traj.q
    return trajectory
