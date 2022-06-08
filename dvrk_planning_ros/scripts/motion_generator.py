#!/usr/bin/env python3

from turtle import distance
import trajectory_follower
import trajectory_generator
import time

import rospy
from sensor_msgs.msg import JointState

from threading import Thread

current_position = [0.0,0.0,0.0,0.0,0.0,0.0]
position_update_flag = 0
trajectory_update_flag = 0
trajectory = []


def generator_function():
    global trajectory_update_flag, current_position, trajectory, position_update_flag
    goal_js = [0.39965444803237915, -0.5209582448005676, 0.13364960253238678, -0.24901601672172546, 1.0345871448516846, -1.196233332157135]
    if not position_update_flag:
        time.sleep(0.3)

    goal1  = [[0, 1, 0, 0],[1, 0, 0, 0],[0, 0, -1, -0.1],[0,0,0,1]]
    goal = [[0.0637,0.9454,0.3197,0.0261],[0.9977,-0.0534,-0.0409,0.0395],[-0.0216,0.3216,-0.9466,-0.1045],[0,0,0,1]]
    
    goal2  = [[0, 1, 0, 0.05],[1, 0, 0, -0.05],[0, 0, -1, -0.1],[0,0,0,1]]
    goal3  = [[0, 1, 0, 0.05],[1, 0, 0, 0.05],[0, 0, -1, -0.1],[0,0,0,1]]
    goal4  = [[0, 1, 0, -0.05],[1, 0, 0, 0.05],[0, 0, -1, -0.1],[0,0,0,1]]
    goal5  = [[0, 1, 0, -0.05],[1, 0, 0, -0.05],[0, 0, -1, -0.1],[0,0,0,1]]
    goal6  = [[0, 1, 0, 0],[1, 0, 0, 0],[0, 0, -1, -0.1],[0,0,0,1]]


    generator_obj = trajectory_generator.Trajectories()
    if position_update_flag==1:
        print(current_position)
        #trajectory = generator_obj.toppra(current_position,goal_js)
        #trajectory = generator_obj.generate_traj(current_position, [goal1])
        trajectory = generator_obj.generate_traj (current_position, [goal2, goal3, goal4, goal5, goal6])

        trajectory_update_flag = 1
        #print(trajectory)

# [goal1,goal2, goal3, goal4, goal5, goal6]

# [[goal1,goal2], [goal3, goal4], [goal5, goal6]]

def follower_function():
    global trajectory_update_flag, current_position, trajectory, position_update_flag
    follower_obj = trajectory_follower.Follower()

    if not trajectory_update_flag:
        time.sleep(0.3)
    if trajectory_update_flag == 1:
        follower_obj.follow_trajectory(trajectory,JointStatePublisher)
    else:
        print ("Trajectory not here yet, retrying ......")
        follower_function()

def position_callback(msg):
        global current_position, position_update_flag
        current_position = msg.position
        position_update_flag = 1

if __name__ == '__main__':
    Thread(target = generator_function).start()
    Thread(target = follower_function).start()
    rospy.init_node('MotionGenerator', anonymous= True)
    rospy.Subscriber("/PSM2/measured_js",JointState,position_callback)
    JointStatePublisher = rospy.Publisher('/PSM2/servo_jp', JointState, queue_size = 1)
    try:
        rospy.spin()
    except KeyboardInterrupt:
        print("shutting down")
