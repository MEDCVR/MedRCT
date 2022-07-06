#!/usr/bin/env python3


#rospy.wait_for_message(self.output_feedback_topic, JointState, timeout=0.01)



import trajectory_follower
import trajectory_generator

from cutting_modules import teleop_goal_interface

import time

import rospy
from sensor_msgs.msg import JointState

from threading import Thread

cutting_cartesian_velocity = 0.005

current_position = [0.0,0.0,0.0,0.0,0.0,0.0]
position_update_flag = 0
jaw_position = [0.0]
jaw_update_flag = 0


instructions = []


########################################################
#Pre-defined goals menu
########################################################
def goal_gen(inp_data):
    goal0  = [[0.0637,0.9454,0.3197,0.0261],[0.9977,-0.0534,-0.0409,0.0395],[-0.0216,0.3216,-0.9466,-0.1045],[0,0,0,1]]
    goal1  = [[0, 1, 0, 0],[1, 0, 0, 0],[0, 0, -1, -0.1],[0,0,0,1]]
    goal2  = [[0, 1, 0, 0.05],[1, 0, 0, -0.05],[0, 0, -1, -0.1],[0,0,0,1]]
    goal3  = [[0, 1, 0, 0.05],[1, 0, 0, 0.05],[0, 0, -1, -0.1],[0,0,0,1]]
    goal4  = [[0, 1, 0, -0.05],[1, 0, 0, 0.05],[0, 0, -1, -0.1],[0,0,0,1]]
    goal5  = [[0, 1, 0, -0.05],[1, 0, 0, -0.05],[0, 0, -1, -0.1],[0,0,0,1]]
    goal6  = [[0, 1, 0, 0],[1, 0, 0, 0],[0, 0, -1, -0.1],[0,0,0,1]]
    goal7  = [[0, 1, 0, 0.03],[1, 0, 0, 0],[0, 0, -1, -0.1],[0,0,0,1]]
    goal8  = [[0.0637,0.9454,0.3197,0],[0.9977,-0.0534,-0.0409,0],[-0.0216,0.3216,-0.9466,-0.1],[0,0,0,1]]

    #pick and place
    # goal0  = [[0, 1, 0, 0],[1, 0, 0, 0],[0, 0, -1, -0.1],[0,0,0,1]]
    # goal1  = [[0, 1, 0, 0],[1, 0, 0, -0.05],[0, 0, -1, -0.13],[0,0,0,1]]
    # goal2  = [[0, 1, 0, 0],[1, 0, 0, -0.03],[0, 0, -1, -0.1],[0,0,0,1]]
    # goal3  = [[0, 1, 0, 0],[1, 0, 0, 0],[0, 0, -1, -0.07],[0,0,0,1]]
    # goal4  = [[0, 1, 0, 0],[1, 0, 0, 0.03],[0, 0, -1, -0.1],[0,0,0,1]]
    # goal5  = [[0, 1, 0, 0],[1, 0, 0, 0.05],[0, 0, -1, -0.13],[0,0,0,1]]
    # goal6 = []
    # goal7 = []
    # goal8 = []

    goals = [goal0, goal1, goal2, goal3, goal4, goal5, goal6, goal7, goal8]
    index = 0
    mode = 0
    try:
        output = [ [goals[int(inp_data[0])]] ]

        if (inp_data[1]=='e'):
            #print (inp_data)
            return output

        for i in range(1,len(inp_data)-1):
            
            if inp_data[i]=='.':
                mode = 0 
            elif inp_data[i]==',':
                mode = 1

            elif mode == 1:
                output[index].append( goals[int(inp_data[i])] )
            elif mode == 0:
                output[index].append( goals[int(inp_data[i])] )
                output.append ([ goals[int(inp_data[i])] ])
                index = index + 1

    except Exception as e: 
        print(e)
        menu()
    return output 

def menu():
    if rospy.is_shutdown():
        return
    
    print("separate goals with '.' and waypoints with ','" )
    print("points are 0-9")
    print("o/c for opening/closing jaw")
    data = ''
    mode = 0
    results = list(input ())
    results.append('e')
    #print (results)
    if rospy.is_shutdown():
        exit()
    if results[0]=='o' or results[0]=='c':
        data = results[0]
        mode = 1
    elif results[1]=='.' or results[1]==',' or results[1]=='e':
        data = goal_gen (results)
        mode = 2
    else:
        menu()
    #results = list(map(int, results))
    print ("Using the following data for goal generation....")
    print(data)
    return data, mode
########################################################


def generator_function():
    global  current_position,  position_update_flag, instructions, jaw_position, jaw_update_flag, cutting_cartesian_velocity

    #goal_js = [0.39965444803237915, -0.5209582448005676, 0.13364960253238678, -0.24901601672172546, 1.0345871448516846, -1.196233332157135]

    if not position_update_flag:
        time.sleep(0.3)
    rospy.wait_for_message("/PSM2/measured_js",JointState, timeout=0.1)
    last_point = 0
    last_jaw = 0
    while not rospy.is_shutdown():

        # print (current_position)
        # print (position_update_flag)



        #data, mode = menu ()
        data, mode, name = teleop_goals()



        # goal_obj = teleop_goal_interface.CuttingInput()
        # data = goal_obj.temp()
        # mode = 2
        #print(data)
        #a = input("hold")
        # print("data")
        # print (data)
        # print ("last point")
        # print (last_point)

        generator_obj = trajectory_generator.Trajectories()
        if position_update_flag==1:
            #print(current_position)
            #trajectory = generator_obj.toppra(current_position,goal_js)
            #trajectory = generator_obj.generate_traj(current_position, [goal1])

            if mode == 2:
                if last_point == 0:
                    last_point = current_position
                trajectory, durations, interpolated_points = generator_obj.generate_traj (last_point, data, name, cutting_cartesian_velocity)
                if trajectory == []:
                    print("already here, no trajectory to generate")
                else:
                    #print(trajectory)
                    last_point = trajectory [len(trajectory)-1]
                    instructions.append(  {'type':'trajectory', 'trajectory': trajectory, 'durations': durations, 'interpolated_points': interpolated_points, 'name':name } )
                
            if mode ==1:
                if jaw_update_flag ==1:
                    if last_jaw == 0:
                        last_jaw = jaw_position
                    jaw_traj, duration, points = generator_obj.generate_traj_jaw(last_jaw, data)
                    instructions.append(  {'type':'jaw', 'trajectory': jaw_traj, 'duration': duration, 'num_points': points } )
                    if len(jaw_traj)>0:
                        last_jaw = jaw_traj [len(jaw_traj)-1]

    #print(instructions)
    # [goal1,goal2, goal3, goal4, goal5, goal6]
    # [[goal1,goal2], [goal3, goal4], [goal5, goal6]]

def follower_function():
    global current_position, position_update_flag, jaw_position
    follower_obj = trajectory_follower.Follower()

    display_flag = 0
    while not rospy.is_shutdown():
        if len(instructions) == 0:
            if display_flag == 0:
                print ("Waiting for further instructions ......")
                display_flag = 1
        else: 
            display_flag = 0
            if instructions[0]['type'] == 'trajectory':
                follower_obj.follow_trajectory(instructions[0]['trajectory'],JointStatePublisher, instructions[0]['durations'], instructions[0]['interpolated_points'],instructions[0]['name'], JawStatePublisher, jaw_position)
                instructions.pop(0)
            elif instructions[0]['type'] == 'jaw':
                follower_obj.follow_jaw_trajectory(instructions[0]['trajectory'],JawStatePublisher, instructions[0]['duration'], instructions[0]['num_points'])
                instructions.pop(0)


    # if rospy.is_shutdown():
    #     return
    # if not trajectory_update_flag:
    #     time.sleep(0.3)
    # if trajectory_update_flag == 1:
    #     follower_obj.follow_trajectory(trajectory,JointStatePublisher, durations, interpolated_points)
    # else:
    #     print ("Trajectory not here yet, retrying ......")
    #     follower_function()

def position_callback(msg):
        global current_position, position_update_flag
        current_position = msg.position
        
        position_update_flag = 1

def get_current_position():
    global current_position
    print (current_position)
    return current_position

def teleop_goals():

    mode = 2
    status = 0
    goal_obj = teleop_goal_interface.CuttingInput()
    temp = goal_obj.sequential_goals()
    data = temp[0]
    name = temp[1]
    
    if data == []:
        status = 1
    while status: 
        #status = goal_obj.get_goals()
        status = goal_obj.temp()
        a = input("hold")
        if (status == 0):
             
            temp = goal_obj.sequential_goals()
            data = temp[0]
            name = temp[1]
            if data == []:
                status = 1
    if data == []:
        print ("some mess here")

    #print (data)
    # print(temp)
    # print (name)
    return [data], mode, name

def jaw_callback(msg):
    global jaw_position, jaw_update_flag
    jaw_position = msg.position[0]
    jaw_update_flag = 1


if __name__ == '__main__':
    
    rospy.init_node('MotionGenerator', anonymous= True)
    rospy.Subscriber("/PSM2/measured_js",JointState,position_callback)
    rospy.Subscriber("/PSM2/jaw/measured_js",JointState,jaw_callback)
    JointStatePublisher = rospy.Publisher('/PSM2/servo_jp', JointState, queue_size = 1)
    JawStatePublisher = rospy.Publisher('/PSM2/jaw/servo_jp', JointState, queue_size = 1)
    Thread(target = generator_function).start()
    Thread(target = follower_function).start()
    try:
        rospy.spin()
    except KeyboardInterrupt:
        print("shutting down")



