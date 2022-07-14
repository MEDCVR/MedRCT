#!/usr/bin/env python3
from numpy import deprecate_with_doc
import rospy
from dvrk_planning.kinematics.psm import PsmKinematicsSolver, LND400006, RTS470007

kinematics = RTS470007()

from scipy.spatial.transform import Rotation as R
from sensor_msgs.msg import JointState
import math

import numpy as np
import matplotlib.pyplot as plt
from scipy import interpolate
from dvrk_planning_msgs.msg import Waypoints
from geometry_msgs.msg import Transform

###############################################
#config
###############################################
# the depth the cutting tool should go further than the waypoint
depth = 0.0001
#orientation of the cutting tool to be maintained
x_incline = 80
y_incline =  0
z_rotation = 90
#Interpolation parameters
interpolated_points = 200
smoothing_factor = 0.01
polynomial_degree = 3
###############################################

all_waypoints = []
current_position = []


def remove_duplicates (x_sample,y_sample,z_sample):
    
    #Todo
    #for i in range():
    return x_sample,y_sample,z_sample
        
       


def get_goals():
    index = 0
    mode = 0
    goal_output = []
    global current_position
    
    
    while not rospy.is_shutdown():
        print("g if current location is a goal")
        print("w if current location is a waypoint")
        print("s to send the points to the generator")
        print("d to discard the points")
        choice = input()
        #try:
        temp = list(choice)
        #curr_position = motion_generator.get_current_position()
        curr_position = current_position
        if temp[0] == 'g' or temp[0] == 'G':
            mode = 0 
        elif temp[0] == 'w' or temp[0] == 'W' or temp[0] == 's' or temp[0] == 'S':
            mode = 1
        elif temp[0] == 'd' or temp[0] == 'D':
            goal_output = []
            mode = 2
        else: 
            print ("invalid input, try again ......")
        
        if len (goal_output) ==0 and mode!=2:
            rospy.wait_for_message("/PSM2/measured_js",JointState, timeout=2)
            goal_output.append([list( curr_position )] )
        else:
            rospy.wait_for_message("/PSM2/measured_js",JointState, timeout=2)
            if mode == 1:
                goal_output[index].append( list( curr_position ))
            elif mode == 0:
                goal_output[index].append( list( curr_position ) )
                goal_output.append ([list( curr_position )])
                index = index + 1
        if temp[0] == 's' or temp[0] == 'S':
            if (len(goal_output))<1:
                print("nothing to send")
                get_goals()
            elif (len(goal_output[0]))<2:
                print("choose at least 2 points")
                get_goals()
            else: 
                send_goals(goal_output)
                get_goals()
        print(goal_output)
        # except:
        #     print ("something went wrong, try again ......")
        #     return 1 


def send_goals(goal_output):
    global all_waypoints
    for i in range (0, len(goal_output)):
        waypoints = []
        for j in range(0, len(goal_output[i])):
            p = PsmKinematicsSolver(kinematics)
            current_postion_cartesian = p.compute_fk(goal_output[i][j])
            current_postion_cartesian = current_postion_cartesian.getA()
            waypoints.append (list(current_postion_cartesian))

        waypoints = interpolator(waypoints)

        waypoints.reverse()

        # r = R.from_euler('xyz', [(-180 ), y_incline, z_rotation], degrees=True)
        # temp = r.as_matrix()
        # temp = temp.tolist()

        temp =  [[0, 1, 0],[1, 0, 0],[0, 0, -1]]

    
        
        waypoint = temp
        waypoint[0].append (waypoints[0] [0][3])
        waypoint[1].append (waypoints[0] [1][3])
        waypoint[2].append (waypoints[0] [2][3] )
        waypoint.append ([0,0,0,1])
        #print(waypoints[ (len(waypoints)-1)][0][3])
        #print (waypoint[0][3])
        #all_waypoints.append({ 'waypoints':[ waypoint, waypoints[0]], 'name':"goal"})
        #all_waypoints.append({ 'waypoints':[ waypoints[0]], 'name':"goal"})
        publish_goals("goal", [ waypoints[0]])
        
        #all_waypoints.append({ 'waypoints':waypoints, 'name':"scissor"})
        publish_goals("scissor", waypoints)

        temp_waypointf = waypoints[ (len(waypoints)-1)]
        temp2 =  [[0, 1, 0],[1, 0, 0],[0, 0, -1]]
        waypoint2 = temp2
        waypoint2[0].append (waypoints[ (len(waypoints)-1)] [0][3])
        waypoint2[1].append (waypoints[ (len(waypoints)-1)] [1][3])
        waypoint2[2].append (waypoints[ (len(waypoints)-1)] [2][3] + (2*depth))
        waypoint2.append ([0,0,0,1])
        #print("all_waypoints", all_waypoints)


        #all_waypoints.append({ 'waypoints':[waypoints[ (len(waypoints)-1)], waypoint2], 'name':"goal"})
        publish_goals("goal", [waypoints[ (len(waypoints)-1)], waypoint2])
        
        

        
        #print (goal_output_cart)


def rotMatix_msg(goals):
    
    waypoints_output=[]
    #print("all goals", goals)
    for i in range(0, len(goals)):
        
        temp = Transform()
        temp.translation.x = goals[i][0][3]
        temp.translation.y = goals[i][1][3]
        temp.translation.z = goals[i][2][3]
        
        goal = goals[i]
        #print(goal)
        r = R.from_matrix([goal[0][:3], goal[1][:3], goal[2][:3]])
        quat = r.as_quat()

        temp.rotation.x = quat[0]
        temp.rotation.y = quat[1]
        temp.rotation.z = quat[2]
        temp.rotation.w = quat[3]

        waypoints_output.append(temp)
    #print("wayout", waypoints_output)
    return waypoints_output

def publish_goals(name, goals):
    dat = Waypoints()
    dat.trajectory_name = name
    dat.instruction_mode = "arm"
    dat.waypoints = rotMatix_msg(goals)
    GoalStatePublisher.publish(dat)


from scipy import interpolate
from mpl_toolkits.mplot3d import Axes3D

def interpolator(waypoints):
    global interpolated_points, smoothing_factor, polynomial_degree
    x_sample = []
    y_sample = []
    z_sample = []
    for i in range(0, len(waypoints)):
        x_sample.append(waypoints[i][0][3])
        y_sample.append(waypoints[i][1][3])
        z_sample.append(waypoints[i][2][3])
    # print(x_sample)
    # print(y_sample)
    # print(z_sample)
    
    x_sample,y_sample,z_sample = remove_duplicates (x_sample,y_sample,z_sample)
    if (len(x_sample))<4:
        polynomial_degree = len(x_sample)-1
    tck, u = interpolate.splprep([x_sample,y_sample,z_sample], s=smoothing_factor, k = polynomial_degree)
    x_knots, y_knots, z_knots = interpolate.splev(tck[0], tck)
    u_fine = np.linspace(0,1,interpolated_points )
    x_fine, y_fine, z_fine = interpolate.splev(u_fine, tck)


    # fig2 = plt.figure(2)
    # ax3d = fig2.add_subplot(111, projection='3d')

    # ax3d.plot(x_sample, y_sample, z_sample, 'r*')
    # ax3d.plot(x_knots, y_knots, z_knots, 'go')
    # ax3d.plot(x_fine, y_fine, z_fine, 'g')
    # #print(x_fine[0])
    # fig2.show()
    # plt.show()

    return jaw_orientation([x_fine, y_fine, z_fine])

def jaw_orientation(goals):
    global depth, x_incline
    #print (goals)

    waypoints = []
    for i in range (0, len(goals[0])-1 ):
        y = goals[1][i+1] - goals[1][i]
        x = goals[0][i+1] - goals[0][i]
        angle = math.atan2(y,x) *180/3.14
        #print (angle)
        r = R.from_euler('xyz', [(-180 + x_incline), y_incline, (angle)+ z_rotation], degrees=True)
        temp = r.as_matrix()
        temp = temp.tolist()

        waypoint = temp
        waypoint[0].append (goals[0][i])
        waypoint[1].append (goals[1][i])
        waypoint[2].append (goals[2][i])
        waypoint.append ([0,0,0,1])
        waypoints.append(waypoint)
        
        if i == len(goals[0])-1:
            waypoint = temp
            waypoint[0].append (goals[0][i+1])
            waypoint[1].append (goals[1][i+1])
            waypoint[2].append (goals[2][i+1])
            waypoints.append(waypoint)

    return(waypoints)

def position_callback(msg):
    global current_position, position_update_flag
    current_position = msg.position

if __name__ == '__main__':


    rospy.init_node('Teleop_Goal_interface', anonymous= True)
    rospy.Subscriber("/PSM2/measured_js", JointState, position_callback)
    GoalStatePublisher = rospy.Publisher("/input/waypoints", Waypoints, queue_size = 10)
    get_goals()
    try:
        rospy.spin()
    except KeyboardInterrupt:
        print("shutting down")




#################################################################################################################################
        # final_waypoints = []
        # for k in range(0,3):
        #     for l in range(0,3):

    # for i in range (0, len(goals)):
    #     for j in range(0, len(goals[i])-1):
    #         goal = goals[i][j]
    #         goal_xyz = []
    #         goal_xyz.append (goal[0][3])
    #         goal_xyz.append (goal[1][3])
    #         goal_xyz.append (goal[2][3] - depth)

    #         goal = goals[i][j+1]
    #         goaln_xyz = []
    #         goaln_xyz.append (goal[0][3])
    #         goaln_xyz.append (goal[1][3])
    #         goaln_xyz.append (goal[2][3] - depth)
            
    #         #angle = (goaln_xyz[1] - goal_xyz [1]) / (goaln_xyz[0] - goal_xyz[0] + 0.000000000000001)
    #         #angle = math.atan(angle) *180/3.14
    #         y = (goaln_xyz[1] - goal_xyz [1])
    #         x = (goaln_xyz[0] - goal_xyz[0] + 0.000000000000001)
    #         angle = math.atan2(y,x) *180/3.14
    #         print("angle")
    #         print (angle)
    #         # print("sign")
    #         # print (  (goaln_xyz[1] - goal_xyz [1])   )
    #         # if (goaln_xyz[1] - goal_xyz [1]) <0:
    #         #     angle = -180-angle
    #         # print(goal_xyz)
    #         # print(r.as_matrix())
    #         # print(r.as_euler('xyz', degrees=True))

    #         r = R.from_euler('xyz', [(-180 + x_incline), 0, (angle)], degrees=True)
    #         temp = r.as_matrix()
    #         for k in range(0,3):
    #             for l in range(0,3):
    #                 goals[i][j][k][l] = temp[k][l]
    #                 goals[i][j+1][k][l] = temp[k][l]
    #     temp = goals[i][len(goals[i])-1] 
    # return goals


    

    

 #print( current_position)



    # def __init__():
    #     #rospy.init_node('TeleopGoals', anonymous= True)
    #     rospy.Subscriber("/PSM2/measured_js", JointState, position_callback)
    
def temp():
    #cartesian
    # goal1  = [[0, 1, 0, 0],[1, 0, 0, 0],[0, 0, -1, -0.1],[0,0,0,1]]
    # goal2  = [[0, 1, 0, 0.05],[1, 0, 0, -0.05],[0, 0, -1, -0.1],[0,0,0,1]]
    # goal3  = [[0, 1, 0, 0.05],[1, 0, 0, 0.05],[0, 0, -1, -0.1],[0,0,0,1]]
    # goal4  = [[0, 1, 0, -0.05],[1, 0, 0, 0.05],[0, 0, -1, -0.1],[0,0,0,1]]
    # goal5  = [[0, 1, 0, -0.05],[1, 0, 0, -0.05],[0, 0, -1, -0.1],[0,0,0,1]]
    # goal6  = [[0, 1, 0, 0],[1, 0, 0, 0],[0, 0, -1, -0.1],[0,0,0,1]]
    # goal7  = [[0, 1, 0, 0.03],[1, 0, 0, 0],[0, 0, -1, -0.1],[0,0,0,1]]
    # goals = [ [goal2, goal7, goal3  ]]
    # return jaw_orientation (goals)  

    #goals = [[[-0.054713692516088486, 0.48224759101867676, 0.11257047206163406, 0.012590869329869747, -0.45482897758483887, -0.520109549164772], [0.3911094069480896, 0.2764164209365845, 0.11329736560583115, -0.009045740589499474, -0.25353044271469116, -0.9698815643787384], [0.3906344771385193, -0.1170898824930191, 0.11087666451931, -0.0009759304230101407, 0.1348545253276825, -0.9718144237995148], [0.08866144716739655, -0.5139707922935486, 0.11582803726196289, -0.00554535910487175, 0.5273154973983765, -0.6652765870094299], [-0.0710441991686821, -0.3180985450744629, 0.10754289478063583, -0.009388613514602184, 0.33412256836891174, -0.5032211691141129]]]
    #needle driver jp
    # goal1 =  [0.0, -0.0, 0.09630000000000001, 0.0, -0.0, 0.0]
    # goal2 =  [0.5080435050560344, 0.49026081966106505, 0.12178957769747238, 0.0, -0.4902608196610651, -0.508043505056034]
    # goal3 =  [0.5080435050560344, -0.49026081966106505, 0.12178957769747238, -0.0, 0.4902608196610651, -0.5080435050560338]
    # goal4 =  [-0.5080435050560344,alims -0.49026081966106505, 0.12178957769747238, 0.0, 0.4902608196610651, 0.5080435050560345]
    # goal5 =  [-0.5080435050560344, 0.49026081966106505, 0.12178957769747238, -0.0, -0.4902608196610651, 0.508043505056034]
    # goal6 =  [0.0, -0.0, 0.09630000000000001, 0.0, -0.0, 0.0]
    # goal7 =  [0.32241855696929245, -0.0, 0.1011786142695382, 0.0, -0.0, -0.32241855696929195]

    #scissor 5cm

    # goal1 =   [0.0, -0.0, 0.04229000000000005, 0.0, -0.0, 0.0]
    # goal2 =  [0.5090392201162325, 0.49107539010122325, 0.06782779286340597, 0.0, -0.49107539010122314, -0.5090392201162321]
    # goal3 =  [0.5090392201162325, -0.49107539010122325, 0.06782779286340597, -0.0, 0.49107539010122286, -0.5090392201162325]
    # goal4 =  [-0.5090392201162325, -0.49107539010122325, 0.06782779286340597, 0.0, 0.49107539010122286, 0.5090392201162321]
    # goal5 =  [-0.5090392201162325, 0.49107539010122325, 0.06782779286340597, -0.0, -0.49107539010122314, 0.5090392201162321]
    # goal6 =  [0.0, -0.0, 0.04229000000000005, 0.0, -0.0, 0.0]
    # goal7 =  [0.32312284660346086, -0.0, 0.04717945861402896, 0.0, -0.0, -0.32312284660346136]

    #scissor 3cm
    goal1 =   [0.0, -0.0, 0.04229000000000005, 0.0, -0.0, 0.0]
    goal2 =  [0.32312284660346086, 0.3378970772970957, 0.05229669581385103, 0.0, -0.3378970772970953, -0.32312284660346097]
    goal3 =  [0.32312284660346086, -0.3378970772970957, 0.05229669581385103, -0.0, 0.3378970772970953, -0.32312284660346097]
    goal4 =  [-0.32312284660346086, -0.3378970772970957, 0.05229669581385103, 0.0, 0.3378970772970953, 0.32312284660346136]
    goal5 =  [-0.32312284660346086, 0.3378970772970957, 0.05229669581385103, -0.0, -0.3378970772970953, 0.32312284660346097]
    goal6 =  [0.0, -0.0, 0.04229000000000005, 0.0, -0.0, 0.0]
    goal7 =  [0.32312284660346086, -0.0, 0.04717945861402896, 0.0, -0.0, -0.32312284660346136]

    goals = [[goal2, goal3, goal4]]

    send_goals (goals)
    return 0


def sequential_goals():
    global all_waypoints
    if len(all_waypoints)>0:
        temp = all_waypoints[0]['waypoints']
        name = all_waypoints[0]['name']
        all_waypoints.pop(0)
        return temp, name
    else:
        return [], ""