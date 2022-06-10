import rospy

def goal_gen(inp_data):
    goal0  = [[0.0637,0.9454,0.3197,0.0261],[0.9977,-0.0534,-0.0409,0.0395],[-0.0216,0.3216,-0.9466,-0.1045],[0,0,0,1]]
    goal1  = [[0, 1, 0, 0],[1, 0, 0, 0],[0, 0, -1, -0.1],[0,0,0,1]]
    goal2  = [[0, 1, 0, 0.05],[1, 0, 0, -0.05],[0, 0, -1, -0.1],[0,0,0,1]]
    goal3  = [[0, 1, 0, 0.05],[1, 0, 0, 0.05],[0, 0, -1, -0.1],[0,0,0,1]]
    goal4  = [[0, 1, 0, -0.05],[1, 0, 0, 0.05],[0, 0, -1, -0.1],[0,0,0,1]]
    goal5  = [[0, 1, 0, -0.05],[1, 0, 0, -0.05],[0, 0, -1, -0.1],[0,0,0,1]]
    goal6  = [[0, 1, 0, 0],[1, 0, 0, 0],[0, 0, -1, -0.1],[0,0,0,1]]
    goals = [goal0, goal1, goal2, goal3, goal4, goal5, goal6]
    index = 0
    mode = 0
    #try:
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

    # except:
    #     menu()
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
    if results[0]=='o' or results[0]=='c':
        data = results[0]
        mode = 1
    elif results[1]=='.' or results[1]==',' or results[1]=='e':
        data = goal_gen (results)
        mode = 2
    else:
        menu()
    #results = list(map(int, results))
    print(data)
    return data, mode

menu()


# temp = [-0.04894161596894264, 0.29180410504341125, 0.10410408675670624, 0.05287863314151764, -0.25886258482933044, -0.011124979704618454]
# temp = [0.0003846920153591782, 0.01227362547069788, 0.10040482878684998, -3.604878656915389e-06, -0.0013414063723757863, 4.482344593270682e-06]

# from dvrk_planning.kinematics.psm import PsmKinematicsSolver, LND400006
# #from timer import Timer

# import numpy as np

# np.set_printoptions(precision = 4, suppress = True)

# #t = Timer(timer_repeat_times = 100)
# p = PsmKinematicsSolver(LND400006())

# def test_fk_ik(joint_pos, test_time = False):
#     # if(test_time):
#     #     output_fk = t.time_average(lambda : p.compute_fk(joint_pos))
#     #     fk_avg_time = t.str_average()
#     #     output_jp = t.time_average(lambda : p.compute_ik(output_fk))
#     #     ik_avg_time = t.str_average()
#     # else:
#     output_fk = p.compute_fk(joint_pos)
#     output_jp = p.compute_ik(output_fk)  
      
#     # Print results
#     print("Input joint pos: \n", joint_pos)
#     print("FK: \n", output_fk)
#     print("IK: \n", np.array(output_jp))
#     # if(test_time):
#     #     print("FK time: ", fk_avg_time)
#     #     print("IK time: ", ik_avg_time)

#     print("-------------------------------------")

# joint_change = 0.1

# joint_pos = temp
# test_fk_ik(joint_pos)



# from scipy.spatial.transform import Rotation as R

# from scipy.spatial.transform import Slerp

# start_orientation = [ [0, -1, 0],[1, 0, 0],[0, 0, 1],]

# end_rotation = [[1, 0, 0], [0, 0, -1], [0, 1, 0]]
# current_postion_cartesian = start_orientation

# goal_xyz = []
# goal_xyz.append (current_postion_cartesian[0][2])
# goal_xyz.append (current_postion_cartesian[1][2])
# goal_xyz.append (current_postion_cartesian[2][2])
# print (goal_xyz)

# key_rots = R.from_matrix( [start_orientation, end_rotation] )

# key_times = [0, 1]

# slerp = Slerp(key_times, key_rots)

# times = [0, 0.5, 0.25, 1]
# interp_rots = slerp(times)

# print (key_rots.as_matrix())
# print ("hi")
# print (interp_rots.as_matrix())
# print ("hello")
# print (key_rots.as_euler('xyz', degrees=True))
# print ("hi")
# print (interp_rots.as_euler('xyz', degrees=True))

# a = [1,1,1,1,1,1,1,1,1,1,1,1,1,1,1,1,1,1]
# print (len (a))

# import numpy
# a_list = list(range(0, (len(a)) ))
# a_list = numpy.array(a_list)
# print (a_list)
# a_list = a_list / (len(a)-1) 
# print(a_list)





# import math
# end_point = [2,7,4]
# start_point = [1,2,3]
# def linear_interpolator_function(start_point, end_point):
#     interpolating_distance = 0.5
#     distance = math.sqrt( math.pow((start_point[0]-end_point[0]),2) +math.pow((start_point[1]-end_point[1]),2) + math.pow((start_point[2]-end_point[2]),2) )
#     print (distance)
#     n = interpolating_distance
#     waypoints = []
#     while n<distance:
#         factor = n/distance
#         x = start_point[0] + (factor * (end_point[0]-start_point[0]) )
#         y = start_point[1] + (factor * (end_point[1]-start_point[1]) )
#         z = start_point[2] + (factor * (end_point[2]-start_point[2]) )
#         n = interpolating_distance+n
#         print (n)
#         waypoints.append([x,y,z])
    
#     print (waypoints)
#     print (len(waypoints))
#     return waypoints

# temp = linear_interpolator_function(start_point, end_point)
# for i in range (1,10):
#     distance = math.sqrt( math.pow((temp[i][0]-temp[i-1][0]),2) +math.pow((temp[i][1]-temp[i-1][1]),2) + math.pow((temp[i][2]-temp[i-1][2]),2) )
#     print (distance)


# import matplotlib.pyplot as plt

# from mpl_toolkits.mplot3d import Axes3D

# fig = plt.figure(figsize=(4,4))

# ax = fig.add_subplot(111, projection='3d')

# fig = plt.figure(figsize=(4,4))

# ax = fig.add_subplot(111, projection='3d')

# for i in range (1,10):
#     ax.scatter(temp[i][0], temp[i][1], temp[i][2]) # plot the point (2,3,4) on the figure

# plt.show()