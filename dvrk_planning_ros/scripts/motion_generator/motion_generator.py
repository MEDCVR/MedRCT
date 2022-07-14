#!/usr/bin/env python3
import importlib
import argparse
import os.path
import rospy
import rospkg
import sys
import yaml
import time
import rospy

from trajectory_modules import trajectory_follower
from trajectory_modules import trajectory_generator
from cutting_modules import teleop_goal_interface
from sensor_msgs.msg import JointState
from sensor_msgs.msg import Joy
from threading import Thread
from dvrk_planning_msgs.msg import Waypoints
from dvrk_planning_msgs.msg import TrajectoryStatus
from scipy.spatial.transform import Rotation as R
from geometry_msgs.msg import Transform

cutting_cartesian_velocity = 0.00125


#Global variables
current_position = [0.0,0.0,0.0,0.0,0.0,0.0]
position_update_flag = 0
jaw_position = [0.0]
jaw_update_flag = 0
instructions = []
follower_interrupt_flag = [0]

last_point = 0
last_jaw = 0
def generator_function(data, mode, name):
    global  current_position,  position_update_flag, instructions, jaw_position, jaw_update_flag, cutting_cartesian_velocity
    global last_jaw, last_point
    if not position_update_flag:
        time.sleep(0.3)
    rospy.wait_for_message("/PSM2/measured_js",JointState, timeout=2)
    # last_point = 0
    # last_jaw = 0
    #while not rospy.is_shutdown():
    #name =""

    #data, mode = menu ()
    #data, mode, name = teleop_goals()

    generator_obj = trajectory_generator.Trajectories()
    if position_update_flag==1:
        if mode == "arm":
            if last_point == 0:
                last_point = current_position
            trajectory, durations, interpolated_points = generator_obj.generate_traj (last_point, data, name, cutting_cartesian_velocity)
            if trajectory == []:
                print("already here, no trajectory to generate")
            else:
                last_point = trajectory [len(trajectory)-1]
                instructions.append(  {'type':'trajectory', 'trajectory': trajectory, 'durations': durations, 'interpolated_points': interpolated_points, 'name':name } )
        if mode == "jaw":
            if jaw_update_flag ==1:
                if last_jaw == 0:
                    last_jaw = jaw_position
                jaw_traj, duration, points = generator_obj.generate_traj_jaw(last_jaw, data)
                instructions.append(  {'type':'jaw', 'trajectory': jaw_traj, 'duration': duration, 'num_points': points } )
                if len(jaw_traj)>0:
                    last_jaw = jaw_traj [len(jaw_traj)-1]

def follower_function():
    global current_position, position_update_flag, jaw_position
    follower_obj = trajectory_follower.Follower()
    display_flag = 0
    while not rospy.is_shutdown():
        if follower_interrupt_flag[0]==1:
            if len(instructions) == 0:
                if display_flag == 0:
                    print ("Waiting for further instructions ......")
                    display_flag = 1
            else: 
                display_flag = 0
                if instructions[0]['type'] == 'trajectory':
                    follower_obj.follow_trajectory(instructions[0]['trajectory'],JointStatePublisher, instructions[0]['durations'], instructions[0]['interpolated_points'],instructions[0]['name'], StatusPublisher, JawStatePublisher, jaw_position)
                    instructions.pop(0)
                elif instructions[0]['type'] == 'jaw':
                    follower_obj.follow_jaw_trajectory(instructions[0]['trajectory'],JawStatePublisher, instructions[0]['duration'], instructions[0]['num_points'])
                    instructions.pop(0)

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
        status = goal_obj.get_goals()
        #status = goal_obj.temp()
        #a = input("hold")
        if (status == 0):
             
            temp = goal_obj.sequential_goals()
            data = temp[0]
            name = temp[1]
            if data == []:
                status = 1
    if data == []:
        print ("some mess here")
    return [data], mode, name

def jaw_callback(msg):
    global jaw_position, jaw_update_flag
    jaw_position = msg.position[0]
    jaw_update_flag = 1

def enable():
    global follower_interrupt_flag, position_update_flag
    print ("Motion generator enabled.....")
    follower_interrupt_flag = [1]
    follower_obj = trajectory_follower.Follower()
    follower_obj.set_follower_status(follower_interrupt_flag)
    position_update_flag = 0

def disable():
    global follower_interrupt_flag, position_update_flag
    print ("Motion generator disabled.....")
    follower_interrupt_flag = [0]
    follower_obj = trajectory_follower.Follower()
    follower_obj.set_follower_status(follower_interrupt_flag)
    position_update_flag = 0

def switching_callback(msg):
    if msg.buttons[0] == 0:
        disable()
    elif msg.buttons[0] == 1:
        enable()
    else:
        print("Something went wrong with the switching")

def set_config(config):
    global cutting_cartesian_velocity
    cutting_cartesian_velocity = config['cutting_parameters']['cutting_cartesian_velocity']
    trajectory_follower.set_config(config)
    trajectory_generator.set_config(config)

def msg_rotMatrix(waypoints):
    waypoints_output=[]
    #print("all goals", waypoints)
    for i in range(0, len(waypoints)):
        
        r = R.from_quat([waypoints[i].rotation.x, waypoints[i].rotation.y, waypoints[i].rotation.z, waypoints[i].rotation.w])
        temp = r.as_matrix()
        point = temp.tolist()
        
        point[0].append(waypoints[i].translation.x)
        point[1].append(waypoints[i].translation.y)
        point[2].append(waypoints[i].translation.z)
        
        point.append([0,0,0,1])
        

        waypoints_output.append(point)
    #print("wayout", waypoints_output)
    return waypoints_output

def waypoint_callback(msg):
    data = msg_rotMatrix(msg.waypoints)
    generator_function([data], msg.instruction_mode, msg.trajectory_name)


if __name__ == '__main__':

    argv = rospy.myargv(argv=sys.argv)

    # Parse arguments
    parser = argparse.ArgumentParser()
    parser.add_argument('-p', '--config_package', type=str, required=False,
                        default = "dvrk_planning_ros",
                        help = 'the package where you store the config yaml, default is "dvrk_planning_ros"')
    parser.add_argument('-y', '--yaml', type=str, required=False,
                        default = "config/teleop.yaml",
                        help = 'the relative path to your yaml file from package directory, default is "config/teleop.yaml"')
    args = parser.parse_args(argv[1:]) # skip argv[0], script name

    # Find yaml full path
    config_package_name = args.config_package
    relative_yaml_path = args.yaml
    package_path = rospkg.RosPack().get_path(config_package_name)
    full_config_path = os.path.join(package_path, relative_yaml_path)

    # Parse yamlposition
    yaml_file = open(full_config_path)
    config_yaml = yaml.load(yaml_file, Loader=yaml.FullLoader)
    #print(config_yaml)
    set_config(config_yaml)

    rospy.init_node('MotionGenerator', anonymous= True)
    rospy.Subscriber(config_yaml['ros_communication'][0]['input']['waypoint_input'],Waypoints,waypoint_callback)
    rospy.Subscriber(config_yaml['ros_communication'][0]['input']['joint_positions_topic'],JointState,position_callback)
    rospy.Subscriber(config_yaml['ros_communication'][0]['input']['jaw_position_topic'],JointState,jaw_callback)
    rospy.Subscriber(config_yaml['toggle_topic'], Joy, switching_callback)
    JointStatePublisher = rospy.Publisher(config_yaml['ros_communication'][0]['output']['joint_control_topic'], JointState, queue_size = 10)
    JawStatePublisher = rospy.Publisher(config_yaml['ros_communication'][0]['output']['jaw_control_topic'], JointState, queue_size = 10)
    StatusPublisher = rospy.Publisher(config_yaml['ros_communication'][0]['output']['trajectory_status_topic'], TrajectoryStatus, queue_size = 10)
    #Thread(target = generator_function).start()
    Thread(target = follower_function).start()
    try:
        rospy.spin()
    except KeyboardInterrupt:
        print("shutting down")



