#!/usr/bin/env python3

import argparse
from re import X
from tkinter import Y
import numpy as np
import os.path
import rospy
import rospkg
import sys
import yaml

from geometry_msgs.msg import TransformStamped, Twist
from sensor_msgs.msg import JointState

from PyKDL import Rotation
from dvrk_planning.controller.teleop_controller import FollowTeleopController, IncrementTeleopController


def to_pykdl(quaternion_yaml):
    x = quaternion_yaml["x"]
    y = quaternion_yaml["y"]
    z = quaternion_yaml["z"]
    w = quaternion_yaml["w"]

    return Rotation.Quaternion(x, y, z, w)

class RosTeleopController:
    def __init__(self, controller_yaml):
        self.name = controller_yaml["name"]
        self.output_topic = rospy.Publisher(controller_yaml["output_topic"], JointState, queue_size = 1)
        self.output_topic = rospy.Subscriber(controller_yaml["output_feedback_topic"], JointState, self._output_feedback_callback)

        output_base_to_camera_rot = Rotation.Quaternion(0, 0, 0, 1)
        output_base_to_camera_rot_str = "output_to_camera_rot"
        if(output_base_to_camera_rot_str in controller_yaml):
            output_base_to_camera_rot = to_pykdl(controller_yaml[output_base_to_camera_rot_str])

        input_topic_name = controller_yaml["input_topic"]
        if controller_yaml["type"] == "follow":
            self.teleop_controller = FollowTeleopController(output_to_camera_rot = output_base_to_camera_rot)
            self.input_sub = rospy.Subscriber(input_topic_name, TransformStamped, self._input_callback_tf)
        elif controller_yaml["type"] == "increment":
            self.teleop_controller = IncrementTeleopController(output_to_camera_rot = output_base_to_camera_rot)
            self.input_sub = rospy.Subscriber(input_topic_name, Twist, self._input_callback_twist)
        else:
            raise NotImplementedError

    # def get_current_output_tf():

    # def get_current_input_tf():

    # Output feedback needs a tf, but js is the simplest type of
    # data for a robot controller, so lets start with that.
    def _output_feedback_callback(self, js):
        x = 0

    def _output_callback(self, js):
        x = 0

    def _input_callback_tf(self, data):
        self.input_tf = data.transform

    def _input_callback_twist(self,data):
        x = 0

class DvrkTeleopNode:
    def __init__(self, config_yaml):
        # print(config_yaml)
        self.ros_teleop_controllers = {}

        for controller_yaml in config_yaml["controllers"]:
            self.ros_teleop_controllers[controller_yaml["name"]] = RosTeleopController(controller_yaml)

        self.clutch_sub = rospy.Subscriber("/console/clutch", Twist, self.clutch_callback)
    def clutch_callback(self, data):
        if data.buttons[0] == 1:
            for rtc in self.ros_teleop_controllers:
                rtc.teleop_controller.clutch()
        elif data.buttons[0] == 0:
            for rtc in self.ros_teleop_controllers:
                # rtc.teleop_controller.unclutch()
                # Need to account fo different clutch style

if __name__ == '__main__':
    argv = rospy.myargv(argv=sys.argv)

    # Parse arguments
    parser = argparse.ArgumentParser()
    parser.add_argument('-cp', '--config_package', type=str, required=False,
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

    # Parse yaml
    yaml_file = open(full_config_path)
    config_yaml = yaml.load(yaml_file, Loader=yaml.FullLoader)

    rospy.init_node('dvrk_teleop')
    try:
        dvrk_teleop_node = DvrkTeleopNode(config_yaml = config_yaml)
        rospy.spin()
    except rospy.ROSInterruptException: pass
