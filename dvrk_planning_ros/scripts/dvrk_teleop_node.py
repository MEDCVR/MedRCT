#!/usr/bin/env python3

import argparse
import os.path
import rospy
import rospkg
import sys
import yaml

from geometry_msgs.msg import TransformStamped, Twist
from sensor_msgs.msg import JointState, Joy

from PyKDL import Rotation, Vector
from dvrk_planning.controller.teleop_controller import FollowTeleopController, IncrementTeleopController, ControllerType
from dvrk_planning_ros.utils import gm_tf_to_numpy_mat

def to_pykdl(quaternion_yaml):
    x = quaternion_yaml["x"]
    y = quaternion_yaml["y"]
    z = quaternion_yaml["z"]
    w = quaternion_yaml["w"]

    return Rotation.Quaternion(x, y, z, w)

class RosTeleopController:
    def __init__(self, controller_yaml):
        self.name = controller_yaml["name"]
        self.output_pub = rospy.Publisher(controller_yaml["output_topic"], JointState, queue_size = 1)
        self.output_feedback_topic = controller_yaml["output_feedback_topic"]
        self.output_feedback_sub = rospy.Subscriber(self.output_feedback_topic, JointState, self._output_feedback_callback)

        output_base_to_camera_rot = Rotation.Quaternion(0, 0, 0, 1)
        output_base_to_camera_rot_str = "output_to_camera_rot"
        if(output_base_to_camera_rot_str in controller_yaml):
            output_base_to_camera_rot = to_pykdl(controller_yaml[output_base_to_camera_rot_str])

        self.input_topic = controller_yaml["input_topic"]
        if controller_yaml["type"] == "follow":
            self._teleop_controller = FollowTeleopController(output_to_camera_rot = output_base_to_camera_rot)
            self.input_topic_type = TransformStamped
            sub_callback = self._input_callback_tf
        elif controller_yaml["type"] == "increment":
            self._teleop_controller = IncrementTeleopController(output_to_camera_rot = output_base_to_camera_rot)
            self.input_topic_type = Twist
            sub_callback = self._input_callback_twist
        else:
            raise NotImplementedError
        self.input_sub = rospy.Subscriber(self.input_topic, self.input_topic_type, sub_callback)

        self._teleop_controller.register(self._output_callback)

    def _get_str_name(self):
        return "Teleop controller [" + self.name + "]"

    def _wait_for_subscribers_message(self):
        print(self._get_str_name(), ": waiting for message from topic [" + self.input_topic +"]" )
        rospy.wait_for_message(self.input_topic, self.input_topic_type)
        print(self._get_str_name(), ": finised waiting for message from topic [" + self.input_topic +"]" )

        print(self._get_str_name(), ": waiting for message from topic [" + self.output_feedback_topic +"]" )
        rospy.wait_for_message(self.output_feedback_topic, self.JointState)
        print(self._get_str_name(), ": finised for message from topic [" + self.output_feedback_topic +"]" )

    def enable(self):
        self._wait_for_subscribers_message()
        # TODO, this is not good oop
        if self._teleop_controller.controller_type == ControllerType.INCREMENT:
            self._teleop_controller.enable(self.current_output_tf)
        elif self._teleop_controller.controller_type == ControllerType.FOLLOW:
            self._teleop_controller.enable(self.current_input_tf, self.current_output_tf)

    def disable(self):
        self._teleop_controller.disable()

    def clutch(self):
        self._teleop_controller.clutch()

    def unclutch(self):
        # TODO, this is not good oop
        if self._teleop_controller.controller_type == ControllerType.INCREMENT:
            self._teleop_controller.unclutch()
        elif self._teleop_controller.controller_type == ControllerType.FOLLOW:
            self._wait_for_subscribers_message()
            self._teleop_controller.unclutch(self.current_input_tf, self.current_output_tf)

    # Output feedback needs a tf, but js is the simplest type of
    # data for a robot controller, so lets start with that.
    def _output_feedback_callback(self, js):
        self.current_output_tf = self._teleop_controller.fk_function(js)

    def _output_callback(self, js):
        self.output_pub.publish()

    def _input_callback_tf(self, data):
        self.current_input_tf = gm_tf_to_numpy_mat(data.transform)
        self.teleop_controller.update(self.current_input_tf)

    def _input_callback_twist(self, data):
        self.teleop_controller.update(
            Vector(data.linear.x, data.linear.y, data.linear.z),
            Rotation.RPY(data.angular.x, data.angular.y, data.angular.z))

class DvrkTeleopNode:
    def __init__(self, config_yaml):
        # print(config_yaml)
        self.ros_teleop_controllers = {}

        for controller_yaml in config_yaml["controllers"]:
            self.ros_teleop_controllers[controller_yaml["name"]] = RosTeleopController(controller_yaml)

        clutch_topic = "/console/clutch"
        if ("clutch_topic" in config_yaml):
            clutch_topic = config_yaml["clutch_topic"]
            self.clutch_sub = rospy.Subscriber(clutch_topic, Joy, self.clutch_callback)

        switch_topic = "/console/camera"
        if ("switcher" in config_yaml):
            switcher_yaml = config_yaml["switcher"]
            switch_topic = switcher_yaml["switch_topic"]
            self.disable_list = switcher_yaml["disable_list"]
            self.enable_list = switcher_yaml["enable_list"]
            self.switch_sub = rospy.Subscriber(switch_topic, Joy, self.switch_callback)

        for _, rtc in self.ros_teleop_controllers.items():
            rtc.enable()

    def clutch_callback(self, data):
        if data.buttons[0] == 1:
            for _, rtc in self.ros_teleop_controllers.items():
                rtc.clutch()
        elif data.buttons[0] == 0:
            for _, rtc in self.ros_teleop_controllers.items():
                rtc.unclutch()

    def _execute_enable_disable(self, enable_list, disable_list):
        for name in enable_list:
            self.ros_teleop_controllers[name].enable()
        for name in disable_list:
            self.ros_teleop_controllers[name].disable()

    def switch_callback(self, data):
        if data.buttons[0] == 1:
            self._execute_enable_disable(self.enable_list, self.disable_list)
        elif data.buttons[0] == 0:
            self._execute_enable_disable(self.disable_list, self.enable_list)

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
