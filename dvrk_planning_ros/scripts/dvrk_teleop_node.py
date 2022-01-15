#!/usr/bin/env python3

import argparse
import numpy as np
import os.path
import rospy
import rospkg
import sys
import yaml

from geometry_msgs.msg import TransformStamped, Twist
from sensor_msgs.msg import JointState

from dvrk_planning.controller.teleop_controller import FollowTeleopController, IncrementTeleopController

class DvrkTeleopNode:
    def __init__(self, config_yaml = ""):
        input_topic_name = "mtm/measured_cp"
        output_topic_name = "psm/servo_jp"
        self.output_topic = rospy.Publisher(output_topic_name, JointState, queue_size = 1)

        output_base_to_camera_rot = np.identity(4)
        # if(config_yaml)

        if config_yaml.type == "follow":
            self.teleop_controller = FollowTeleopController(output_to_camera_rot = output_base_to_camera_rot)
            self.input_sub = rospy.Subscriber(input_topic_name, TransformStamped, self.input_callback_tf)
        elif config_yaml.type == "increment":
            self.teleop_controller = IncrementTeleopController(output_to_camera_rot = output_base_to_camera_rot)
            self.input_sub = rospy.Subscriber(input_topic_name, Twist, self.input_callback_twist)
        else:
            raise NotImplementedError

    def input_callback_tf(self, data):
        x = 0

    def input_callback_twist(self,data):
        x = 0

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
