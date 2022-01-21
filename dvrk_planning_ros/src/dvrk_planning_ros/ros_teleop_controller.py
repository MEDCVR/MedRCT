import rospy
import numpy as np

from dvrk_planning.kinematics.utilities import get_harmonized_joint_positions, TWO_PI
from sensor_msgs.msg import JointState

class RosTeleopController:
    def __init__(self, controller_yaml):
        self.name = controller_yaml["name"]
        output_yaml = controller_yaml["output"]
        self.js_msg = JointState()
        self.output_pub = rospy.Publisher(output_yaml["control_topic"], JointState, queue_size = 1)
        self.output_feedback_topic = output_yaml["feedback_topic"]
        self.output_feedback_sub = rospy.Subscriber(self.output_feedback_topic, JointState, self._output_feedback_callback)

        self.current_output_jps = np.array([])

    def _get_str_name(self):
        return "Teleop controller [" + self.name + "]"

    def _wait_for_output_feedback_sub_msg(self, always_print = False):
        try:
            if(always_print):
                 raise
            rospy.wait_for_message(self.output_feedback_topic, JointState, timeout=0.01) # timeout 0.1s to see if publishing
        except:
            print(self._get_str_name(), ": waiting for message from topic [" + self.output_feedback_topic +"]" )
            rospy.wait_for_message(self.output_feedback_topic, JointState)
            print(self._get_str_name(), ": finished for message from topic [" + self.output_feedback_topic +"]" )

    def enable(self):
        raise NotImplementedError

    def disable(self):
        raise NotImplementedError

    def clutch(self):
        raise NotImplementedError

    def unclutch(self):
        raise NotImplementedError

    def _output_callback(self, joint_positions):
        self.js_msg.position = get_harmonized_joint_positions(joint_positions, self.current_output_jps)
        self.output_pub.publish(self.js_msg)

    def _output_feedback_callback(self, js):
        self.js_msg.name =  js.name
        self.current_output_jps = js.position
