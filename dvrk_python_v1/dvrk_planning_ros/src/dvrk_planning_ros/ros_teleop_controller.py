from dataclasses import dataclass
import rospy
import numpy as np
import threading

from dataclasses import dataclass
from sensor_msgs.msg import JointState

from dvrk_planning.kinematics.utilities import get_harmonized_joint_positions, TWO_PI
from dvrk_planning_ros.average_timer import AverageTimer

class JointStateControlInterface:
    def __init__(self, control_topic_name, feedback_topic_name, store_js_func, prev_jsci = None):
        self.control_topic_name = control_topic_name
        self.feedback_topic_name = feedback_topic_name
        self.store_js_func = store_js_func
        self.prev_jsci = prev_jsci

        self.control_topic_name_pub = rospy.Publisher(control_topic_name, JointState, queue_size = 1)
        self.feedback_topic_sub = rospy.Subscriber(self.feedback_topic_name, JointState, self._feedback_callback)

        self.size_and_idx_obtained = False
        self.size = -1
        self.start_idx = -1

        self.js_msg = JointState()
        self.feedback_js_msg = None
        self.is_enabled = False
        self._enabled_condition = threading.Condition()

    def enable(self, joint_state_names = None):
        self.is_enabled = True
        self._enabled_condition.acquire()
        self._enabled_condition.wait() # To make sure caller() gets store_js_func() done at least 1x
                                       # when enabled
        self._enabled_condition.release()

        if joint_state_names:
            self.js_msg.name = joint_state_names[self.start_idx: self.start_idx + self.size]
        else: # None
            self.js_msg.name = self.feedback_js_msg.name

    def _feedback_callback(self, js_msg):
        self.feedback_js_msg =  js_msg
        if self.size_and_idx_obtained and self.is_enabled:
            self.store_js_func()[self.start_idx: self.start_idx + self.size] = js_msg.position
            self._enabled_condition.acquire()
            self._enabled_condition.notify() 
            self._enabled_condition.release()

    def wait_for_output_feedback_sub_msg(self, always_print = False, prepend_str = ""):
        try:
            if always_print:
                rospy.wait_for_message(self.feedback_topic_name, JointState, timeout=0.1)
                raise
        except:
            print(prepend_str, ": waiting for message from topic [" + self.feedback_topic_name +"]")
            rospy.wait_for_message(self.feedback_topic_name, JointState)
            print(prepend_str, ": finished for message from topic [" + self.feedback_topic_name +"]")

    def obtain_size_and_index(self):
        self.size = len(self.feedback_js_msg.position)
        if not self.prev_jsci:
            self.start_idx = 0
            self.size_and_idx_obtained = True
            return
        # else
        if not self.prev_jsci.size_and_idx_obtained: # if not called in for loop
            self.prev_jsci.obtain_size_and_index() # recursiveness
        self.start_idx = self.prev_jsci.start_idx + self.prev_jsci.size
        self.size_and_idx_obtained = True
        return

    def publish(self, joint_state_positions):
        self.js_msg.position = joint_state_positions[self.start_idx: self.start_idx + self.size]
        self.control_topic_name_pub.publish(self.js_msg)
@dataclass
class JointLimits:
    lower: np.ndarray = None
    upper: np.ndarray = None

class RosTeleopController:
    def __init__(self, controller_yaml, ros_input_type,
                 joint_state_names = None, is_print_wait_msg = False):
        self.name = controller_yaml["name"]
        self.input_topic = controller_yaml["input"]["topic"]
        self.ros_input_type = ros_input_type
        self.is_print_wait_msg = is_print_wait_msg
        self.joint_state_names = joint_state_names
        if "is_timed" in controller_yaml and controller_yaml["is_timed"]:
            self._average_timer = AverageTimer()
            self.input_sub = rospy.Subscriber(self.input_topic, self.ros_input_type, self._input_callback_timed)
        else:
            self._average_timer = None
            self.input_sub = rospy.Subscriber(self.input_topic, self.ros_input_type, self._input_callback_impl)

        output_yaml = controller_yaml["output"]
        self.js_control_interfaces = []
        prev_jsci = None
        for control_interface_yaml in output_yaml["control_interfaces"]:
            jsci = JointStateControlInterface(
                    control_interface_yaml["control_topic"],
                    control_interface_yaml["feedback_topic"],
                    self.get_current_output_jps,
                    prev_jsci)
            self.js_control_interfaces.append(jsci)
            prev_jsci = jsci

        self.__current_output_jps = np.array([])
        self.__is_currrent_output_jps_initialized = False

        self.joint_limits = None
        if "joint_limits" in output_yaml:
            joint_limits_yaml = output_yaml["joint_limits"]
            joint_limits = self._get_joint_limits(joint_limits_yaml) # Raise error if fail
            self.joint_limits = joint_limits

    def get_current_output_jps(self):
        if not self.__is_currrent_output_jps_initialized:
            raise "__is_currrent_output_jps_initialized is False, enable the RosTeleopController() one time first"
        return self.__current_output_jps


    def _get_joint_limits(self, joint_limits_yaml):
        # TODO If this was in the correct place like teleop controller, we can check joint size
        # and teleop controller has knowledge of output robot kinematics
        joint_limits = JointLimits()
        upper_limits = np.array(joint_limits_yaml["upper"])
        lower_limits = np.array(joint_limits_yaml["lower"])
        if(len(upper_limits) != len(lower_limits)):
            raise KeyError("upper joint limits and lower joint limits need to to be same size")
        joint_limits.upper = upper_limits
        joint_limits.lower = lower_limits
        return joint_limits

    def _get_str_name(self):
        return "Teleop controller [" + self.name + "]"

    def enable(self):
        self._wait_for_output_feedback_sub_msg(self.is_print_wait_msg)
        if len(self.__current_output_jps) == 0:
            total_size = 0
            for js_ci in self.js_control_interfaces:
                js_ci.obtain_size_and_index()
                total_size += js_ci.size
            self.__current_output_jps = np.zeros(total_size)
            self.__is_currrent_output_jps_initialized = True
            for js_ci in self.js_control_interfaces:
                js_ci.enable(self.joint_state_names)

    def disable(self):
        pass

    def clutch(self):
        pass

    def unclutch(self):
        self._wait_for_output_feedback_sub_msg(self.is_print_wait_msg)

    def _wait_for_output_feedback_sub_msg(self, always_print= False):
        for js_ci in self.js_control_interfaces:
            js_ci.wait_for_output_feedback_sub_msg(
                always_print= always_print, prepend_str=self._get_str_name())

    def _clamp_to_joint_limits(self, joint_positions):
        return np.clip(joint_positions, self.joint_limits.lower, self.joint_limits.upper)

    def _output_callback(self, joint_positions):
        harmonized_jp = get_harmonized_joint_positions(joint_positions, np.array(self.__current_output_jps))
        # TODO: These should be in dvrk_planning teleop controllers, but not easy as
        # of now due to its implementations
        if self.joint_limits is not None:
            harmonized_jp = self._clamp_to_joint_limits(harmonized_jp)
        for js_ci in self.js_control_interfaces:
            js_ci.publish(harmonized_jp)
            rospy.sleep(0.01) # Delay needed for downstream to synchronize properly
        # print("self.get_current_output_jps()\n", self.get_current_output_jps())
        # print("harmonized_jp send: \n", harmonized_jp)

    def wait_for_input_sub_msg(self, always_print=False):
        try:
            if always_print:
                 raise
            rospy.wait_for_message(self.input_topic, self.ros_input_type, timeout=0.1)
        except:
            print(self._get_str_name(), ": waiting for message from topic [" + self.input_topic +"]" )
            rospy.wait_for_message(self.input_topic, self.ros_input_type)
            print(self._get_str_name(), ": finished waiting for message from topic [" + self.input_topic +"]" )

    def _input_callback_impl(data):
        raise NotImplementedError

    def _input_callback_timed(self, data):
        self._average_timer.start()
        self._input_callback_impl(data)
        self._average_timer.stop()

    def get_str_statistics_report(self):
        if self._average_timer:
            return f"---[{self.name}]---\n" + "Input callback latency:\n" + self._average_timer.str_statistics_report()
        return None
