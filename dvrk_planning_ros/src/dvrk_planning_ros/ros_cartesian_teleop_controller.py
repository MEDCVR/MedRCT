import rospy
import tf
import yaml

from geometry_msgs.msg import TransformStamped, Twist
from sensor_msgs.msg import JointState

from PyKDL import Rotation, Vector

from dvrk_planning.controller.cartesian_teleop_controller import CartesianFollowTeleopController, CartesianIncrementTeleopController, InputType
from dvrk_planning_ros.utils import gm_tf_to_numpy_mat
from dvrk_planning_ros.ros_teleop_controller import RosTeleopController

def quat_yaml_to_pykdl(quaternion_yaml):
    x = quaternion_yaml["x"]
    y = quaternion_yaml["y"]
    z = quaternion_yaml["z"]
    w = quaternion_yaml["w"]

    return Rotation.Quaternion(x, y, z, w)

def output_ref_to_input_rot_from_yaml(output_ref_to_input_rot_yaml):
    if "quaternion" in output_ref_to_input_rot_yaml:
        return quat_yaml_to_pykdl(output_ref_to_input_rot_yaml["quaternion"])
    elif "lookup_tf" in output_ref_to_input_rot_yaml:
        lookup_tf_yaml = output_ref_to_input_rot_yaml["lookup_tf"]
        t = tf.TransformListener()
        rospy.sleep(1) # Sleep is needed so TransformListener has time to get the tf's
        _, quat_rot  = t.lookupTransform(lookup_tf_yaml["camera_tf"],lookup_tf_yaml["base_tf"], rospy.Time(0))
        return Rotation.Quaternion(quat_rot[0], quat_rot[1], quat_rot[2], quat_rot[3])
    else:
        raise KeyError ("output_to_camera in yaml looking for quaternion or lookup_tf")

class RosCartesiansTeleopController(RosTeleopController):
    def __init__(self, controller_yaml, kinematics_solver):
        super().__init__(controller_yaml)
        output_yaml = controller_yaml["output"]
        self.js_msg.name = kinematics_solver.get_active_joint_names()

        output_ref_to_input_rot = Rotation.Quaternion(0, 0, 0, 1)
        if("output_ref_to_input_rot" in output_yaml):
            output_ref_to_input_rot = output_ref_to_input_rot_from_yaml(output_yaml["output_ref_to_input_rot"])
        input_yaml = controller_yaml["input"]
        self.input_topic = input_yaml["topic"]
        if input_yaml["type"] == "follow":
            self._teleop_controller = CartesianFollowTeleopController(kinematics_solver, output_ref_to_input_rot = output_ref_to_input_rot)
            self.input_topic_type = TransformStamped
            sub_callback = self._input_callback_tf
        elif input_yaml["type"] == "increment":
            self._teleop_controller = CartesianIncrementTeleopController(kinematics_solver, output_ref_to_input_rot = output_ref_to_input_rot)
            self.input_topic_type = Twist
            sub_callback = self._input_callback_twist
        else:
            raise KeyError ("controller: type: must be follow or increment")
        self.input_sub = rospy.Subscriber(self.input_topic, self.input_topic_type, sub_callback)

        self._teleop_controller.register(self._output_callback)

    def _wait_for_input_sub_msg(self):
        if self._teleop_controller.input_type == InputType.FOLLOW:
            print(self._get_str_name(), ": waiting for message from topic [" + self.input_topic +"]" )
            rospy.wait_for_message(self.input_topic, self.input_topic_type)
            print(self._get_str_name(), ": finised waiting for message from topic [" + self.input_topic +"]" )

    def enable(self):
        self._wait_for_output_feedback_sub_msg()
        # TODO, this is not good oop
        if self._teleop_controller.controller_type == InputType.INCREMENT:
            self._teleop_controller.enable(self.current_output_tf)
        elif self._teleop_controller.controller_type == InputType.FOLLOW:
            self._wait_for_input_sub_msg()
            self._teleop_controller.enable(self.current_input_tf, self.current_output_tf)

    def disable(self):
        self._teleop_controller.disable()

    def clutch(self):
        self._teleop_controller.clutch()

    def unclutch(self):
        # TODO, this is not good oop
        if self._teleop_controller.controller_type == InputType.INCREMENT:
            self._teleop_controller.unclutch()
        elif self._teleop_controller.controller_type == InputType.FOLLOW:
            self._wait_for_input_sub_msg()
            self._wait_for_output_feedback_sub_msg()
            self._teleop_controller.unclutch(self.current_input_tf, self.current_output_tf)

    # Output feedback needs a tf, but js is the simplest type of
    # data for a robot controller, so lets start with that.
    def _output_feedback_callback(self, js):
        self.current_output_tf = self._teleop_controller.kinematics_solver.compute_fk(js)

    def _input_callback_tf(self, data):
        self.current_input_tf = gm_tf_to_numpy_mat(data.transform)
        self._teleop_controller.update(self.current_input_tf)

    def _input_callback_twist(self, data):
        self._teleop_controller.update(
            Vector(data.linear.x, data.linear.y, data.linear.z),
            Rotation.RPY(data.angular.x, data.angular.y, data.angular.z))
