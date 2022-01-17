from enum import Enum

import numpy as np
# Should remove this if kinematics don't use
from PyKDL import Frame, Rotation, Vector

from dvrk_planning.kinematics.psm import PsmKinematicsSolver, LND40006
from dvrk_planning.utilities import convert_frame_to_mat

class OutputType(Enum):
    PSM = 0
    ECM = 1

class ControllerType(Enum):
    ERROR = 0
    FOLLOW = 1
    INCREMENT = 2

def get_rot_and_p(tf):
    rot = tf[0:3, 0:3]
    p = tf[0:3,3]
    return rot, p.flatten()

# This class automates the output_callback() into the update() function which is called everytime an
# input event happens.
# During an input event, it is also possible to change behaviour calling various API.
# Such as clutch/unclutch, enable/disable.
class TeleopController():
    # Suggest to only use output_to_camera_to, rather than using both with input_to_rot_adjustment
    # Try to align camera axis to the axis of your input changes.
    def __init__(self,
            controller_type,
            output_type = OutputType.PSM,
            output_to_camera_rot = Rotation.Quaternion(0, 0, 0, 1)):
            # input_to_rot_adjustment = Rotation.Quaternion(0, 0, 0, 1)): # Not doing input to rot adjustment now, as it gets confusing
        self.controller_type = controller_type
        self.output_to_camera_rot_tf = convert_frame_to_mat(Frame(output_to_camera_rot, Vector(0.0, 0.0, 0.0)))
        # self.input_rot_adjustment_tf = convert_frame_to_mat(Frame(input_to_rot_adjustment, Vector(0.0, 0.0, 0.0)))

        self.is_registered = False
        self.is_clutched = False
        self.is_enabled = False

        if (output_type == OutputType.PSM):
            self.kin_solver = PsmKinematicsSolver(LND40006())
        else:
            raise KeyError ('Right now only PSM')

    def register(self, output_callback):
        self.output_callback = output_callback
        self.is_registered = True

    def clutch(self):
        self.is_clutched = True

    def _unclutch(self):
        self.is_clutched = False

    # Enable/disable is for if you want your input to disable control of an output,
    # and input to enable to control another output
    # This way, input can also control various outputs simultaneously,
    # by creating multiple TeleopControllers with the same input.
    def _enable(self):
        self.is_enabled = True

    def disable(self):
        self.is_enabled = False

    def _rotation_adjustment(self, input_diff_tf):
        # rotated_input_tf = np.matmul(input_diff_tf, self.input_rot_adjustment_tf)
        rotated_output_tf = np.matmul(self.output_to_camera_rot_tf, input_diff_tf)

        return rotated_output_tf

    def _update_output_tf(self, input_diff_tf, output_tf):
        input_diff_tf_rotated = self._rotation_adjustment(input_diff_tf)

        input_diff_rot, input_diff_p = get_rot_and_p(input_diff_tf_rotated)
        cur_output_rot, cur_output_p = get_rot_and_p(output_tf)

        cur_output_p = cur_output_p + input_diff_p
        cur_output_rot = np.matmul(cur_output_rot, input_diff_rot)

        output_tf[0:3, 0:3] = cur_output_rot
        output_tf[0:3, 3] = cur_output_p

    ## Event driven by input frequency. Call this in your input loop/callback
    def update(self, *args):
        if(not self.is_registered):
            print("Hey, need to call register. I'm not updating")
            return False
        if(self.is_clutched):
            return True
        if(not self.is_enabled):
            return True

        absolute_output_tf = self._update_impl(args)
        output_js = self.kin_solver.compute_ik(absolute_output_tf)
        self.output_callback(output_js)
        return True

    def _update_impl(self, args):
        raise NotImplementedError

class FollowTeleopController(TeleopController):
    def __init__(self, output_type=OutputType.PSM, output_to_camera_rot=Rotation.Quaternion(0, 0, 0, 1)):
        super().__init__(ControllerType.FOLLOW, output_type, output_to_camera_rot)

    def enable(self, start_input_tf, start_output_tf):
        self.start_input_tf = np.copy(start_input_tf)
        self.start_output_tf = np.copy(start_output_tf)
        super()._enable()

    # When unclutching:
    # Position of output tf stays the same
    # MTM adjusts rotation for current output rotation. Need to check, but that wont work for occulus, or haptic pen.
    # start_output_tf should be the current tf of the output when uncluthing
    def unclutch(self, start_input_tf, start_output_tf):
        self.start_input_tf = np.copy(start_input_tf)
        self.start_output_tf = np.copy(start_output_tf)
        super()._unclutch()

    def __update_input_tf(self, absolute_input_tf):
        input_tf_difference = np.matmul(np.linalg.inv(self.start_input_tf), absolute_input_tf)
        absolute_output_tf = np.copy(self.start_output_tf)
        self._update_output_tf(input_tf_difference, absolute_output_tf)
        return absolute_output_tf

    def _update_impl(self, args):
        return self.__update_input_tf(*args)

class IncrementTeleopController(TeleopController):
    def __init__(self, output_type=OutputType.PSM, output_to_camera_rot=Rotation.Quaternion(0, 0, 0, 1)):
        super().__init__(ControllerType.INCREMENT, output_type, output_to_camera_rot)

    def enable(self, current_output_tf):
        self.current_output_tf = np.copy(current_output_tf)
        super()._enable()

    def unclutch(self):
        return super()._unclutch()

    def __increment_input_tf(self, inc_position: Vector, inc_quaternion: Rotation):
        input_diff_tf = convert_frame_to_mat(Frame(inc_quaternion, inc_position))
        self._update_output_tf(input_diff_tf, self.current_output_tf)
        return self.current_output_tf

    def _update_impl(self, args):
        return self.__increment_input_tf(*args)
