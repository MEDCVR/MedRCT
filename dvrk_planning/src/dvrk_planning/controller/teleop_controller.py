import numpy as np
import dvrk_planning as dp
from  dvrk_planning.utilities import convert_frame_to_mat

# Should remove this if kinematics don't use
from PyKDL import Frame, Rotation, Vector

# Should be event driven by the input loop()
class TeleopController():
    # Suggest to only use output_to_camera_to, rather than using both with input_to_rot_adjustment
    # Try to align camera axis to the axis of your input changes.
    def __init__(self, output_to_camera_rot = Rotation.Quaternion(0, 0, 0, 1), input_to_rot_adjustment = Rotation.Quaternion(0, 0, 0, 1)):
        self.output_to_camera_rot_tf = convert_frame_to_mat(Frame(output_to_camera_rot), Vector(0.0, 0.0, 0.0))
        self.input_rot_adjustment_tf = convert_frame_to_mat(Frame(input_to_rot_adjustment), Vector(0.0, 0.0, 0.0))

        self.is_registered = False
        self.is_clutched = False
        self.is_enabled = False

    def rotation_adjustment(self, input_diff_tf):
        rotated_output_tf = np.matmul(input_diff_tf, self.input_rot_adjustment_tf)
        rotated_output_tf = np.matmul(self.output_to_camera_rot_tf, rotated_output_tf)

        return rotated_output_tf

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
        self.is_disabled = False

    ## Event driven by input frequency. Call this in your input loop/callback
    def update(self, args):
        if(not self.is_registered):
            print("Hey, need to call register. I'm not updating")
            return False
        if(self.is_clutched):
            return True
        if(not self.is_enabled):
            return True

        self.output_callback(self._update_impl(args))
        return True

    def _update_impl(self, args):
        raise NotImplementedError

class FollowTeleopController(TeleopController):
    def enable(self, start_input_tf, start_psm_tf):
        self.start_input_tf = start_input_tf
        self.start_psm_tf = start_psm_tf
        super()._enable()

    ## When unclutching:
    ## Position of output tf stays the same
    ## Rotation of output tf always follows input, psm will update rotation after unclutch
    def unclutch(self, start_input_tf, start_psm_tf):
        self.start_input_tf = start_input_tf
        self.start_psm_tf = start_psm_tf
        super()._unclutch()

    def __update_input_tf(self, absolute_input_tf):

        input_tf_difference = np.matmul(np.linalg.inv(self.start_input_tf), absolute_input_tf)
        input_tf_diff_rotated = self.rotation_adjustment(input_tf_difference)
        absolute_psm_tf = np.matmul(self.start_psm_tf, input_tf_diff_rotated)
        output_js = dp.kinematics.psm.compute_ik(absolute_psm_tf)
        return output_js

    def _update_impl(self, args):
        return self.__update_input_tf(**args)

def to_tf(pos, rot):
    tf = np.identity(4)
    return tf

class IncrementTeleopController(TeleopController):
    def enable(self, current_psm_tf):
        self.current_psm_tf = current_psm_tf
        super()._enable()

    def unclutch(self):
        return super()._unclutch()

    def __increment_input_tf(self, inc_position, inc_quaternion):
        input_diff_tf = convert_frame_to_mat(Frame(inc_quaternion, inc_position))
        self.current_psm_tf = self.rotation_adjustment(input_diff_tf) + self.current_psm_tf

        output_js = dp.kinematics.psm.compute_ik(self.current_psm_tf)
        return output_js

    def _update_impl(self, args):
        return self.__increment_input_tf(**args)
