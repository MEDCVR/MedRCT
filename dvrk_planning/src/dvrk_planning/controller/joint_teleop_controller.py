import numpy as np
from dvrk_planning.controller.teleop_controller import TeleopController, InputType, ControllerType

def get_rot_and_p(tf):
    rot = tf[0:3, 0:3]
    p = tf[0:3,3]
    return rot, p.flatten()

class JointTeleopController(TeleopController):
    # Suggest to only use output_to_camera_to, rather than using both with input_to_rot_adjustment
    # Try to align camera axis to the axis of your input changes.
    def __init__(self,
            input_type):
        super().__init__(input_type, ControllerType.JOINT)

    ## Event driven by input frequency. Call this in your input loop/callback
    def update(self, *args):
        if(not self._update()):
            return False

        output_js = self._update_impl(args)
        self.output_callback(output_js)
        return True

    def _update_impl(self, args):
        raise NotImplementedError

class JointFollowTeleopController(JointTeleopController):
    def __init__(self, scale = 1.0):
        super().__init__(InputType.FOLLOW)
        self.scale = scale

    def enable(self, start_input_js, start_output_js):
        # Should move start_output_js slowly to match
        super()._enable()

    def unclutch(self, start_input_js, start_output_js):
        # Should move start_output_js slowly to match
        super()._unclutch()

    """
    Parameters
    ----------
    absolute_input_js : ndarray
        1D array containing data with `float` type.
    """
    def __update_input_js(self, absolute_input_js):
        absolute_output_js = self.scale * absolute_input_js
        return absolute_output_js

    def _update_impl(self, args):
        return self.__update_input_js(*args)

class JointIncrementTeleopController(JointTeleopController):
    def __init__(self):
        super().__init__(InputType.INCREMENT)
        self.current_output_js = np.array([])

    def enable(self, current_output_js):
        self.current_output_js = np.copy(current_output_js)
        super()._enable()

    def unclutch(self):
        return super()._unclutch()

    """
    Parameters
    ----------
    increment_js : ndarray
        1D array containing data with `float` type. Must have same length as self.current_output_js from enable
    """
    def __increment_input_js(self, increment_js):
        if(len(increment_js) != len(self.current_output_js)):
            print("Length input js not the same as current_output_js")
            return self.current_output_js
        self.current_output_js = self.current_output_js + increment_js
        return self.current_output_js

    def _update_impl(self, args):
        return self.__increment_input_js(*args)
