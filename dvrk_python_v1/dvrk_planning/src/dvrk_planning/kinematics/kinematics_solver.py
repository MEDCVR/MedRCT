import numpy as np
from abc import ABC, abstractmethod

class KinematicsSolver(ABC):
    @abstractmethod
    def compute_fk(self, joint_positions: np.ndarray) -> np.ndarray(float):
        """_summary_
        Args:
            joint_positions (np.ndarray(float)): 1D ndarray of joint positions

        Returns:
            np.ndarray(float): 2D ndarray which represents a 4x4 transform matrix
        """
        pass
    @abstractmethod
    def compute_ik(self, T_tip_0_mat: np.ndarray, current_joint_positions: np.ndarray, ee_metadata: tuple = ()) -> np.ndarray(float):
        """_summary_
        Args:
            T_tip_0_mat (np.ndarray(float)): 2D ndarray which represents a 4x4 transform matrix
            current_joint_positions (np.ndarray(float)): 1D ndarray of joint positions
            ee_metadata (tuple): End effector informaton to solve ik simultaneously. Should
            only be used if the output robot controller controls ee_states together with it's other joints.
        Returns:
            (np.ndarray(float)): 1D ndarray of joint positions
        """
        pass

    # Uncomment when proper use case to use yet
    # def joint_to_actuator(self, joint_positions: np.ndarray) -> np.ndarray(float):
    #     """ (Optional) If your output to the robot interface is something other than joint positions.
    #     For example actuator positions or dial positions.
    #     Args:
    #         joint_positions (np.ndarray(float)): 1D ndarray of joint positions

    #     Returns:
    #         np.ndarray(float): 1D ndarray of actuator (or other) positions
    #     """
    #     return joint_positions

    def actuator_to_joint(self, actuator_positions: np.ndarray) -> np.ndarray(float):
        """(Optional) If your output to the robot interface is something other than joint positions.
        For example actuator positions or dial positions.

        Args:
            actuator_positions (np.ndarray(float)): 1D ndarray of actuator (or other) positions

        Returns:
            np.ndarray(float): 1D ndarray of joint positions
        """
        return actuator_positions

    # These two below belong in the robot description class later
    @abstractmethod
    def get_active_joint_names(self) -> list(str):
        pass
    @abstractmethod
    def get_link_names(self) -> list(str):
        pass
