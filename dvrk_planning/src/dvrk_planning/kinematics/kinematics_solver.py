from abc import ABC, abstractmethod

class KinematicsSolver(ABC):
    @abstractmethod
    def compute_fk(self, joint_positions):
        pass
    @abstractmethod
    def compute_ik(self, T_tip_0_mat):
        pass
