from abc import ABC, abstractmethod
import numpy as np

class KinematicsSolver(ABC):
    @abstractmethod
    def compute_fk(self, joint_positions):
        pass
    @abstractmethod
    def compute_ik(self, T_tip_0_mat):
        pass

    # These two below belong in the robot description class later
    @abstractmethod
    def get_active_joint_names(self):
        pass
    @abstractmethod
    def get_link_names(self):
        pass

    def compute_relative_fk(self, joint_positions, reference_link, target_link):
        up_to_link = self.get_link_names().index(reference_link) + 1
        T_0_n = self.compute_fk(joint_positions) # TODO target link
        if (up_to_link == self.links_num):
            T_l_n = np.identity(4)
            T_l_n[0:3, 0:3] = T_0_n[0:3, 0:3]
        else:
            T_0_l = self.compute_fk(joint_positions, up_to_link)
            T_l_n = np.matmul(np.linalg.inv(T_0_l), T_0_n)

        return T_l_n
