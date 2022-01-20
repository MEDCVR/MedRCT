import numpy as np

TWO_PI = 2 * np.pi

# Must be same length, not going to check here
def harmonize_joint_positions(joint_positions, reference_jps):
    new_joint_positions = np.copy(joint_positions)
    abs_joint_diffs = reference_jps - new_joint_positions
    sign_diffs = np.sign(reference_jps - new_joint_positions)
    shifted_two_pi_jps = sign_diffs * TWO_PI + new_joint_positions 
