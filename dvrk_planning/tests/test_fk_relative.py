from dvrk_planning.kinematics.psm import PsmKinematicsSolver, LND400006
from timer import Timer
import numpy as np

np.set_printoptions(precision = 4, suppress = True)

t = Timer(timer_repeat_times = 1)
p = PsmKinematicsSolver(LND400006())

joint_pos = [0.0, 0.0, 0.0, 0.0]
target_frame = "tool_tip_link"
reference_frame = "insertion_link"

output_fk = t.time_average(lambda : p.compute_fk_relative(joint_pos, reference_frame, target_frame))
print("FK: \n", output_fk)
print(t.str_average())
