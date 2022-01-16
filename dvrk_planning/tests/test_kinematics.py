import dvrk_planning as dp
from timer import Timer

t = Timer()

joint_pos = [0.0, 0.0, 0.1, 0.0, 0.0, 0.0]
print("Input joint pos: \n", joint_pos)

output_fk = t.time_average(lambda : dp.kinematics.psm.compute_fk(joint_pos))
print("FK: \n", output_fk)
print(t.str_average())

output_jp = t.time_average(lambda : dp.kinematics.psm.compute_ik(output_fk))
print("IK: \n", output_jp)
print(t.str_average())

t.start()
output_jp = dp.kinematics.psm.compute_ik(output_fk)
t.stop()
print(t.str_elapsed_time())
