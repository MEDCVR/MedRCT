import dvrk_planning as dp

joint_pos = [0.0, 0.0, 0.1, 0.0, 0.0, 0.0]
print("Input joint pos: ", )

output_fk = dp.kinematics.psm.compute_fk(joint_pos)
print("FK: ", output_fk)

output_jp = dp.kinematics.psm.compute_ik(output_fk)
print("IK: ", output_jp)