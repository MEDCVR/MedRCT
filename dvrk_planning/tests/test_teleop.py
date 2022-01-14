from PyKDL import Rotation, Vector
import numpy as np

from dvrk_planning.controller.teleop_controller import FollowTeleopController, IncrementTeleopController
from dvrk_planning.kinematics.psm import compute_fk, compute_ik

np.set_printoptions(precision=3)
np.set_printoptions(suppress=True)

def callback_function(js):
    print("output joint state to send: \n", js)
    print("output transform result: \n", compute_fk(js))

def test_itc():
    itc = IncrementTeleopController()
    itc.register(callback_function)

    joint_pos = [0.0, 0.0, 0.1, 0.0, 0.0, 0.0]
    output_start_tf = compute_fk(joint_pos)
    print("output start transform: \n", output_start_tf)
    itc.enable(output_start_tf)

    inc_x = 0.01
    print("increment step x: \n", inc_x)

    for i in range(10):
        itc.update(Vector(inc_x, 0.0, 0.0), Rotation.Quaternion(0.0, 0.0, 0.0, 1.0))

print("--- Test Increment Teleop Controller --- \n")
test_itc()

print("\n")
print("--- Test Follower Teleop Controller --- \n")
