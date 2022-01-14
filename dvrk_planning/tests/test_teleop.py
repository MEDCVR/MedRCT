from subprocess import call
from PyKDL import Rotation, Vector
import numpy as np

from dvrk_planning.controller.teleop_controller import FollowTeleopController, IncrementTeleopController
from dvrk_planning.kinematics.psm import compute_fk

np.set_printoptions(precision=3)
np.set_printoptions(suppress=True)

def callback_function(js):
    print("output joint state to send: \n", js)
    print("output transform result: \n", compute_fk(js))

joint_pos = [0.0, 0.0, 0.1, 0.0, 0.0, 0.0]
output_start_tf = compute_fk(joint_pos)
inc_x = 0.01

def test_itc():
    itc = IncrementTeleopController()
    itc.register(callback_function)

    print("output start transform: \n", output_start_tf)
    itc.enable(output_start_tf)

    print("increment step: \n", inc_x)
    print("Incrementing x --- \n")
    for _ in range(5):
        itc.update(Vector(inc_x, 0.0, 0.0), Rotation.Quaternion(0.0, 0.0, 0.0, 1.0))

    print("Incrementing rot x --- \n")
    for _ in range(5):
        itc.update(Vector(0.0, 0.0, 0.0), Rotation.RPY(0.1, 0.0, 0.0))

def test_ftc():
    ftc = FollowTeleopController()
    ftc.register(callback_function)
    print("output start transform: \n", output_start_tf)

    input_start_tf = np.identity(4)
    print("input start transform: \n", input_start_tf)

    ftc.enable(input_start_tf, output_start_tf)
    print("increment step: \n", inc_x)

    print("Incrementing x --- \n")
    for _ in range(5):
        input_start_tf[0,3] = input_start_tf[0,3] + inc_x
        ftc.update(input_start_tf)

print("--- Test Increment Teleop Controller --- \n")
test_itc()

print("\n")
print("--- Test Follower Teleop Controller --- \n")
test_ftc()
