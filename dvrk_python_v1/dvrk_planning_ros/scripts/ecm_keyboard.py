#!/usr/bin/env python

import threading
import rospy
import numpy as np

from geometry_msgs.msg import TwistStamped
from sensor_msgs.msg import JointState

import pygame

# sudo apt install python3-pygame

class PublishJointStateThread(threading.Thread):
    def __init__(self, rate):
        super(PublishJointStateThread, self).__init__()
        self.publisher = rospy.Publisher('/ECM/servo_jp', JointState, queue_size = 1)
        self.subscriber = rospy.Subscriber('/ECM/measured_js', JointState, self.save_measured_js)
        self.jpos_increment = np.array([0.0, 0.0, 0.0, 0.0])
        self.measured_js = np.array([0.0, 0.0, 0.0, 0.0])
        self.condition = threading.Condition()
        self.done = False

        # Set timeout to None if rate is 0 (causes new_message to wait forever
        # for new data to publish)
        if rate != 0.0:
            self.timeout = 1.0 / rate
        else:
            self.timeout = None

        rospy.wait_for_message("/ECM/measured_js",JointState, timeout=2)
        self.command_js = np.copy(self.measured_js)

        self.start()

    def save_measured_js(self, js):
        self.measured_js = js.position

    def update(self, jpos_increment):
        assert(jpos_increment.shape[0] == 4)
        assert(jpos_increment.shape == self.command_js.shape)
        self.condition.acquire()
        self.command_js = + self.command_js + jpos_increment
        # Notify publish thread that we have a new message.
        self.condition.notify()
        self.condition.release()

    def stop(self):
        self.condition.acquire()
        self.condition.notify()
        self.condition.release()

        self.done = True
        self.join()

    def run(self):
        js_msg = JointState()
        js_msg.name = ["outer_yaw", "outer_pitch", "insertion", "outer_roll"]
        while not self.done:
            self.condition.acquire()
            # Wait for a new message or timeout.
            self.condition.wait(self.timeout)
            self.condition.release()

            js_msg.header.stamp = rospy.Time.now()
            js_msg.position = self.command_js
            # Publish.
            self.publisher.publish(js_msg)


msg = """
Reading from the keyboard  and Publishing to Twist!
---------------------------
d : +yaw
a : -yaw
w : +pitch
s : -pitch
i : +insert
k : -down (-z)
j : +roll
l : -roll

CTRL-C to quit
"""

if __name__=="__main__":
    rospy.init_node('teleop_twist_keyboard')
    print(msg)

    rate = rospy.get_param("~rate", 20) # Hz
    repeat = rospy.get_param("~repeat_rate", 0.0)

    print("rate: ", rate, "Hz")

    pub_js_thread = PublishJointStateThread(repeat)

    pygame.init()
    screen = pygame.display.set_mode((640, 200))
    pygame.display.set_caption('Click Here and Press Keys to Teleop')
    pygame.mouse.set_visible(1)

    angle_speed = 0.01      # rad/s
    prismatic_speed = 0.001 # mm/s
    theta_speeds = np.array([angle_speed, angle_speed, prismatic_speed, angle_speed])
    clock = pygame.time.Clock()
    while not rospy.is_shutdown():
        clock.tick(rate)
        theta_increments = np.array([0.0, 0.0, 0.0, 0.0])

        for event in pygame.event.get():
            if event.type == pygame.QUIT:
                run = False

        keys = pygame.key.get_pressed()
        is_key_pressed = False
        # Linear
        if keys[pygame.K_w]:
            theta_increments[1] =  -theta_speeds[1]
            is_key_pressed = True
        if keys[pygame.K_s]:
            theta_increments[1] = theta_speeds[1]
            is_key_pressed = True
        if keys[pygame.K_d]:
            theta_increments[0] =  theta_speeds[0]
            is_key_pressed = True
        if keys[pygame.K_a]:
            theta_increments[0] = -theta_speeds[0]
            is_key_pressed = True
        if keys[pygame.K_i]:
            theta_increments[2] =  theta_speeds[2]
            is_key_pressed = True
        if keys[pygame.K_k]:
            theta_increments[2] =  -theta_speeds[2]
            is_key_pressed = True
        if keys[pygame.K_l]:
            theta_increments[3] =  theta_speeds[3]
            is_key_pressed = True
        if keys[pygame.K_j]:
            theta_increments[3] =  -theta_speeds[3]
            is_key_pressed = True

        if(is_key_pressed):
            # print("x: {}, y: {}, z: {}".format(x, y, z))
            # print("rot_x: {}, rot_y: {}, rot_z: {}".format(rot_x, rot_y, rot_z))
            # print("jaw_inc: {}".format(jaw_inc))
            pub_js_thread.update(theta_increments)

    pub_js_thread.stop()
