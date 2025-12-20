#!/usr/bin/env python3

import threading
import sys
import rclpy
from rclpy.node import Node
from geometry_msgs.msg import TwistStamped
from sensor_msgs.msg import JointState
import pygame
import argparse

msg = """
Reading from the keyboard  and Publishing to Twist!
---------------------------
i : +x
, : -x
j : +y
l : -y
t : up (+z)
b : down (-z)
I : +rot_x
< : -rot_x
J : +rot_y
L : -rot_y
T : +rot_z
B : -rot_z
------
[ : jaw close
] : jaw open

CTRL-C to quit
"""

class PublishTransformThread(threading.Thread):
    def __init__(self, node, rate):
        super().__init__()
        self.node = node
        self.publisher = node.create_publisher(TwistStamped, '/keyboard/twist', 1)
        self.x = self.y = self.z = 0.0
        self.rot_x = self.rot_y = self.rot_z = 0.0
        self.increment = 0.0
        self.rot_increment = 0.0
        self.condition = threading.Condition()
        self.done = False
        self.timeout = (1.0 / rate) if rate != 0.0 else None
        self.start()

    def update(self, x, y, z, rot_x, rot_y, rot_z, increment, rot_increment):
        with self.condition:
            self.x, self.y, self.z = x, y, z
            self.rot_x, self.rot_y, self.rot_z = rot_x, rot_y, rot_z
            self.increment = increment
            self.rot_increment = rot_increment
            self.condition.notify()

    def stop(self):
        with self.condition:
            self.done = True
            self.condition.notify()
        self.join()

    def run(self):
        twist = TwistStamped()
        while True:
            with self.condition:
                if self.done:
                    break
                self.condition.wait(self.timeout)
                twist.twist.linear.x = self.x * self.increment
                twist.twist.linear.y = self.y * self.increment
                twist.twist.linear.z = self.z * self.increment
                twist.twist.angular.x = self.rot_x * self.rot_increment
                twist.twist.angular.y = self.rot_y * self.rot_increment
                twist.twist.angular.z = self.rot_z * self.rot_increment
                twist.header.stamp = self.node.get_clock().now().to_msg()
            self.publisher.publish(twist)

class PublishJointStateThread(threading.Thread):
    def __init__(self, node, rate):
        super().__init__()
        self.node = node
        self.publisher = node.create_publisher(JointState, '/keyboard/joint_state', 1)
        self.jpos = 0.0
        self.increment = 0.0
        self.condition = threading.Condition()
        self.done = False
        self.timeout = (1.0 / rate) if rate != 0.0 else None
        self.start()

    def update(self, jpos, increment):
        with self.condition:
            self.jpos = jpos
            self.increment = increment
            self.condition.notify()

    def stop(self):
        with self.condition:
            self.done = True
            self.condition.notify()
        self.join()

    def run(self):
        js = JointState()
        js.name = ['jaw']
        while True:
            with self.condition:
                if self.done:
                    break
                self.condition.wait(self.timeout)
                js.position = [self.jpos * self.increment]
            js.header.stamp = self.node.get_clock().now().to_msg()
            self.publisher.publish(js)


def main(args=None):
    rclpy.init(args=args)
    argv = sys.argv
    # Parse arguments
    parser = argparse.ArgumentParser()
    parser.add_argument('-s', '--scale', type=float, required=False,
                        default = 1.0,
                        help = 'the package where you store the config yaml, default is "dvrk_planning_ros"')
    args = parser.parse_args(argv[1:]) # skip argv[0], script name


    node = Node('teleop_twist_keyboard_pygame')
    node.declare_parameter('increment', 0.001 * args.scale)
    node.declare_parameter('rot_increment', 0.05)
    node.declare_parameter('rate', 20)
    node.declare_parameter('repeat_rate', 0.0)

    increment = node.get_parameter('increment').value
    rot_increment = node.get_parameter('rot_increment').value
    rate = node.get_parameter('rate').value
    repeat = node.get_parameter('repeat_rate').value

    print(msg)
    print(f"increment: {increment} m, rate: {rate} Hz, speed if holding keys: {increment * rate} m/s")

    pub_tf_thread = PublishTransformThread(node, repeat)
    pub_js_thread = PublishJointStateThread(node, repeat)

    pygame.init()
    screen = pygame.display.set_mode((640, 200))
    pygame.display.set_caption('Click Here and Press Keys to Teleop')
    pygame.mouse.set_visible(True)
    clock = pygame.time.Clock()

    try:
        while rclpy.ok():
            clock.tick(rate)
            x = y = z = 0
            rot_x = rot_y = rot_z = 0
            jaw_inc = 0

            for event in pygame.event.get():
                if event.type == pygame.QUIT:
                    raise KeyboardInterrupt
                if event.type == pygame.KEYDOWN:
                    key_name = pygame.key.name(event.key)
                    print(f"Key pressed: {key_name}")
                    node.get_logger().info(f"Key pressed: {key_name}")

            keys = pygame.key.get_pressed()
            mods = pygame.key.get_mods()
            is_key_pressed = False

            # Translation when SHIFT not held
            if not (mods & pygame.KMOD_SHIFT):
                if keys[pygame.K_i]: x, is_key_pressed = 1, True
                if keys[pygame.K_COMMA]: x, is_key_pressed = -1, True
                if keys[pygame.K_j]: y, is_key_pressed = 1, True
                if keys[pygame.K_l]: y, is_key_pressed = -1, True
                if keys[pygame.K_t]: z, is_key_pressed = 1, True
                if keys[pygame.K_b]: z, is_key_pressed = -1, True
            # Rotation when SHIFT held
            else:
                if keys[pygame.K_i]: rot_x, is_key_pressed = 1, True
                if keys[pygame.K_COMMA]: rot_x, is_key_pressed = -1, True
                if keys[pygame.K_j]: rot_y, is_key_pressed = 1, True
                if keys[pygame.K_l]: rot_y, is_key_pressed = -1, True
                if keys[pygame.K_t]: rot_z, is_key_pressed = 1, True
                if keys[pygame.K_b]: rot_z, is_key_pressed = -1, True

            # Jaw control
            if keys[pygame.K_RIGHTBRACKET]: jaw_inc, is_key_pressed = 1, True
            if keys[pygame.K_LEFTBRACKET]: jaw_inc, is_key_pressed = -1, True

            if is_key_pressed:
                pub_tf_thread.update(x, y, z, rot_x, rot_y, rot_z, increment, rot_increment)
                pub_js_thread.update(jaw_inc, rot_increment)

            rclpy.spin_once(node, timeout_sec=0)
    except KeyboardInterrupt:
        pass
    finally:
        pub_tf_thread.stop()
        pub_js_thread.stop()
        node.destroy_node()
        rclpy.shutdown()

if __name__ == '__main__':
    main()
