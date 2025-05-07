#!/usr/bin/env python3

import threading
import sys
import rclpy
from rclpy.node import Node
from geometry_msgs.msg import TwistStamped
from sensor_msgs.msg import JointState
import pygame

msg = """
Reading from the keyboard  and Publishing to Twist!
---------------------------
d : +x
a : -x
w : +y
s : -y
e : up (+z)
q : down (-z)
D : +rot_x
A : -rot_x
W : +rot_y
S : -rot_y
E : +rot_z
Q : -rot_z
------
o : jaw close
p : jaw open

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
    node = Node('teleop_twist_keyboard_pygame')
    node.declare_parameter('increment', 0.0005)
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
                if keys[pygame.K_d]: x, is_key_pressed = 1, True
                if keys[pygame.K_a]: x, is_key_pressed = -1, True
                if keys[pygame.K_w]: y, is_key_pressed = 1, True
                if keys[pygame.K_s]: y, is_key_pressed = -1, True
                if keys[pygame.K_e]: z, is_key_pressed = 1, True
                if keys[pygame.K_q]: z, is_key_pressed = -1, True
            # Rotation when SHIFT held
            else:
                if keys[pygame.K_d]: rot_x, is_key_pressed = 1, True
                if keys[pygame.K_a]: rot_x, is_key_pressed = -1, True
                if keys[pygame.K_w]: rot_y, is_key_pressed = 1, True
                if keys[pygame.K_s]: rot_y, is_key_pressed = -1, True
                if keys[pygame.K_e]: rot_z, is_key_pressed = 1, True
                if keys[pygame.K_q]: rot_z, is_key_pressed = -1, True

            # Jaw control
            if keys[pygame.K_p]: jaw_inc, is_key_pressed = 1, True
            if keys[pygame.K_o]: jaw_inc, is_key_pressed = -1, True

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
