#!/bin/bash

rostopic pub /joint_state sensor_msgs/JointState "header:
  seq: 0
  stamp: {secs: 0, nsecs: 0}
  frame_id: ''
name: ['first_joint']
position: [0]
velocity: [0]
effort: [0]"
