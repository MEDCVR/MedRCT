#!/bin/bash

rostopic pub -1 /console/clutch sensor_msgs/Joy "header:
  seq: 0
  stamp: {secs: 0, nsecs: 0}
  frame_id: ''
buttons: [1]
"
