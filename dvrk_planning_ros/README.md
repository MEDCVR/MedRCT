# dvrk_planning_ros


## Launch node and test

Launch the node:

```bash
    roslaunch dvrk_planning_ros dvrk_planning.launch
```

Call the ik service:

```bash
    rosservice call /psm/compute_ik "{tf_stamped:{transform:{translation:{x: 0.0, y: 0.0, z: -0.09670}, rotation:{x: 0.7071068, y: 0.7071068, z: 0.0, w: 0.0}}}}"
```

Output should be:

```bash
joint_state: 
  header: 
    seq: 0
    stamp: 
      secs: 0
      nsecs:         0
    frame_id: ''
  name: 
    - outer_yaw
    - outer_pitch
    - outer_insertion
    - outer_roll
    - outer_wrist_pitch
    - outer_wrist_yaw
  position: [0.0, -0.0, 0.1, 0.0, -0.0, 0.0]
  velocity: []
  effort: []
```
