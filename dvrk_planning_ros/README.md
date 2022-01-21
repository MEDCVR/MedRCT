# dvrk_planning_ros


## Build
Make workspace so folder structure is <your_workspace>/src/dvrk_planning

```
cd <your_workspace>
catkin build
source devel/setup.bash
```

## Launch dvrk_planning_node

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
  position: [0.0, -0.0, 0.093, 0.0, -0.0, 0.0]
  velocity: []
  effort: []
```

## Launch dvrk_teleop_node

```
rosrun dvrk_planning_ros dvrk_teleop_node.py
```

The default arguments tries to find the yaml file in `dvrk_planning_ros` package, with a relative path `/config/teleop.yaml`.
To specify another config file path, and/or another package where the config is relative to:

```
rosrun dvrk_planning_ros dvrk_teleop_node.py -p <another_package> -y <relative_path_to_yaml>
```
