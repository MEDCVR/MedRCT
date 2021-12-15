# dvrk_planning_ros


## Launch node and test

Launch the node: 

```bash
    roslaunch dvrk_planning_ros dvrk_planning.launch
```

Call the ik service: 

```bash
    rosservice call /psm/compute_ik "{tf_stamped:{transform:{translation:{x: 0.0, y: 0.0, z: 0.0}, rotation:{w: 1.0}}}}"
```
