
This folder is a metapackage for dvrk planning algorithms.

List of Packages:

- dvrk_planning: Python package for dvrk kinematics computation (right now).
- dvrk_planning_msgs: Ros messages and services definitions for the dvrk_planning_ros node.
- dvrk_planning_ros: The ros node which exposes the dvrk_planning api as ros services/msgs. Build [instructions.](dvrk_planning_ros/README.md)
- cutting_app: A motion generation and shared control specifically for cutting with the dVRK and RTS 420007 tool. Build [instructions.](cutting_app/README.md)

# Clone

```bash
mkdir ~/dplan_ws/src
cd ~/dplan_ws/src
git clone https://mcsgitlab.utm.utoronto.ca/medcvr/dvrk_planning.git
```

# Build
```bash
cd ~/dplan_ws
catkin build dvrk_planning_ros
```
