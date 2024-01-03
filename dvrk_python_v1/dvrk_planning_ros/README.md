# dvrk_planning_ros

## Dependencies

Install ROS: http://wiki.ros.org/noetic/Installation/Ubuntu

Install catkin tools https://catkin-tools.readthedocs.io/en/latest/installing.html

## Build
Make workspace so folder structure is <your_workspace>/src/dvrk_planning

source ros:

```bash
source /opt/ros/noetic/setup.bash
```

build:

```bash
cd <your_workspace>
catkin build dvrk_planning_ros
source devel/setup.bash
```

## Launch dvrk_teleop_node

Launch the default keyboard and PSM2 teleop:

```bash
rosrun dvrk_planning_ros dvrk_teleop_node.py -y config/keyboard_psm2.yaml
```

Then in another terminal, run the keyboard node:
```bash
rosrun dvrk_planning_ros psm_teleop_keyboard.py
```

Launch the MTMR PSM2 teleop:

```bash
rosrun dvrk_planning_ros dvrk_teleop_node.py  -y config/mtmr_psm2.yaml
# or moving in camera frame
rosrun dvrk_planning_ros dvrk_teleop_node.py  -y config/mtmr_psm2_cam.yaml
```
Hold MTM, then press MONO pedal on console to enable. NOTE!! that MTM falls

To specify another config file path, and/or another package where the config is relative to:
```bash
rosrun dvrk_planning_ros dvrk_teleop_node.py -p <another_package> -y <relative_path_to_yaml>
```
