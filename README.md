# dvrk_planning

This repo is a metapackage for dvrk planning algorithms.

List of Packages: 

- dvrk_planning: Python package for dvrk kinematics computation (right now).
- dvrk_planning_msgs: Ros messages and services definitions for the dvrk_planning_ros node.
- dvrk_planning_ros: The ros node which exposes the dvrk_planning api as ros services/msgs.


# MEDRCT

Install colcon:

```
curl -s https://packagecloud.io/install/repositories/dirk-thomas/colcon/script.deb.sh | sudo bash
sudo apt install python3-colcon-common-extensions
```

Install clang and spdlog:

```
sudo apt install clang-format-7
sudo apt install libspdlog-dev
```


## Build and run the example

Install ROS (TODO link)

```
mkdir ~/medrct_ws
git clone TODO_MEDRCT_LINK
```

### Build

Build medrct with ROS, dVRK kinematics, and default controllers.

```
source /opt/ros/noetic/setup.bash
cd ~/medrct_ws
colcon build --packages-up-to medrct_ros_app medrct_dvrk_env medrct_default_controller
```

Clone AMBF and build:
```
cd ~/medrct_ws
git clone https://github.com/WPI-AIM/ambf
cd ambf
mkdir build && cd build
cmake ..
make
```

### Run
Each step is a new terminal
1. Start `roscore`

2. Start AMBF PSM output modality:
```
cd ~/medrct_ws/ambf/build
source devel/setup.bash
cd ../bin/lin-x86_64/
./ambf_simulator -l 5
```

3. Start AMBF CRTK interface:
```
source ~/medrct_ws/ambf/build/devel/setup.bash
cd ~/medrct_ws/dvrk_planning/dvrk_python_v1/ambf_ros_crtk/scripts
python3 ambf_crtk_translator.py
```
4. Start the teleoperation node:
```
source ~/medrct_ws/install/setup.bash
cd ~/medrct_ws/build/medrct_ros_app
./ros_teleop_node medrct_ros_app/config/increment_example.yaml
```

5. Start the keyboard input device ros publisher:
```
source /opt/ros/noetic/setup.bash
cd ~/medrct_ws/dvrk_planning/dvrk_python_v1/dvrk_planning_ros/scripts
python3 psm_teleop_keyboard.py
```

## Issues with KDL and Eigen

```
sudo ln -s /usr/include/eigen3/Eigen /usr/include/Eigen
```
