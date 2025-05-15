# MEDRCT

## Dependencies

Install dependencies:
```bash
sudo apt install libspdlog-dev libeigen3-dev liburdfdom-dev
```

Install colcon:

```bash
curl -s https://packagecloud.io/install/repositories/dirk-thomas/colcon/script.deb.sh | sudo bash
sudo apt install python3-colcon-common-extensions
```

## Build and run the example

Install ROS: http://wiki.ros.org/noetic/Installation/Ubuntu

```bash
mkdir ~/medrct_ws && cd medrct_ws
git clone https://mcsgitlab.utm.utoronto.ca/medcvr/dvrk_planning
```

### Build

Build medrct with ROS, dVRK kinematics, and default controllers.

```bash
source /opt/ros/noetic/setup.bash
cd ~/medrct_ws
colcon build --packages-up-to medrct_ros_app medrct_dvrk_env medrct_default_controller
```

Clone AMBF and build:
```bash
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

## Start a simulator
2. Start AMBF PSM output modality:
```bash
cd ~/medrct_ws/ambf/build
source devel/setup.bash
cd ../bin/lin-x86_64/
./ambf_simulator -l 5
```

3. Start AMBF CRTK interface:
```bash
source ~/medrct_ws/ambf/build/devel/setup.bash
cd ~/medrct_ws/dvrk_planning/dvrk_python_v1/ambf_ros_crtk/scripts
python3 ambf_crtk_translator.py
```
## Start a dvrk robot instead


3. 
```bash
roscd dvrk_config
rosrun dvrk_robot dvrk_console_json -j <specific_group>/console-MTMR-PSM2.json
```

## Teleoperation


4. Start the teleoperation node:
```bash
source ~/medrct_ws/install/setup.bash
roslaunch medrct_ros_app ros_teleop.launch config:=config/increment_example.yaml
```

5. Start the keyboard input device ros publisher:
```bash
source /opt/ros/noetic/setup.bash
cd ~/medrct_ws/dvrk_planning/dvrk_python_v1/dvrk_planning_ros/scripts
python3 psm_teleop_keyboard.py
```

## Issues with KDL and Eigen

```bash
sudo ln -s /usr/include/eigen3/Eigen /usr/include/Eigen
```

## For Developers

Install clang and spdlog:

```
sudo apt install clang-format-7
```

# dvrk_planning Instructions

Old and depcrecated dvrk_planning [instructions.](dvrk_python_v1/README.md)

# ROS2

build:
```bash
colcon build --packages-up-to medrct_ros2_app medrct_default_controller medrct_dvrk_env                             
```

Start dvrk console: refer to medcvr dvrk wiki

run:
```bash
ros2 run medrct_ros2_app ros2_teleop_node --ros-args -p config:=config/increment_example.yaml
```

Start keyboard:
```
    cd <dvrk_planning_folder>/ros2/medrct_ros2_app/scripts 
    python3 ros2_psm_buffered_teleop_keyboard.py 
```

## ROS2 Franka

build:
```bash
colcon build --packages-up-to medrct_franka_env medrct_default_controller medrct_ros2_app
```