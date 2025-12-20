# ROS2 Build

Install ROS2 for your specific ubuntu version.

```bash
mkdir ~/medrct_ws && cd medrct_ws
git clone https://github.com/MEDCVR/MedRCT
```

Dependencies
```bash
sh ~/medrct_ws/MedRCT/install_deps.sh
```

## DVRK Build 

Build:
```bash
source /opt/ros/<version>/setup.bash
cd ~/medrct_ws
colcon build --packages-up-to medrct_ros2_app medrct_default_controller medrct_dvrk_env
```

If you want to use a simulator, please clone our Unity binaries, and ros_tcp_endpoint:


```bash
cd ~/medrct_ws
sh MedRCT/download_psm_unity_ros2.sh
git clone -b main-ros2 https://github.com/Unity-Technologies/ROS-TCP-Endpoint.git
source /opt/ros/<version>/setup.bash
colcon build --packages-up-to ros_tcp_endpoint
```

## DVRK Run

Each is a new terminal.

### Start a simulator

Unity psm

```bash
cd ~/medrct_ws/MedRCT/ros2/psm_unity/
./ros2_psm.x86_64
```

ROS TCP Endpoint
```bash
source ~/medrct_ws/install/setup.bash
ros2 launch ros_tcp_endpoint endpoint.py
```
## or start a dvrk robot instead
Refer to dvrk ros2 examples. Please use namespace /PSM1/

### Teleoepration
run:
```bash
source ~/medrct_ws/install/setup.bash

# For unity psm sim
ros2 run medrct_ros2_app ros2_teleop_node --ros-args -p config:=config/increment_example_scaled.yaml 

# For real psm
ros2 run medrct_ros2_app ros2_teleop_node --ros-args -p config:=config/increment_example.yaml
```

Start keyboard:
```bash
source ~/medrct_ws/install/setup.bash
cd ~/medrct_ws/MedRCT/ros2/medrct_ros2_app/scripts

# For sim
python3 ros2_psm_buffered_teleop_keyboard.py --scale 20

# For real
python3 ros2_psm_buffered_teleop_keyboard.py --scale 1
```

## Franka Build

build:
```bash
colcon build --packages-up-to medrct_franka_env medrct_default_controller medrct_ros2_app
```

TODO on Franka sim and build
