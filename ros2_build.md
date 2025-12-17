# ROS2 Build

Install ROS2 for your specific ubuntu version.

```bash
mkdir ~/medrct_ws && cd medrct_ws
git clone https://mcsgitlab.utm.utoronto.ca/medcvr/dvrk_planning
want a dvrk simulator
```

Dependencies
```bash
sh ~/medrct_ws/dvrk_planning/install_deps.sh
```

## DVRK Build 

Build:
```bash
source /opt/ros/<version>/setup.bash
cd ~/medrct_ws
colcon build --packages-up-to medrct_ros2_app medrct_default_controller medrct_dvrk_env
```

If you want to use a simulator build ambf:

```bash
cd ~/medrct_ws
git clone -b ambf-3.0 https://github.com/WPI-AIM/ambf
colcon build --packages-up-to AMBF ros_comm_plugin ambf_server tf_function ambf_client
```

## DVRK Run

Each is a new terminal.

### Start a simulator

1. Ambf with psm
```bash
source ~/medrct_ws/install/setup.bash
ambf_simulator -l 5 # PSM
```
2. Ambf crtk translator
```bash
source ~/medrct_ws/install/setup.bash
 cd ~/medrct_ws/dvrk_planning/ros2/medrct_ros2_app/scripts/
 python3 ambf_crtk_translator.py
```

## or start a dvrk robot instead
Refer to dvrk ros2 examples. Please use namespace /PSM1/

### Teleoepration
run:
```bash
source ~/medrct_ws/install/setup.bash
ros2 run medrct_ros2_app ros2_teleop_node --ros-args -p config:=config/increment_example.yaml
```

Start keyboard:
```bash
cd <dvrk_planning_folder>/ros2/medrct_ros2_app/scripts
python3 ros2_psm_buffered_teleop_keyboard.py
```

## Franka Build

build:
```bash
colcon build --packages-up-to medrct_franka_env medrct_default_controller medrct_ros2_app
```
