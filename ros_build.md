
# ROS1 Build

Install ROS: http://wiki.ros.org/noetic/Installation/Ubuntu

```bash
mkdir ~/medrct_ws && cd medrct_ws
git clone https://github.com/MEDCVR/MedRCT
git clone -b ambf-1.0 https://github.com/WPI-AIM/ambf # If you want a simulator
```

## Docker

Since ROS1 is old, the instructions use docker. Start the ros:noetic docker container:
```bash
sudo docker pull ros:noetic

# Start the docker container
cd ~/medrct_ws/MedRCT/docker
sh run_ros_noetic.sh
```

For each new terminal, all the following steps require to go into the docker container:

```bash
sh ~/medrct_ws/MedRCT/docker/exec_ros_noetic.sh
```

If the display does not work, on host computer try:
xhost +local:docker

## Build

Dependencies
```bash
sh ~/medrct_ws/MedRCT/install_deps.sh
```

Build medrct with ROS, dVRK kinematics, and default controllers.

```bash
source /opt/ros/noetic/setup.bash
cd ~/medrct_ws
colcon build --packages-up-to medrct_ros_app medrct_dvrk_env medrct_default_controller
```

If you want to use a simulator build ambf:
```bash
source /opt/ros/noetic/setup.bash
cd ~/medrct_ws/ambf
sudo apt update && xargs -a ~/medrct_ws/MedRCT/ambf-1.0-apt-requirements.txt sudo apt-get install -y
mkdir build && cd build
cmake ..
make
```

## Run
Each step is a new terminal

1. 
```bash
source /opt/ros/noetic/setup.bash
roscore
```
## Start a simulator
2. Start AMBF PSM output modality:
```bash
source ~/medrct_ws/ambf/build/devel/setup.bash
cd ~/medrct_ws/ambf/bin/lin-x86_64/
./ambf_simulator -l 5
```

3. Start AMBF CRTK interface:
```bash
source ~/medrct_ws/ambf/build/devel/setup.bash
cd ~/medrct_ws/MedRCT/ros/medrct_ros_app/scripts
python3 ambf_crtk_translator.py 
# If it doesn't work (psm doesn't stand), ctrl+c and try again
```

## or Start a dvrk robot instead

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
cd ~/medrct_ws/MedRCT/ros/medrct_ros_app/scripts
python3 psm_teleop_keyboard.py
```
