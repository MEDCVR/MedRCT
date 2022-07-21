Tab 1: 
cd sim_ws/ambf/bin/lin-x86_64/
./ambf_simulator -l 5

Tab 2: 
cd dvrk_ws/src/dvrk_planning/ambf_ros_crtk/scripts/
source ~/sim_ws/ambf/build/devel/setup.bash
python3 ambf_crtk_translator.py

Tab 3:
cd dvrk_ws
source devel/setup.bash
cd src
rosrun dvrk_planning_ros dvrk_teleop_node.py -y config/keyboard_psm2.yaml

Tab 4:
cd dvrk_ws
source devel/setup.bash
rosrun dvrk_planning_ros psm_teleop_keyboard.py

Tab 5:
cd dvrk_ws
source devel/setup.bash 
rosrun dvrk_planning_ros motion_generator.py -y config/motion_generator.yaml

Tab 6:
cd dvrk_ws
source devel/setup.bash 
rosrun dvrk_planning_ros teleop_goal_interface.py 

Tab 7:
cd dvrk_ws
source devel/setup.bash
rosrun dvrk_planning_ros switch_sim.py 

Tab 8:
cd dvrk_ws
source devel/setup.bash
rosrun dvrk_planning_ros shared_autonomy_control.py 



RVIZ

Tab 1:
cd dvrk_ws
source devel/setup.bash 
roslaunch dvrk_description psm_rviz.launch 

Tab 2:
cd dvrk_ws
source devel/setup.bash 
rosrun dvrk_planning_ros rviz_interface.py 
