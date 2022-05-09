# dvrk_planning

This repo is a metapackage for dvrk planning algorithms.

List of Packages: 

- dvrk_planning: Python package for dvrk kinematics computation (right now).
- dvrk_planning_msgs: Ros messages and services definitions for the dvrk_planning_ros node.
- dvrk_planning_ros: The ros node which exposes the dvrk_planning api as ros services/msgs.


# MEDRCT
Install:
* colcon: https://colcon.readthedocs.io/en/released/user/installation.html

```
sudo apt install clang-format-7
sudo apt install libspdlog-dev
```

Build:
```
colcon build --packages-up-to "your package"
```

Build test:
```
colcon build --packages-up-to "your package" --cmake-args DBUILD_TESTING=ON

colcon test --packages-up-to "your_package"

colcon test-results --verbose
```