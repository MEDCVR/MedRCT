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

Build:
```
colcon build --packages-up-to "package_name"
```

Test:
```
colcon test --packages-select "package_name"

colcon test-results --verbose
```


## Issues with KDL and Eigen

```
sudo ln -s /usr/include/eigen3/Eigen /usr/include/Eigen
```
