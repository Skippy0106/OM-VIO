# OM_VIO
![Image](LOGO.png?raw=true "Title")
## Arthors
* Zhe-Hui Chen

## Preamble
OW_VINS is a package aimed to solve SLAM problem in real time with multi-camera-inertial system.

## Prerequisites
1. ROS Kinetic or Melodic [ROS INSTALLATION](http://wiki.ros.org/ROS/Installation)
2. Ceres Solver [CERES INSTALLATION](http://ceres-solver.org/installation.html)
3. PCL [PCL INSTALLATION](https://pointclouds.org/downloads/)

## Build OW_VINS
1. `$ cd [workspace]/src`
2. `$ git clone [this url]`
3. `$ catkin_make`
4. `$ source [workspace]/devel/setup.bash` or
`$ source [workspace]/devel/setup.zsh`

## Run OW_VINS
#### operate two cameras in experiment 
1. modify the parameters in `OM-VIO/config/exp/exp_imu_config.yaml`
2. modify the parameters in `OM-VIO/config/exp/cam0.yaml` and `ow_vins/config/exp/cam1.yaml`
3. `roslaunch vins_estimator exp.launch`

#### visualize the estimation results
1. enter the path `/OM_VIO/config`
2. rivz -d obs_cam_visualize_config.rviz


