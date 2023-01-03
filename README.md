# OW_VINS
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
#### operate two cameras with NTU_VIRAL dataset
1. modify the parameters in `ow_vins/config/ntu_viral/ntu_imu_config.yaml`
2. modify the parameters in `ow_vins/config/ntu_viral/cam0.yaml` and `ow_vins/config/ntu_viral/cam1.yaml`
3. `roslaunch vins_estimator ntu_viral.launch`

#### operate two cameras with HILTI dataset
1. modify the parameters in `ow_vins/config/hilti/[year]/hilti_imu_config.yaml`
2. modify the parameters in `ow_vins/config/hilti/[year]/cam0.yaml` and `ow_vins/config/hilti/[year]/cam1.yaml`
3. `roslaunch vins_estimator hilti.launch`

#### visualize the estimation results
1. `roslaunch vins_estimator obs_cam_visualize.launch`

## Result
#### Evaluate with the EVO tools
1. The [results](https://hackmd.io/u9gbL3VkRE2ntJs_SsPA1w?view) under NTU_VIRAL dataset
2. The [results](https://hackmd.io/TIUXasrASVmohV8RbpvRpA) under HILTI dataset 

