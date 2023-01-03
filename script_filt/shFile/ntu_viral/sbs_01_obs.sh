#! /bin/bash
DATA_NAME="sbs_01"
DATA_PATH="/home/zhehui/NTU_VIRAL/sbs_01"
RESULT_NAME="sbs_01_obs"
RESULT_PATH="/home/zhehui/estimation_ws/bag/NTU_VIRAL/results"

source /home/zhehui/estimation_ws/devel/setup.bash &&
roslaunch vins_estimator ntu_viral_obs.launch &
rosbag record -O $RESULT_PATH/$RESULT_NAME.bag /vins_estimator/odometry /vins_estimator/path /vins_estimator/sameId_feature_img /leica/pose/relative /vins_estimator/odometry_cam0 /vins_estimator/odometry_cam1 __name:=my_bag &
rosbag play $DATA_PATH/$DATA_NAME.bag --clock -r 0.5 &&
rosnode kill /my_bag && killall -9 rosmaster
