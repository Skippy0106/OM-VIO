#! /bin/bash
DATA_NAME="indoor_flying1_data"
DATA_PATH="/home/zhehui/estimation_ws/bag/MVSEC/indoor"
RESULT_NAME="indoor_flying1_result"
RESULT_PATH=$MVSEC_RESULT_PATH

roslaunch vins_estimator euroc.launch &
rosbag record -O $RESULT_PATH/$RESULT_NAME.bag /vins_estimator/odometry /vins_estimator/path /vins_estimator/sameId_feature_img __name:=my_bag &
rosbag play $DATA_PATH/$DATA_NAME.bag --clock -r 0.25 &&
rosnode kill /my_bag && killall -9 rosmaster
