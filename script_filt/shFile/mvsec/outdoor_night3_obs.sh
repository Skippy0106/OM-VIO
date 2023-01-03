#! /bin/bash
DATA_NAME="outdoor_night3_data"
GT_NAME="outdoor_night3_gt"
DATA_PATH="/home/zhehui/estimation_ws/bag/MVSEC/outdoor"
RESULT_NAME="outdoor_night3_result_obs"
RESULT_PATH=$MVSEC_RESULT_PATH

source /home/zhehui/estimation_ws/devel/setup.bash &&
roslaunch vins_estimator euroc_mvsec_outdoor.launch &
source /home/zhehui/px4_ws/devel/setup.bash &&
rosrun offboard show_gt &
rosbag record -O $RESULT_PATH/$RESULT_NAME.bag /vins_estimator/odometry /vins_estimator/path /vins_estimator/sameId_feature_img /transformed_ground_truth __name:=my_bag &
rosbag play $DATA_PATH/$DATA_NAME.bag --clock -r 0.25 &&
rosbag play $DATA_PATH/$GT_NAME.bag --clock &&
rosnode kill /my_bag && killall -9 rosmaster

