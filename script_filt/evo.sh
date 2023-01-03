BAG_PATH="/home/zhehui/estimation_ws/bag/MVSEC/results"
Result_name="evo_results.txt"

rpe_command="/transformed_ground_truth /vins_estimator/odometry"
rpe_options="-r trans_part --t_max_diff 0.1"
rpe_options_plot="-r trans_part -p --t_max_diff 0.1 --plot_mode xy"

ape_command="/transformed_ground_truth /vins_estimator/odometry"
ape_options="-r trans_part --t_max_diff 0.1 -a --n_to_align 50"
ape_options_plot="-r trans_part --t_max_diff 0.1 -p --plot_mode xy"

# RPE
#evo_="evo_rpe"
#command_=$rpe_command" "$rpe_options
#Result_name="evo_rpe_results.txt"

# APE
evo_="evo_ape"
command_=$ape_command" "$ape_options
Result_name="evo_ape_results.txt"

echo $BAG_PATH

echo "Bag: outdoor_day1_result" > $BAG_PATH/$Result_name && $evo_ bag $BAG_PATH/outdoor_day1_result.bag $command_ >> $BAG_PATH/$Result_name
echo "Finish outdoor_day1_result"
echo "Bag: outdoor_day2_result" >> $BAG_PATH/$Result_name && $evo_ bag $BAG_PATH/outdoor_day2_result.bag $command_ >> $BAG_PATH/$Result_name
echo "Finish outdoor_day2_result"
echo "Bag: outdoor_night1_result" >> $BAG_PATH/$Result_name && $evo_ bag $BAG_PATH/outdoor_night1_result.bag $command_ >> $BAG_PATH/$Result_name
echo "Finish outdoor_night1_result"
echo "Bag: outdoor_night2_result" >> $BAG_PATH/$Result_name && $evo_ bag $BAG_PATH/outdoor_night2_result.bag $command_ >> $BAG_PATH/$Result_name
echo "Finish outdoor_night2_result"
echo "Bag: outdoor_night3_result" >> $BAG_PATH/$Result_name && $evo_ bag $BAG_PATH/outdoor_night3_result.bag $command_ >> $BAG_PATH/$Result_name
echo "Finish outdoor_night3_result"
echo "Bag: outdoor_day1_result_obs" >> $BAG_PATH/$Result_name && $evo_ bag $BAG_PATH/outdoor_day1_result_obs.bag $command_ >> $BAG_PATH/$Result_name
echo "Finish outdoor_day1_result_obs"
echo "Bag: outdoor_day2_result_obs" >> $BAG_PATH/$Result_name && $evo_ bag $BAG_PATH/outdoor_day2_result_obs.bag $command_ >> $BAG_PATH/$Result_name
echo "Finish outdoor_day2_result_obs"
echo "Bag: outdoor_night1_result_obs" >> $BAG_PATH/$Result_name && $evo_ bag $BAG_PATH/outdoor_night1_result_obs.bag $command_ >> $BAG_PATH/$Result_name
echo "Finish outdoor_night1_result_obs"
echo "Bag: outdoor_night2_result_obs" >> $BAG_PATH/$Result_name && $evo_ bag $BAG_PATH/outdoor_night2_result_obs.bag $command_ >> $BAG_PATH/$Result_name
echo "Finish outdoor_night2_result_obs"
echo "Bag: outdoor_night3_result_obs" >> $BAG_PATH/$Result_name && $evo_ bag $BAG_PATH/outdoor_night3_result_obs.bag $command_ >> $BAG_PATH/$Result_name
echo "Finish outdoor_night3_result_obs"
