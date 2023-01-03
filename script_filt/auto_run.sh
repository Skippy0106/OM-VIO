#!/bin/bash
MVSEC=false
NTU_VIRAL=true
export MVSEC_RESULT_PATH="/home/zhehui/estimation_ws/bag/MVSEC/results"

if $MVSEC; then
echo "Run MVSEC dataset..."
# MVSEC outdoot
#bash shFile/outdoor_day1.sh && 
#bash shFile/outdoor_day2.sh &&
#bash shFile/outdoor_night1.sh &&
#bash shFile/outdoor_night2.sh &&
#bash shFile/outdoor_night3.sh

# MVSEC outdoot with obs weightwd
#bash shFile/outdoor_day1_obs.sh
bash shFile/outdoor_day2_obs.sh
#bash shFile/outdoor_night1_obs.sh &&
#bash shFile/outdoor_night2_obs.sh &&
#bash shFile/outdoor_night3_obs.sh 
fi

if $NTU_VIRAL; then
echo "Run NTU_VIRAL dataset..."
# --- Original Vins ---
bash shFile/ntu_viral/eee_01.sh &&
bash shFile/ntu_viral/eee_02.sh &&
bash shFile/ntu_viral/eee_03.sh &&
bash shFile/ntu_viral/nya_01.sh &&
bash shFile/ntu_viral/nya_02.sh &&
bash shFile/ntu_viral/nya_03.sh &&
bash shFile/ntu_viral/sbs_01.sh &&
bash shFile/ntu_viral/sbs_02.sh &&
bash shFile/ntu_viral/sbs_03.sh 
# ------
# --- Vins with observability ---
#bash shFile/ntu_viral/eee_01_obs.sh &&
#bash shFile/ntu_viral/eee_02_obs.sh &&
#bash shFile/ntu_viral/eee_03_obs.sh &&
#bash shFile/ntu_viral/nya_01_obs.sh &&
#bash shFile/ntu_viral/nya_02_obs.sh &&
#bash shFile/ntu_viral/nya_03_obs.sh &&
#bash shFile/ntu_viral/sbs_01_obs.sh &&
#bash shFile/ntu_viral/sbs_02_obs.sh &&
#bash shFile/ntu_viral/sbs_03_obs.sh 
# ------
fi
