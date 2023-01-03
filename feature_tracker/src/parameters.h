#pragma once
#include <ros/ros.h>
#include <opencv2/highgui/highgui.hpp>

#define two_cam_test 1

extern int ROW;
extern int COL;
extern int FOCAL_LENGTH;

#if two_cam_test
extern int NUM_OF_CAM;
extern std::string IMAGE0_TOPIC;
extern std::string IMAGE1_TOPIC;
#else
const int NUM_OF_CAM = 1;
extern std::string IMAGE_TOPIC;
#endif
extern std::string IMU_TOPIC;
extern std::string FISHEYE_MASK;
extern std::vector<std::string> CAM_NAMES;
extern int MAX_CNT;
extern int MIN_DIST;
extern int WINDOW_SIZE;
extern int FREQ;
extern double F_THRESHOLD;
extern int SHOW_TRACK;
extern int STEREO_TRACK;
extern int EQUALIZE;
extern int FISHEYE;
extern bool PUB_THIS_FRAME;

void readParameters(ros::NodeHandle &n);
