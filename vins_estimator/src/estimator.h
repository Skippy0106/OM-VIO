#pragma once

#include "parameters.h"
#include "feature_manager.h"
#include "utility/utility.h"
#include "utility/tic_toc.h"
#include "initial/solve_5pts.h"
#include "initial/initial_sfm.h"
#include "initial/initial_alignment.h"
#include "initial/initial_ex_rotation.h"
#include <std_msgs/Header.h>
#include <std_msgs/Float32.h>

#include <ceres/ceres.h>
#include "factor/imu_factor.h"
#include "factor/pose_local_parameterization.h"
#include "factor/projection_factor.h"
#include "factor/projection_td_factor.h"
#include "factor/marginalization_factor.h"

#include <unordered_map>
#include <queue>
#include <opencv2/core/eigen.hpp>

#define two_cam_test 1

class Estimator
{
  public:
    Estimator();

    void setParameter();

    // interface
    void processIMU(double t, const Vector3d &linear_acceleration, const Vector3d &angular_velocity);
    void processImage(const map<int, vector<pair<int, Eigen::Matrix<double, 7, 1>>>> &image, const std_msgs::Header &header);
    void setReloFrame(double _frame_stamp, int _frame_index, vector<Vector3d> &_match_points, Vector3d _relo_t, Matrix3d _relo_r);

    // internal
    void clearState();
    bool initialStructure();
    bool visualInitialAlign();
    bool relativePose(Matrix3d &relative_R, Vector3d &relative_T, int &l , int camera_id);
    void slideWindow();
    void solveOdometry(const map<int, vector<pair<int, Eigen::Matrix<double, 7, 1>>>> &image, const std_msgs::Header &header);
    void solveOdometry_init(const map<int, vector<pair<int, Eigen::Matrix<double, 7, 1>>>> &image, const std_msgs::Header &header);
    void slideWindowNew();
    void slideWindowOld();
    void optimization();
    void optimization_cam0();
    void optimization_cam1();
    void vector2double();
    void double2vector();
    //two camera init 
    void vector2double_init();
    void double2vector_init();
    void estimate_diff();

    bool failureDetection();

    // obs_control
    void obs_trace(const map<int, vector<pair<int, Eigen::Matrix<double, 7, 1>>>> &image, const std_msgs::Header &header);
    // select camera for initialization 
    void obs_trace_init(const map<int, vector<pair<int, Eigen::Matrix<double, 7, 1>>>> &image, const std_msgs::Header &header,int camera_id);


    void obs_optimization();
    double obs_para_Pose[SIZE_POSE];
    double obs_para_SpeedBias[SIZE_SPEEDBIAS];
    int valid_feature;
#if two_cam_test
    double Observation_matrix_trace[2];
#else
    double Observation_matrix_trace;
#endif
    double inverse_observation_matrix_trace;
    double jacobian[3];

    // for current frame
#if two_cam_test
    vector<pair<int, cv::Point2f>> sameId_pts[2];
    vector<double> pts_observability[2];
    vector<double> pts_all_observability;
#else
    vector<pair<int, cv::Point2f>> sameId_pts;
    vector<double> pts_observability;
#endif
    // for frames in sliding window
    void optimization_obs();
    double total_obs_val();
    int total_camera_residual();

    // standard derivative params
#if two_cam_test
    double mean[2], stdev[2];
    double all_mean = 0;
    double all_stdev = 0;
    double obs_sum = 0;
#else
    double mean, stdev;
#endif

    //debug
    double debug_lamda_l;
    double debug_trace = 0;

    enum SolverFlag
    {
        INITIAL,
        NON_LINEAR
    };

    enum MarginalizationFlag
    {
        MARGIN_OLD = 0,
        MARGIN_SECOND_NEW = 1
    };

    SolverFlag solver_flag;
    MarginalizationFlag  marginalization_flag;
    // two camera init
    Vector3d g,g_1;
    MatrixXd Ap[2], backup_A;
    VectorXd bp[2], backup_b;

#if two_cam_test
    Matrix3d ric[2];
    Vector3d tic[2];
#else
    Matrix3d ric[NUM_OF_CAM];
    Vector3d tic[NUM_OF_CAM];
#endif

    Vector3d Ps[(WINDOW_SIZE + 1)];
    Vector3d Vs[(WINDOW_SIZE + 1)];
    Matrix3d Rs[(WINDOW_SIZE + 1)];
    Vector3d Bas[(WINDOW_SIZE + 1)];
    Vector3d Bgs[(WINDOW_SIZE + 1)];
    double td;
    // two camera init 
    Vector3d Ps_0[(WINDOW_SIZE + 1)];
    Vector3d Vs_0[(WINDOW_SIZE + 1)];
    Matrix3d Rs_0[(WINDOW_SIZE + 1)];
    Vector3d Ps_1[(WINDOW_SIZE + 1)];
    Vector3d Vs_1[(WINDOW_SIZE + 1)];
    Matrix3d Rs_1[(WINDOW_SIZE + 1)];

    Matrix3d back_R0, last_R, last_R0;
    Vector3d back_P0, last_P, last_P0;
    // sliding window for OBS 
    double last_OBS;
    std_msgs::Header Headers[(WINDOW_SIZE + 1)];

    IntegrationBase *pre_integrations[(WINDOW_SIZE + 1)];
    Vector3d acc_0, gyr_0;

    vector<double> dt_buf[(WINDOW_SIZE + 1)];
    vector<Vector3d> linear_acceleration_buf[(WINDOW_SIZE + 1)];
    vector<Vector3d> angular_velocity_buf[(WINDOW_SIZE + 1)];

    int frame_count;
    int sum_of_outlier, sum_of_back, sum_of_front, sum_of_invalid;

    FeatureManager f_manager;
    MotionEstimator m_estimator;
    InitialEXRotation initial_ex_rotation;

    bool first_imu;
    bool is_valid, is_key;
    bool failure_occur;

    vector<Vector3d> point_cloud;
    vector<Vector3d> margin_cloud;
    vector<Vector3d> key_poses;
    double initial_timestamp;


    double para_Pose[WINDOW_SIZE + 1][SIZE_POSE];
    //two cam init 
    double para_Pose_cam0[WINDOW_SIZE + 1][SIZE_POSE];
    double para_Pose_cam1[WINDOW_SIZE + 1][SIZE_POSE];


    double para_SpeedBias[WINDOW_SIZE + 1][SIZE_SPEEDBIAS];
    // two camera init 
    double para_SpeedBias_cam0[WINDOW_SIZE + 1][SIZE_SPEEDBIAS];
    double para_SpeedBias_cam1[WINDOW_SIZE + 1][SIZE_SPEEDBIAS];

    double para_Feature[NUM_OF_F][SIZE_FEATURE];
    // two camera init
    double para_Feature_cam0[NUM_OF_F][SIZE_FEATURE];
    double para_Feature_cam1[NUM_OF_F][SIZE_FEATURE];


#if two_cam_test
    double para_Ex_Pose[2][SIZE_POSE];
#else
    double para_Ex_Pose[NUM_OF_CAM][SIZE_POSE];
#endif
    double para_Retrive_Pose[SIZE_POSE];
    double para_Td[1][1];
    double para_Tr[1][1];

    int loop_window_index;

    MarginalizationInfo *last_marginalization_info;
    vector<double *> last_marginalization_parameter_blocks;


    vector<double *> last_marginalization_parameter_blocks_1;

    map<double, ImageFrame> all_image_frame;
    IntegrationBase *tmp_pre_integration;

    //relocalization variable
    bool relocalization_info;
    double relo_frame_stamp;
    double relo_frame_index;
    int relo_frame_local_index;
    vector<Vector3d> match_points;
    double relo_Pose[SIZE_POSE];
    Matrix3d drift_correct_r;
    Vector3d drift_correct_t;
    Vector3d prev_relo_t;
    Matrix3d prev_relo_r;
    Vector3d relo_relative_t;
    Quaterniond relo_relative_q;
    double relo_relative_yaw;
};
