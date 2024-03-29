#ifndef FEATURE_MANAGER_H
#define FEATURE_MANAGER_H

#include <list>
#include <algorithm>
#include <vector>
#include <numeric>
using namespace std;

#include <eigen3/Eigen/Dense>
using namespace Eigen;

#include <ros/console.h>
#include <ros/assert.h>

#include "parameters.h"

class FeaturePerFrame
{
  public:
#if two_cam_test
    FeaturePerFrame(const Eigen::Matrix<double, 7, 1> &_point, double td, int _camera_id=0)
    {
        point.x() = _point(0);
        point.y() = _point(1);
        point.z() = _point(2);
        uv.x() = _point(3);
        uv.y() = _point(4);
        velocity.x() = _point(5);
        velocity.y() = _point(6);
        cur_td = td;

        //obs
        obs_value = 0.0;

        camera_id = _camera_id;
    }
#else
    FeaturePerFrame(const Eigen::Matrix<double, 7, 1> &_point, double td)
    {
        point.x() = _point(0);
        point.y() = _point(1);
        point.z() = _point(2);
        uv.x() = _point(3);
        uv.y() = _point(4);
        velocity.x() = _point(5); 
        velocity.y() = _point(6); 
        cur_td = td;

        //obs
        obs_value = 0.0;
    }
#endif
    double cur_td;
    Vector3d point;
    Vector2d uv;
    Vector2d velocity;
    double z;
    bool is_used;
    double parallax;
    MatrixXd A;
    VectorXd b;
    double dep_gradient;

    //obs
    double obs_value;

#if two_cam_test
    int camera_id;
#endif
};

class FeaturePerId
{
  public:
    const int feature_id;
    int start_frame;
    vector<FeaturePerFrame> feature_per_frame;

    int used_num;
    bool is_outlier;
    bool is_margin;
    double estimated_depth;

    // for two camera init 

    double estimated_cam0_depth;
    double estimated_cam1_depth;

    int solve_flag; // 0 haven't solve yet; 1 solve succ; 2 solve fail;

    Vector3d gt_p;

    FeaturePerId(int _feature_id, int _start_frame)
        : feature_id(_feature_id), start_frame(_start_frame),
          used_num(0), estimated_depth(-1.0), solve_flag(0)
    {
    }

    int endFrame();
};

class FeatureManager
{
  public:
    FeatureManager(Matrix3d _Rs[]);

    void setRic(Matrix3d _ric[]);

    void clearState();

    int getFeatureCount();
    // two camera init 
    int getFeatureCount_init(int camera_id);

    bool addFeatureCheckParallax(int frame_count, const map<int, vector<pair<int, Eigen::Matrix<double, 7, 1>>>> &image, double td);
    void debugShow();
    vector<pair<Vector3d, Vector3d>> getCorresponding(int frame_count_l, int frame_count_r);
    // two camera init 
    vector<pair<Vector3d, Vector3d>> getCorresponding_init(int frame_count_l, int frame_count_r,int camera_id);

    //void updateDepth(const VectorXd &x);
    void setDepth(const VectorXd &x);
    //two camera init 
    void setDepth_init(const VectorXd &x,int camera_id);

    void removeFailures();
    void clearDepth(const VectorXd &x);
    VectorXd getDepthVector();
    // two camera init 
    VectorXd getDepthVector_init(int camera_id);

    //VectorXd getDepthVector_opt();
    void triangulate(Vector3d Ps[], Vector3d tic[], Matrix3d ric[]);
    // two camera init
    void triangulate_init(Vector3d Ps[], Vector3d tic[], Matrix3d ric[],int camera_id);

    void triangulate_init_opt(Vector3d Ps[], Vector3d tic[], Matrix3d ric[],int camera_id);


    void removeBackShiftDepth(Eigen::Matrix3d marg_R, Eigen::Vector3d marg_P, Eigen::Matrix3d new_R, Eigen::Vector3d new_P);
    void removeBack();
    void removeFront(int frame_count);
    void removeOutlier();
    list<FeaturePerId> feature;
    int last_track_num;
    

#if two_cam_test
    enum MainCam
    {
        CAM0,
        CAM1,
        ALL_CAM
    };

    MainCam main_cam;
#endif

  private:
    double compensatedParallax2(const FeaturePerId &it_per_id, int frame_count);
    const Matrix3d *Rs;
#if two_cam_test
    Matrix3d ric[2];
#else
    Matrix3d ric[NUM_OF_CAM];
#endif
};

#endif
