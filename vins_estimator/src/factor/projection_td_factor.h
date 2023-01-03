#pragma once

#include <ros/assert.h>
#include <ceres/ceres.h>
#include <Eigen/Dense>
#include "../utility/utility.h"
#include "../utility/tic_toc.h"
#include "../parameters.h"

class ProjectionTdFactor : public ceres::SizedCostFunction<2, 7, 7, 7, 1, 1>
{
  public:
    ProjectionTdFactor(const Eigen::Vector3d &_pts_i, const Eigen::Vector3d &_pts_j,
                       const Eigen::Vector2d &_velocity_i, const Eigen::Vector2d &_velocity_j,
                       const double _td_i, const double _td_j, const double _row_i, const double _row_j);
    ProjectionTdFactor(const Eigen::Vector3d &_pts_i, const Eigen::Vector3d &_pts_j,
                       const Eigen::Vector2d &_velocity_i, const Eigen::Vector2d &_velocity_j,
                       const double _td_i, const double _td_j, const double _row_i, const double _row_j,
                       const double &_my_observability, const double &_total_obs_val, const int &_total_camera_residual);
    virtual bool Evaluate(double const *const *parameters, double *residuals, double **jacobians) const;
    void check(double **parameters);

    Eigen::Vector3d pts_i, pts_j;
    Eigen::Vector3d velocity_i, velocity_j;
    double td_i, td_j;
    Eigen::Matrix<double, 2, 3> tangent_base;
    double row_i, row_j;
    static Eigen::Matrix2d sqrt_info;
    static double sum_t;

    // obs
    double my_observability;
    double total_obs_val; // the sum of all obs_val in the sliding window
    int total_camera_residual; // the number of camera residuals
    bool OBS_COEFFICIENT;
};
