#include "obs_factor.h"

ObsFactor::ObsFactor(const Eigen::Vector3d &_pts_i, const Eigen::Vector3d &_pts_j) : pts_i(_pts_i), pts_j(_pts_j)
{

}

ObsFactor::Evaluate(double const *const *parameters, double *residuals, double **jacobians) const
{
  /*Vector3d pts_i = it_per_id.feature_per_frame[imu_i].point;
//    Vector3d pts_j = it_per_id.feature_per_frame[imu_j].point;

  Vector3d Pi(para_Pose[imu_i][0], para_Pose[imu_i][1], para_Pose[imu_i][2]);
  Quaterniond Qi(para_Pose[imu_i][6], para_Pose[imu_i][3], para_Pose[imu_i][4], para_Pose[imu_i][5]);

  Vector3d Pj(para_Pose[imu_j][0], para_Pose[imu_j][1], para_Pose[imu_j][2]);
  Quaterniond Qj(para_Pose[imu_j][6], para_Pose[imu_j][3], para_Pose[imu_j][4], para_Pose[imu_j][5]);

  Vector3d tic(para_Ex_Pose[0][0], para_Ex_Pose[0][1], para_Ex_Pose[0][2]);
  Quaterniond qic(para_Ex_Pose[0][6], para_Ex_Pose[0][3], para_Ex_Pose[0][4], para_Ex_Pose[0][5]);

  Matrix3d ric = qic.toRotationMatrix();
  Matrix3d Ri = Qi.toRotationMatrix();
  Matrix3d Rj = Qj.toRotationMatrix();

  double inv_dep_i = para_Feature[feature_index][0];

  Vector3d pts_camera_i = pts_i/inv_dep_i;
  Vector3d pts_imu_i = qic * pts_camera_i + tic;
  Vector3d pts_w = Qi * pts_imu_i + Pi;
  Vector3d pts_imu_j = Qj.inverse() * (pts_w - Pj);
  Vector3d pts_camera_j = qic.inverse() * (pts_imu_j - tic);
  double dep_j = pts_camera_j.z();

  //不知道為什麼會有 dep_j = 0
  if(dep_j == 0)
    continue;

  MatrixXd reduce(2,3);

  reduce << 1 / dep_j, 0, -pts_camera_j(0) / (dep_j * dep_j),
            0, 1 / dep_j, -pts_camera_j(1) / (dep_j * dep_j);

  Matrix3d R_c_b = ric.transpose();
  Matrix3d R_bj_w = Rj.transpose();
  Matrix3d R_w_bi = Ri;
  double lamda_l = inv_dep_i;

  Vector3d p_w_bi = Pi;
  Vector3d p_w_bj= Pj;
  Vector3d P_b_c = tic;

  // from rc matrix
  Eigen::Matrix3d temp_top3 = (-1.0 * R_c_b * R_bj_w).transpose();
  Eigen::Matrix3d temp_mid3 = (R_c_b * Utility::skewSymmetric(R_bj_w * (R_w_bi * (R_c_b.transpose() * 1.0 / lamda_l * pts_i + P_b_c) + p_w_bi - p_w_bj))).transpose();

  // construct 9 x 3 matrix;
  Eigen::MatrixXd temp_right_matrix(9,3);
  temp_right_matrix.setZero();
  temp_right_matrix.block<3,3>(0,0) = temp_top3;
  temp_right_matrix.block<3,3>(3,0) = temp_mid3;

  // construct 2 x 9 matrix;
  Eigen::MatrixXd rc_Matrix(2,9);
  rc_Matrix.setZero();
  rc_Matrix = reduce * temp_right_matrix.transpose();

  H.block<2,9>((2 * residual_num),0) = rc_Matrix;
  feature_index++;
  residual_num++;

  MatrixXd CostFunction(9,9);
  CostFunction = H.transpose() * H;
  Observation_matrix_trace = CostFunction.trace();

  // debug
  ROS_INFO("valid_feature: %d", valid_feature);
  std::cout << "Observation_matrix_trace: " << Observation_matrix_trace << std::endl;*/
}
