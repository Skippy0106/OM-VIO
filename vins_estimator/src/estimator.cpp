#include "estimator.h"
#define TEST 0

Estimator::Estimator() : f_manager{Rs}
{
    ROS_INFO("init begins");
    clearState();
}

void Estimator::setParameter()
{
    for (int i = 0; i < NUM_OF_CAM; i++)
    {
        tic[i] = TIC[i];
        ric[i] = RIC[i];
    }
    f_manager.setRic(ric);
    ProjectionFactor::sqrt_info = FOCAL_LENGTH / 1.5 * Matrix2d::Identity();
    ProjectionTdFactor::sqrt_info = FOCAL_LENGTH / 1.5 * Matrix2d::Identity();
    td = TD;
}

void Estimator::clearState()
{
    for (int i = 0; i < WINDOW_SIZE + 1; i++)
    {
        Rs[i].setIdentity();
        Ps[i].setZero();
        Vs[i].setZero();
        Bas[i].setZero();
        Bgs[i].setZero();
        dt_buf[i].clear();
        linear_acceleration_buf[i].clear();
        angular_velocity_buf[i].clear();

        if (pre_integrations[i] != nullptr)
            delete pre_integrations[i];
        pre_integrations[i] = nullptr;
    }

    for (int i = 0; i < NUM_OF_CAM; i++)
    {
        tic[i] = Vector3d::Zero();
        ric[i] = Matrix3d::Identity();
    }

    for (auto &it : all_image_frame)
    {
        if (it.second.pre_integration != nullptr)
        {
            delete it.second.pre_integration;
            it.second.pre_integration = nullptr;
        }
    }

    solver_flag = INITIAL;
    first_imu = false,
    sum_of_back = 0;
    sum_of_front = 0;
    frame_count = 0;
    solver_flag = INITIAL;
    initial_timestamp = 0;
    all_image_frame.clear();
    td = TD;

    if (tmp_pre_integration != nullptr)
        delete tmp_pre_integration;
    if (last_marginalization_info != nullptr)
        delete last_marginalization_info;

    tmp_pre_integration = nullptr;
    last_marginalization_info = nullptr;
    last_marginalization_parameter_blocks.clear();

    f_manager.clearState();

    failure_occur = 0;
    relocalization_info = 0;

    drift_correct_r = Matrix3d::Identity();
    drift_correct_t = Vector3d::Zero();
}

void Estimator::processIMU(double dt, const Vector3d &linear_acceleration, const Vector3d &angular_velocity)
{
    if (!first_imu)
    {
        first_imu = true;
        acc_0 = linear_acceleration;
        gyr_0 = angular_velocity;
    }

    if (!pre_integrations[frame_count])
    {
        pre_integrations[frame_count] = new IntegrationBase{acc_0, gyr_0, Bas[frame_count], Bgs[frame_count]};
    }
    if (frame_count != 0)
    {
        pre_integrations[frame_count]->push_back(dt, linear_acceleration, angular_velocity);
        // if(solver_flag != NON_LINEAR)
        tmp_pre_integration->push_back(dt, linear_acceleration, angular_velocity);

        dt_buf[frame_count].push_back(dt);
        linear_acceleration_buf[frame_count].push_back(linear_acceleration);
        angular_velocity_buf[frame_count].push_back(angular_velocity);

        int j = frame_count;
        Vector3d un_acc_0 = Rs[j] * (acc_0 - Bas[j]) - g;
        Vector3d un_gyr = 0.5 * (gyr_0 + angular_velocity) - Bgs[j];
        Rs[j] *= Utility::deltaQ(un_gyr * dt).toRotationMatrix();
        Vector3d un_acc_1 = Rs[j] * (linear_acceleration - Bas[j]) - g;
        Vector3d un_acc = 0.5 * (un_acc_0 + un_acc_1);
        Ps[j] += dt * Vs[j] + 0.5 * dt * dt * un_acc;
        Vs[j] += dt * un_acc;
    }
    acc_0 = linear_acceleration;
    gyr_0 = angular_velocity;
}

void Estimator::processImage(const map<int, vector<pair<int, Eigen::Matrix<double, 7, 1>>>> &image, const std_msgs::Header &header)
{
    ROS_DEBUG("new image coming ------------------------------------------");
    ROS_DEBUG("Adding feature points %lu", image.size());
    if (f_manager.addFeatureCheckParallax(frame_count, image, td))
        marginalization_flag = MARGIN_OLD;
    else
        marginalization_flag = MARGIN_SECOND_NEW;

    ROS_DEBUG("this frame is--------------------%s", marginalization_flag ? "reject" : "accept");
    ROS_DEBUG("%s", marginalization_flag ? "Non-keyframe" : "Keyframe");
    ROS_DEBUG("Solving %d", frame_count);
    ROS_DEBUG("number of feature: %d", f_manager.getFeatureCount());
    Headers[frame_count] = header;

    ImageFrame imageframe(image, header.stamp.toSec());
    imageframe.pre_integration = tmp_pre_integration;
    all_image_frame.insert(make_pair(header.stamp.toSec(), imageframe));
    tmp_pre_integration = new IntegrationBase{acc_0, gyr_0, Bas[frame_count], Bgs[frame_count]};

    if (ESTIMATE_EXTRINSIC == 2)
    {
        ROS_INFO("calibrating extrinsic param, rotation movement is needed");
        if (frame_count != 0)
        {
            vector<pair<Vector3d, Vector3d>> corres = f_manager.getCorresponding(frame_count - 1, frame_count);
            Matrix3d calib_ric;
            if (initial_ex_rotation.CalibrationExRotation(corres, pre_integrations[frame_count]->delta_q, calib_ric))
            {
                ROS_WARN("initial extrinsic rotation calib success");
                ROS_WARN_STREAM("initial extrinsic rotation: " << endl
                                                               << calib_ric);
                ric[0] = calib_ric;
                RIC[0] = calib_ric;
                ESTIMATE_EXTRINSIC = 1;
            }
        }
    }

#if two_cam_test
    f_manager.main_cam = f_manager.ALL_CAM;
#endif

    if (solver_flag == INITIAL)
    {
        if (frame_count == WINDOW_SIZE)
        {
            bool result = false;
            if (ESTIMATE_EXTRINSIC != 2 && (header.stamp.toSec() - initial_timestamp) > 0.1)
            {
                result = initialStructure();
                initial_timestamp = header.stamp.toSec();
            }
            if (result)
            {
                std::cout << "starting solving initialization processing " << std::endl;
                solver_flag = NON_LINEAR;
                // maybe can switch to all camera
                solveOdometry_init(image, header);
                slideWindow();
                f_manager.removeFailures();
                ROS_INFO("Initialization finish!");
                last_R = Rs[WINDOW_SIZE];
                last_P = Ps[WINDOW_SIZE];
                last_R0 = Rs[0];
                last_P0 = Ps[0];
            }
            else
                slideWindow();
        }
        else
            frame_count++;
    }
    else
    {
        f_manager.main_cam = f_manager.ALL_CAM;
        TicToc t_solve;
        solveOdometry(image, header);
        ROS_DEBUG("solver costs: %fms", t_solve.toc());

        if (failureDetection())
        {
            ROS_WARN("failure detection!");
            failure_occur = 1;
            clearState();
            setParameter();
            ROS_WARN("system reboot!");
            return;
        }

        TicToc t_margin;
        slideWindow();
        f_manager.removeFailures();
        ROS_DEBUG("marginalization costs: %fms", t_margin.toc());
        // prepare output of VINS
        key_poses.clear();
        for (int i = 0; i <= WINDOW_SIZE; i++)
            key_poses.push_back(Ps[i]);

        last_R = Rs[WINDOW_SIZE];
        last_P = Ps[WINDOW_SIZE];
        last_R0 = Rs[0];
        last_P0 = Ps[0];
    }
}
bool Estimator::initialStructure()
{
    TicToc t_sfm;
    // check imu observibility
    {
        map<double, ImageFrame>::iterator frame_it;
        Vector3d sum_g;
        for (frame_it = all_image_frame.begin(), frame_it++; frame_it != all_image_frame.end(); frame_it++)
        {
            double dt = frame_it->second.pre_integration->sum_dt;
            Vector3d tmp_g = frame_it->second.pre_integration->delta_v / dt;
            sum_g += tmp_g;
        }
        Vector3d aver_g;
        aver_g = sum_g * 1.0 / ((int)all_image_frame.size() - 1);
        double var = 0;
        for (frame_it = all_image_frame.begin(), frame_it++; frame_it != all_image_frame.end(); frame_it++)
        {
            double dt = frame_it->second.pre_integration->sum_dt;
            Vector3d tmp_g = frame_it->second.pre_integration->delta_v / dt;
            var += (tmp_g - aver_g).transpose() * (tmp_g - aver_g);
            // cout << "frame g " << tmp_g.transpose() << endl;
        }
        var = sqrt(var / ((int)all_image_frame.size() - 1));
        // ROS_WARN("IMU variation %f!", var);
        if (var < 0.25)
        {
            ROS_INFO("IMU excitation not enouth!");
            // return false;
        }
    }
    Quaterniond Q_cam0[frame_count + 1], Q_cam1[frame_count + 1];
    Vector3d T_cam0[frame_count + 1], T_cam1[frame_count + 1];
    vector<SFMFeature> sfm_f_cam0, sfm_f_cam1;
    map<int, Vector3d> sfm_tracked_cam0_points, sfm_tracked_cam1_points;
    for (auto &it_per_id : f_manager.feature)
    {
        int camera_id = it_per_id.feature_per_frame[0].camera_id;
        int imu_j = it_per_id.start_frame - 1;
        // two camera init
        SFMFeature tmp_feature_cam0, tmp_feature_cam1;
        tmp_feature_cam0.state = false;
        tmp_feature_cam1.state = false;
        if (camera_id == 0)
        {
            tmp_feature_cam0.id = it_per_id.feature_id;
            for (auto &it_per_frame : it_per_id.feature_per_frame)
            {
                imu_j++;
                Vector3d pts_j = it_per_frame.point;
                tmp_feature_cam0.observation.push_back(make_pair(imu_j, Eigen::Vector2d{pts_j.x(), pts_j.y()}));
            }
            sfm_f_cam0.push_back(tmp_feature_cam0);
        }
        else if (camera_id == 1)
        {
            tmp_feature_cam1.id = it_per_id.feature_id;
            for (auto &it_per_frame : it_per_id.feature_per_frame)
            {
                imu_j++;
                Vector3d pts_j = it_per_frame.point;
                tmp_feature_cam1.observation.push_back(make_pair(imu_j, Eigen::Vector2d{pts_j.x(), pts_j.y()}));
            }
            sfm_f_cam1.push_back(tmp_feature_cam1);
        }
    }
    Matrix3d relative_R_cam0;
    Vector3d relative_T_cam0;
    Matrix3d relative_R_cam1;
    Vector3d relative_T_cam1;
    int l;
    if (!relativePose(relative_R_cam0, relative_T_cam0, l, 0) || !relativePose(relative_R_cam1, relative_T_cam1, l, 1))
    {
        ROS_INFO("Not enough features or parallax; Move device around");
        return false;
    }
    // two camera init
    GlobalSFM sfm_cam0, sfm_cam1;
    if (!sfm_cam0.construct(frame_count + 1, Q_cam0, T_cam0, l,
                            relative_R_cam0, relative_T_cam0,
                            sfm_f_cam0, sfm_tracked_cam0_points) ||
        !sfm_cam1.construct(frame_count + 1, Q_cam1, T_cam1, l,
                            relative_R_cam1, relative_T_cam1,
                            sfm_f_cam1, sfm_tracked_cam1_points))
    {
        ROS_DEBUG("global SFM failed!");
        marginalization_flag = MARGIN_OLD;
        return false;
    }
    // solve pnp for all frame
    map<double, ImageFrame>::iterator frame_it;
    map<int, Vector3d>::iterator it;
    frame_it = all_image_frame.begin();
    for (int i = 0; frame_it != all_image_frame.end(); frame_it++)
    {
        // provide initial guess
        cv::Mat r_0, rvec_0, t_0, D_0, tmp_r_0;
        cv::Mat r_1, rvec_1, t_1, D_1, tmp_r_1;
        if ((frame_it->first) == Headers[i].stamp.toSec())
        {
            frame_it->second.is_key_frame = true;
            // two camera init
            frame_it->second.R[0] = Q_cam0[i].toRotationMatrix() * RIC[0].transpose();
            frame_it->second.R[1] = Q_cam1[i].toRotationMatrix() * RIC[1].transpose();
            // frame_it->second.T = T[i];
            frame_it->second.T[0] = T_cam0[i];
            frame_it->second.T[1] = T_cam1[i];
            i++;
            continue;
        }
        if ((frame_it->first) > Headers[i].stamp.toSec())
        {
            i++;
        }
        // two camera init
        Matrix3d R_cam0_inital = (Q_cam0[i].inverse()).toRotationMatrix();
        Matrix3d R_cam1_inital = (Q_cam1[i].inverse()).toRotationMatrix();
        Vector3d P_cam0_inital = -R_cam0_inital * T_cam0[i];
        Vector3d P_cam1_inital = -R_cam1_inital * T_cam1[i];
        cv::eigen2cv(R_cam0_inital, tmp_r_0);
        cv::eigen2cv(R_cam1_inital, tmp_r_1);
        cv::Rodrigues(tmp_r_0, rvec_0);
        cv::Rodrigues(tmp_r_1, rvec_1);
        cv::eigen2cv(P_cam0_inital, t_0);
        cv::eigen2cv(P_cam1_inital, t_1);

        frame_it->second.is_key_frame = false;
        vector<cv::Point3f> pts_3_vector_cam0;
        vector<cv::Point3f> pts_3_vector_cam1;
        vector<cv::Point2f> pts_2_vector_cam0;
        vector<cv::Point2f> pts_2_vector_cam1;
        for (auto &id_pts : frame_it->second.points)
        {
            int feature_id = id_pts.first;
            for (auto &i_p : id_pts.second)
            {
                it = sfm_tracked_cam0_points.find(feature_id);
                if (it != sfm_tracked_cam0_points.end())
                {
                    Vector3d world_pts = it->second;
                    cv::Point3f pts_3(world_pts(0), world_pts(1), world_pts(2));
                    pts_3_vector_cam0.push_back(pts_3);
                    Vector2d img_pts = i_p.second.head<2>();
                    cv::Point2f pts_2(img_pts(0), img_pts(1));
                    pts_2_vector_cam0.push_back(pts_2);
                }
                it = sfm_tracked_cam1_points.find(feature_id);
                if (it != sfm_tracked_cam1_points.end())
                {
                    Vector3d world_pts = it->second;
                    cv::Point3f pts_3(world_pts(0), world_pts(1), world_pts(2));
                    pts_3_vector_cam1.push_back(pts_3);
                    Vector2d img_pts = i_p.second.head<2>();
                    cv::Point2f pts_2(img_pts(0), img_pts(1));
                    pts_2_vector_cam1.push_back(pts_2);
                }
            }
        }
        cv::Mat K_0 = (cv::Mat_<double>(3, 3) << 1, 0, 0, 0, 1, 0, 0, 0, 1);
        cv::Mat K_1 = (cv::Mat_<double>(3, 3) << 1, 0, 0, 0, 1, 0, 0, 0, 1);
        if (pts_3_vector_cam0.size() < 6 || pts_3_vector_cam1.size() < 6)
        {
            ROS_DEBUG("Not enough points for solve pnp !");
            return false;
        }
        if (!cv::solvePnP(pts_3_vector_cam0, pts_2_vector_cam0, K_0, D_0, rvec_0, t_0, 1) || !cv::solvePnP(pts_3_vector_cam1, pts_2_vector_cam1, K_1, D_1, rvec_1, t_1, 1))
        {
            ROS_DEBUG("solve pnp fail!");
            return false;
        }
        cv::Rodrigues(rvec_0, r_0);
        MatrixXd R0_pnp, tmp_R0_pnp;
        cv::cv2eigen(r_0, tmp_R0_pnp);
        R0_pnp = tmp_R0_pnp.transpose();
        MatrixXd T0_pnp;
        cv::cv2eigen(t_0, T0_pnp);
        T0_pnp = R0_pnp * (-T0_pnp);

        cv::Rodrigues(rvec_1, r_1);
        MatrixXd R1_pnp, tmp_R1_pnp;
        cv::cv2eigen(r_1, tmp_R1_pnp);
        R1_pnp = tmp_R1_pnp.transpose();
        MatrixXd T1_pnp;
        cv::cv2eigen(t_1, T1_pnp);
        T1_pnp = R1_pnp * (-T1_pnp);

        frame_it->second.R[0] = R0_pnp * RIC[0].transpose();
        frame_it->second.R[1] = R1_pnp * RIC[1].transpose();
        frame_it->second.T[0] = T0_pnp;
        frame_it->second.T[1] = T1_pnp;
    }
    if (visualInitialAlign())
        return true;
    else
    {
        ROS_INFO("misalign visual structure with IMU");
        return false;
    }
}

bool Estimator::visualInitialAlign()
{
    TicToc t_g;
    VectorXd x;
    VectorXd x_1;

    // solve scale
#if two_cam_test
    bool result_cam0 = VisualIMUAlignment(all_image_frame, Bgs, g, x, 0);
    bool result_cam1 = VisualIMUAlignment(all_image_frame, Bgs, g_1, x_1, 1);
#else
    bool result = VisualIMUAlignment(all_image_frame, Bgs, g, x);
#endif

    if (!result_cam0 || !result_cam1)
    {
        ROS_DEBUG("solve g failed!");
        return false;
    }

    // change state
    for (int i = 0; i <= frame_count; i++)
    {
        Matrix3d Ri = all_image_frame[Headers[i].stamp.toSec()].R[0];
        Vector3d Pi = all_image_frame[Headers[i].stamp.toSec()].T[0];
        Ps_0[i] = Pi;
        Rs_0[i] = Ri;
        Ri = all_image_frame[Headers[i].stamp.toSec()].R[1];
        Pi = all_image_frame[Headers[i].stamp.toSec()].T[1];
        Ps_1[i] = Pi;
        Rs_1[i] = Ri;
        all_image_frame[Headers[i].stamp.toSec()].is_key_frame = true;
    }
    VectorXd dep = f_manager.getDepthVector();
    for (int i = 0; i < dep.size(); i++)
        dep[i] = -1;
    f_manager.clearDepth(dep);

    // triangulat on cam pose , no tic
    Vector3d TIC_TMP[NUM_OF_CAM];
    for (int i = 0; i < NUM_OF_CAM; i++)
        TIC_TMP[i].setZero();
    ric[0] = RIC[0];
    ric[1] = RIC[1];
    f_manager.setRic(ric);
    f_manager.triangulate_init(Ps_0, &(TIC_TMP[0]), &(RIC[0]), 0);
    f_manager.triangulate_init(Ps_1, &(TIC_TMP[1]), &(RIC[1]), 1);

    double s = (x.tail<1>())(0);
    double s_1 = (x_1.tail<1>())(0);
    std::cout << "scale of CAM0 = " << s << std::endl;
    std::cout << "scale of CAM1 = " << s_1 << std::endl;
    for (int i = 0; i <= WINDOW_SIZE; i++)
    {
        pre_integrations[i]->repropagate(Vector3d::Zero(), Bgs[i]);
    }
    for (int i = frame_count; i >= 0; i--)
    {
        Ps_0[i] = s * Ps_0[i] - Rs_0[i] * TIC[0] - (s * Ps_0[0] - Rs_0[0] * TIC[0]);
        Ps_1[i] = s_1 * Ps_1[i] - Rs_1[i] * TIC[1] - (s_1 * Ps_1[0] - Rs_1[0] * TIC[1]);
    }

    int kv = -1;
    map<double, ImageFrame>::iterator frame_i;
    for (frame_i = all_image_frame.begin(); frame_i != all_image_frame.end(); frame_i++)
    {
        if (frame_i->second.is_key_frame)
        {
            kv++;
            Vs_0[kv] = frame_i->second.R[0] * x.segment<3>(kv * 3);
            Vs_1[kv] = frame_i->second.R[1] * x_1.segment<3>(kv * 3);
        }
    }
    for (auto &it_per_id : f_manager.feature)
    {
        it_per_id.used_num = it_per_id.feature_per_frame.size();
        if (!(it_per_id.used_num >= 2 && it_per_id.start_frame < WINDOW_SIZE - 2))
            continue;

#if two_cam_test
        // the first optimize only depends on features in camesra0
        if (f_manager.main_cam == f_manager.CAM0)
        {
            if (it_per_id.feature_per_frame[0].camera_id != 0)
                continue;
        }
        else if (f_manager.main_cam == f_manager.CAM1)
            if (it_per_id.feature_per_frame[0].camera_id != 1)
                continue;
#endif

        // it_per_id.estimated_depth *= s;
        if (it_per_id.feature_per_frame[0].camera_id == 0)
        {
            it_per_id.estimated_cam0_depth *= s;
        }
        else if (it_per_id.feature_per_frame[0].camera_id == 1)
        {
            it_per_id.estimated_cam1_depth *= s_1;
        }
    }

    Matrix3d R0 = Utility::g2R(g);
    Matrix3d R0_cam1 = Utility::g2R(g_1);
    double yaw = Utility::R2ypr(R0 * Rs_0[0]).x();
    double yaw_1 = Utility::R2ypr(R0_cam1 * Rs_1[0]).x();
    R0 = Utility::ypr2R(Eigen::Vector3d{-yaw, 0, 0}) * R0;
    R0_cam1 = Utility::ypr2R(Eigen::Vector3d{-yaw_1, 0, 0}) * R0_cam1;
    g = R0 * g;
    g_1 = R0_cam1 * g_1;
    // Matrix3d rot_diff = R0 * Rs[0].transpose();
    Matrix3d rot_diff = R0;
    Matrix3d rot_diff_cam1 = R0_cam1;
    for (int i = 0; i <= frame_count; i++)
    {
        Ps_0[i] = rot_diff * Ps_0[i];
        Ps_1[i] = rot_diff_cam1 * Ps_1[i];
        Rs_0[i] = rot_diff * Rs_0[i];
        Rs_1[i] = rot_diff_cam1 * Rs_1[i];
        Vs_0[i] = rot_diff * Vs_0[i];
        Vs_1[i] = rot_diff_cam1 * Vs_1[i];
    }
    ROS_DEBUG_STREAM("g0     " << g.transpose());
    ROS_DEBUG_STREAM("my R0  " << Utility::R2ypr(Rs[0]).transpose());

    return true;
}

bool Estimator::relativePose(Matrix3d &relative_R, Vector3d &relative_T, int &l, int camera_id)
{
    // find previous frame which contians enough correspondance and parallex with newest frame
    for (int i = 0; i < WINDOW_SIZE; i++)
    {
        vector<pair<Vector3d, Vector3d>> corres;
        corres = f_manager.getCorresponding_init(i, WINDOW_SIZE, camera_id);
        if (corres.size() > 20)
        {
            double sum_parallax = 0;
            double average_parallax;
            for (int j = 0; j < int(corres.size()); j++)
            {
                Vector2d pts_0(corres[j].first(0), corres[j].first(1));
                Vector2d pts_1(corres[j].second(0), corres[j].second(1));
                double parallax = (pts_0 - pts_1).norm();
                sum_parallax = sum_parallax + parallax;
            }
            average_parallax = 1.0 * sum_parallax / int(corres.size());
            if (average_parallax * 460 > 30 && m_estimator.solveRelativeRT(corres, relative_R, relative_T))
            {
                l = i;
                ROS_DEBUG("average_parallax %f choose l %d and newest frame to triangulate the whole structure", average_parallax * 460, l);
                return true;
            }
        }
    }
    return false;
}

void Estimator::solveOdometry(const map<int, vector<pair<int, Eigen::Matrix<double, 7, 1>>>> &image, const std_msgs::Header &header)
{
    if (frame_count < WINDOW_SIZE)
        return;
    if (solver_flag == NON_LINEAR)
    {
        TicToc t_tri;
        auto start_time = std::chrono::high_resolution_clock::now();
        f_manager.triangulate(Ps, tic, ric);
        ROS_DEBUG("triangulation costs %f", t_tri.toc());
        //        std::cout << "=================== optimization ===================" << std::endl;
        //optimization();
        vector2double();
        obs_trace(image, header);
        double2vector();
        //        std::cout << "=================== optimization_obs ===================" << std::endl;
        optimization_obs();
        auto end_time = std::chrono::high_resolution_clock::now();
        std::chrono::duration<double> time_taken = end_time - start_time;
        double seconds_taken = time_taken.count();
        std::cout << "Time taken: " << seconds_taken << " seconds" << std::endl;
    }
}

void Estimator::solveOdometry_init(const map<int, vector<pair<int, Eigen::Matrix<double, 7, 1>>>> &image, const std_msgs::Header &header)
{
    if (frame_count < WINDOW_SIZE)
        return;
    if (solver_flag == NON_LINEAR)

    {
        TicToc t_tri;

        f_manager.triangulate_init(Ps_0, tic, ric, 0);
        f_manager.triangulate_init(Ps_1, tic, ric, 1);

        optimization_cam0();
        optimization_cam1();
        /*do converage check */
        double diff_pos = 0;
        double diff_vel = 0;
        double pos_threshold = 0.1;
        double vel_threshold = 0.15;

        diff_pos = (diff_pos / WINDOW_SIZE);
        diff_vel = (diff_vel / WINDOW_SIZE);
        if (diff_pos > pos_threshold || diff_vel > vel_threshold)
        {
            obs_check(image, header);
        }
        else
        {
            double2vector_init(0);
        }
    }
}

void Estimator::vector2double()
{
    for (int i = 0; i <= WINDOW_SIZE; i++)
    {
        para_Pose[i][0] = Ps[i].x();
        para_Pose[i][1] = Ps[i].y();
        para_Pose[i][2] = Ps[i].z();
        Quaterniond q{Rs[i]};
        para_Pose[i][3] = q.x();
        para_Pose[i][4] = q.y();
        para_Pose[i][5] = q.z();
        para_Pose[i][6] = q.w();

        para_SpeedBias[i][0] = Vs[i].x();
        para_SpeedBias[i][1] = Vs[i].y();
        para_SpeedBias[i][2] = Vs[i].z();

        para_SpeedBias[i][3] = Bas[i].x();
        para_SpeedBias[i][4] = Bas[i].y();
        para_SpeedBias[i][5] = Bas[i].z();

        para_SpeedBias[i][6] = Bgs[i].x();
        para_SpeedBias[i][7] = Bgs[i].y();
        para_SpeedBias[i][8] = Bgs[i].z();
    }
    for (int i = 0; i < NUM_OF_CAM; i++)
    {
        para_Ex_Pose[i][0] = tic[i].x();
        para_Ex_Pose[i][1] = tic[i].y();
        para_Ex_Pose[i][2] = tic[i].z();
        Quaterniond q{ric[i]};
        para_Ex_Pose[i][3] = q.x();
        para_Ex_Pose[i][4] = q.y();
        para_Ex_Pose[i][5] = q.z();
        para_Ex_Pose[i][6] = q.w();
    }

    VectorXd dep = f_manager.getDepthVector();
    for (int i = 0; i < f_manager.getFeatureCount(); i++)
        para_Feature[i][0] = dep(i);
    if (ESTIMATE_TD)
        para_Td[0][0] = td;
}

void Estimator::double2vector()
{
    Vector3d origin_R0 = Utility::R2ypr(Rs[0]);
    Vector3d origin_P0 = Ps[0];

    if (failure_occur)
    {
        origin_R0 = Utility::R2ypr(last_R0);
        origin_P0 = last_P0;
        failure_occur = 0;
    }
    Vector3d origin_R00 = Utility::R2ypr(Quaterniond(para_Pose[0][6],
                                                     para_Pose[0][3],
                                                     para_Pose[0][4],
                                                     para_Pose[0][5])
                                             .toRotationMatrix());
    double y_diff = origin_R0.x() - origin_R00.x();
    // TODO
    Matrix3d rot_diff = Utility::ypr2R(Vector3d(y_diff, 0, 0));
    if (abs(abs(origin_R0.y()) - 90) < 1.0 || abs(abs(origin_R00.y()) - 90) < 1.0)
    {
        ROS_DEBUG("euler singular point!");
        rot_diff = Rs[0] * Quaterniond(para_Pose[0][6],
                                       para_Pose[0][3],
                                       para_Pose[0][4],
                                       para_Pose[0][5])
                               .toRotationMatrix()
                               .transpose();
    }

    for (int i = 0; i <= WINDOW_SIZE; i++)
    {

        Rs[i] = rot_diff * Quaterniond(para_Pose[i][6], para_Pose[i][3], para_Pose[i][4], para_Pose[i][5]).normalized().toRotationMatrix();

        Ps[i] = rot_diff * Vector3d(para_Pose[i][0] - para_Pose[0][0],
                                    para_Pose[i][1] - para_Pose[0][1],
                                    para_Pose[i][2] - para_Pose[0][2]) +
                origin_P0;

        Vs[i] = rot_diff * Vector3d(para_SpeedBias[i][0],
                                    para_SpeedBias[i][1],
                                    para_SpeedBias[i][2]);

        Bas[i] = Vector3d(para_SpeedBias[i][3],
                          para_SpeedBias[i][4],
                          para_SpeedBias[i][5]);

        Bgs[i] = Vector3d(para_SpeedBias[i][6],
                          para_SpeedBias[i][7],
                          para_SpeedBias[i][8]);
    }

    for (int i = 0; i < NUM_OF_CAM; i++)
    {
        tic[i] = Vector3d(para_Ex_Pose[i][0],
                          para_Ex_Pose[i][1],
                          para_Ex_Pose[i][2]);
        ric[i] = Quaterniond(para_Ex_Pose[i][6],
                             para_Ex_Pose[i][3],
                             para_Ex_Pose[i][4],
                             para_Ex_Pose[i][5])
                     .toRotationMatrix();
    }

    VectorXd dep = f_manager.getDepthVector();
    for (int i = 0; i < f_manager.getFeatureCount(); i++)
        dep(i) = para_Feature[i][0];
    f_manager.setDepth(dep);
    if (ESTIMATE_TD)
        td = para_Td[0][0];

    // relative info between two loop frame
    if (relocalization_info)
    {
        Matrix3d relo_r;
        Vector3d relo_t;
        relo_r = rot_diff * Quaterniond(relo_Pose[6], relo_Pose[3], relo_Pose[4], relo_Pose[5]).normalized().toRotationMatrix();
        relo_t = rot_diff * Vector3d(relo_Pose[0] - para_Pose[0][0],
                                     relo_Pose[1] - para_Pose[0][1],
                                     relo_Pose[2] - para_Pose[0][2]) +
                 origin_P0;
        double drift_correct_yaw;
        drift_correct_yaw = Utility::R2ypr(prev_relo_r).x() - Utility::R2ypr(relo_r).x();
        drift_correct_r = Utility::ypr2R(Vector3d(drift_correct_yaw, 0, 0));
        drift_correct_t = prev_relo_t - drift_correct_r * relo_t;
        relo_relative_t = relo_r.transpose() * (Ps[relo_frame_local_index] - relo_t);
        relo_relative_q = relo_r.transpose() * Rs[relo_frame_local_index];
        relo_relative_yaw = Utility::normalizeAngle(Utility::R2ypr(Rs[relo_frame_local_index]).x() - Utility::R2ypr(relo_r).x());
        // cout << "vins relo " << endl;
        // cout << "vins relative_t " << relo_relative_t.transpose() << endl;
        // cout << "vins relative_yaw " <<relo_relative_yaw << endl;
        relocalization_info = 0;
    }
}
void Estimator::vector2double_init()
{
    for (int i = 0; i <= WINDOW_SIZE; i++)
    {
        para_Pose_cam0[i][0] = Ps_0[i].x();
        para_Pose_cam0[i][1] = Ps_0[i].y();
        para_Pose_cam0[i][2] = Ps_0[i].z();

        para_Pose_cam1[i][0] = Ps_1[i].x();
        para_Pose_cam1[i][1] = Ps_1[i].y();
        para_Pose_cam1[i][2] = Ps_1[i].z();

        Quaterniond q{Rs_0[i]};
        Quaterniond q_1{Rs_0[i]};

        para_Pose_cam0[i][3] = q.x();
        para_Pose_cam0[i][4] = q.y();
        para_Pose_cam0[i][5] = q.z();
        para_Pose_cam0[i][6] = q.w();

        para_Pose_cam1[i][3] = q_1.x();
        para_Pose_cam1[i][4] = q_1.y();
        para_Pose_cam1[i][5] = q_1.z();
        para_Pose_cam1[i][6] = q_1.w();

        para_SpeedBias_cam0[i][0] = Vs_0[i].x();
        para_SpeedBias_cam0[i][1] = Vs_0[i].y();
        para_SpeedBias_cam0[i][2] = Vs_0[i].z();

        para_SpeedBias_cam0[i][3] = Bas[i].x();
        para_SpeedBias_cam0[i][4] = Bas[i].y();
        para_SpeedBias_cam0[i][5] = Bas[i].z();

        para_SpeedBias_cam0[i][6] = Bgs[i].x();
        para_SpeedBias_cam0[i][7] = Bgs[i].y();
        para_SpeedBias_cam0[i][8] = Bgs[i].z();

        para_SpeedBias_cam1[i][0] = Vs_1[i].x();
        para_SpeedBias_cam1[i][1] = Vs_1[i].y();
        para_SpeedBias_cam1[i][2] = Vs_1[i].z();

        para_SpeedBias_cam1[i][3] = Bas[i].x();
        para_SpeedBias_cam1[i][4] = Bas[i].y();
        para_SpeedBias_cam1[i][5] = Bas[i].z();

        para_SpeedBias_cam1[i][6] = Bgs[i].x();
        para_SpeedBias_cam1[i][7] = Bgs[i].y();
        para_SpeedBias_cam1[i][8] = Bgs[i].z();
    }

    for (int i = 0; i < NUM_OF_CAM; i++)
    {
        para_Ex_Pose[i][0] = tic[i].x();
        para_Ex_Pose[i][1] = tic[i].y();
        para_Ex_Pose[i][2] = tic[i].z();
        Quaterniond q{ric[i]};
        para_Ex_Pose[i][3] = q.x();
        para_Ex_Pose[i][4] = q.y();
        para_Ex_Pose[i][5] = q.z();
        para_Ex_Pose[i][6] = q.w();
    }

    VectorXd dep_0 = f_manager.getDepthVector_init(0);
    VectorXd dep_1 = f_manager.getDepthVector_init(1);
    for (int i = 0; i < f_manager.getFeatureCount(); i++)
    {
        para_Feature_cam0[i][0] = dep_0(i);
        para_Feature_cam1[i][0] = dep_1(i);
    }
    if (ESTIMATE_TD)
        para_Td[0][0] = td;
}

void Estimator::double2vector_init(int camera_id)
{
    Vector3d origin_R0 = Utility::R2ypr(Rs_0[0]);
    Vector3d origin_P0 = Ps_0[0];
    Vector3d origin_R1 = Utility::R2ypr(Rs_1[0]);
    Vector3d origin_P1 = Ps_1[0];
    if (failure_occur)
    {
        origin_R0 = Utility::R2ypr(last_R0);
        origin_P0 = last_P0;
        failure_occur = 0;
        origin_R1 = Utility::R2ypr(last_R0);
        origin_P1 = last_P0;
    }
    Vector3d origin_R00 = Utility::R2ypr(Quaterniond(para_Pose_cam0[0][6],
                                                     para_Pose_cam0[0][3],
                                                     para_Pose_cam0[0][4],
                                                     para_Pose_cam0[0][5])
                                             .toRotationMatrix());
    double y_diff = origin_R0.x() - origin_R00.x();
    // TODO
    Matrix3d rot_diff = Utility::ypr2R(Vector3d(y_diff, 0, 0));
    if (abs(abs(origin_R0.y()) - 90) < 1.0 || abs(abs(origin_R00.y()) - 90) < 1.0)
    {
        ROS_DEBUG("euler singular point!");
        rot_diff = Rs_0[0] * Quaterniond(para_Pose_cam0[0][6],
                                         para_Pose_cam0[0][3],
                                         para_Pose_cam0[0][4],
                                         para_Pose_cam0[0][5])
                                 .toRotationMatrix()
                                 .transpose();
    }

    Vector3d origin_R10 = Utility::R2ypr(Quaterniond(para_Pose_cam1[0][6],
                                                     para_Pose_cam1[0][3],
                                                     para_Pose_cam1[0][4],
                                                     para_Pose_cam1[0][5])
                                             .toRotationMatrix());
    double y1_diff = origin_R1.x() - origin_R10.x();
    // TODO
    Matrix3d rot_diff_1 = Utility::ypr2R(Vector3d(y1_diff, 0, 0));
    if (abs(abs(origin_R1.y()) - 90) < 1.0 || abs(abs(origin_R10.y()) - 90) < 1.0)
    {
        ROS_DEBUG("euler singular point!");
        rot_diff_1 = Rs_1[0] * Quaterniond(para_Pose_cam1[0][6],
                                           para_Pose_cam1[0][3],
                                           para_Pose_cam1[0][4],
                                           para_Pose_cam1[0][5])
                                   .toRotationMatrix()
                                   .transpose();
    }

    for (int i = 0; i <= WINDOW_SIZE; i++)
    {
        if (camera_id == 0)
        {
            Rs[i] = rot_diff * Quaterniond(para_Pose_cam0[i][6], para_Pose_cam0[i][3], para_Pose_cam0[i][4], para_Pose_cam0[i][5]).normalized().toRotationMatrix();

            Ps[i] = rot_diff * Vector3d(para_Pose_cam0[i][0] - para_Pose_cam0[0][0],
                                        para_Pose_cam0[i][1] - para_Pose_cam0[0][1],
                                        para_Pose_cam0[i][2] - para_Pose_cam0[0][2]) +
                    origin_P0;

            Vs[i] = rot_diff * Vector3d(para_SpeedBias_cam0[i][0],
                                        para_SpeedBias_cam0[i][1],
                                        para_SpeedBias_cam0[i][2]);
            Bas[i] = Vector3d(para_SpeedBias_cam0[i][3],
                              para_SpeedBias_cam0[i][4],
                              para_SpeedBias_cam0[i][5]);
        }
        else if (camera_id == 1)
        {
            Rs[i] = rot_diff_1 * Quaterniond(para_Pose_cam1[i][6], para_Pose_cam1[i][3], para_Pose_cam1[i][4], para_Pose_cam1[i][5]).normalized().toRotationMatrix();

            Ps[i] = rot_diff_1 * Vector3d(para_Pose_cam1[i][0] - para_Pose_cam1[0][0],
                                          para_Pose_cam1[i][1] - para_Pose_cam1[0][1],
                                          para_Pose_cam1[i][2] - para_Pose_cam1[0][2]) +
                    origin_P1;

            Vs[i] = rot_diff_1 * Vector3d(para_SpeedBias_cam1[i][0],
                                          para_SpeedBias_cam1[i][1],
                                          para_SpeedBias_cam1[i][2]);
            Bgs[i] = Vector3d(para_SpeedBias_cam1[i][6],
                              para_SpeedBias_cam1[i][7],
                              para_SpeedBias_cam1[i][8]);
        }
    }

    for (int i = 0; i < NUM_OF_CAM; i++)
    {
        tic[i] = Vector3d(para_Ex_Pose[i][0],
                          para_Ex_Pose[i][1],
                          para_Ex_Pose[i][2]);
        ric[i] = Quaterniond(para_Ex_Pose[i][6],
                             para_Ex_Pose[i][3],
                             para_Ex_Pose[i][4],
                             para_Ex_Pose[i][5])
                     .toRotationMatrix();
    }

    VectorXd dep_0 = f_manager.getDepthVector_init(0);
    VectorXd dep_1 = f_manager.getDepthVector_init(1);
    for (int i = 0; i < f_manager.getFeatureCount(); i++)
    {
        if (camera_id == 0)
        {
            dep_0(i) = para_Feature_cam0[i][0];
        }
        else if (camera_id == 1)
        {
            dep_1(i) = para_Feature_cam1[i][0];
        }
    }
    f_manager.setDepth(dep_0);
    if (ESTIMATE_TD)
        td = para_Td[0][0];
    // relative info between two loop frame
}
bool Estimator::failureDetection()
{
    if (f_manager.last_track_num < 2)
    {
        ROS_INFO(" little feature %d", f_manager.last_track_num);
        // return true;
    }
    if (Bas[WINDOW_SIZE].norm() > 2.5)
    {
        ROS_INFO(" big IMU acc bias estimation %f", Bas[WINDOW_SIZE].norm());
        return true;
    }
    if (Bgs[WINDOW_SIZE].norm() > 1.0)
    {
        ROS_INFO(" big IMU gyr bias estimation %f", Bgs[WINDOW_SIZE].norm());
        return true;
    }
    /*
    if (tic(0) > 1)
    {
        ROS_INFO(" big extri param estimation %d", tic(0) > 1);
        return true;
    }
    */
    Vector3d tmp_P = Ps[WINDOW_SIZE];
    if ((tmp_P - last_P).norm() > 5)
    {
        ROS_INFO(" big translation");
        return true;
    }
    if (abs(tmp_P.z() - last_P.z()) > 1)
    {
        ROS_INFO(" big z translation");
        return true;
    }
    Matrix3d tmp_R = Rs[WINDOW_SIZE];
    Matrix3d delta_R = tmp_R.transpose() * last_R;
    Quaterniond delta_Q(delta_R);
    double delta_angle;
    delta_angle = acos(delta_Q.w()) * 2.0 / 3.14 * 180.0;
    if (delta_angle > 50)
    {
        ROS_INFO(" big delta_angle ");
        // return true;
    }
    return false;
}

void Estimator::optimization()
{
#if two_cam_test
#endif

    ceres::Problem problem;
    ceres::LossFunction *loss_function;
    // loss_function = new ceres::HuberLoss(1.0);
    loss_function = new ceres::CauchyLoss(1.0);
    for (int i = 0; i < WINDOW_SIZE + 1; i++)
    {
        ceres::LocalParameterization *local_parameterization = new PoseLocalParameterization();
        problem.AddParameterBlock(para_Pose[i], SIZE_POSE, local_parameterization);
        problem.AddParameterBlock(para_SpeedBias[i], SIZE_SPEEDBIAS);
    }
    for (int i = 0; i < NUM_OF_CAM; i++)
    {
        ceres::LocalParameterization *local_parameterization = new PoseLocalParameterization();
        problem.AddParameterBlock(para_Ex_Pose[i], SIZE_POSE, local_parameterization);
        if (!ESTIMATE_EXTRINSIC)
        {
            ROS_DEBUG("fix extinsic param");
            problem.SetParameterBlockConstant(para_Ex_Pose[i]);
        }
        else
            ROS_DEBUG("estimate extinsic param");
    }
    if (ESTIMATE_TD)
    {
        problem.AddParameterBlock(para_Td[0], 1);
        // problem.SetParameterBlockConstant(para_Td[0]);
    }

    TicToc t_whole, t_prepare;
    vector2double();

    if (last_marginalization_info)
    {
        // construct new marginlization_factor
        MarginalizationFactor *marginalization_factor = new MarginalizationFactor(last_marginalization_info);
        problem.AddResidualBlock(marginalization_factor, NULL,
                                 last_marginalization_parameter_blocks);
    }

    for (int i = 0; i < WINDOW_SIZE; i++)
    {
        int j = i + 1;
        if (pre_integrations[j]->sum_dt > 10.0)
            continue;
        IMUFactor *imu_factor = new IMUFactor(pre_integrations[j]);
        problem.AddResidualBlock(imu_factor, NULL, para_Pose[i], para_SpeedBias[i], para_Pose[j], para_SpeedBias[j]);
    }
    int f_m_cnt = 0;
    int feature_index = -1;
    valid_feature = 0; // obs
    for (auto &it_per_id : f_manager.feature)
    {
        it_per_id.used_num = it_per_id.feature_per_frame.size();
        if (!(it_per_id.used_num >= 2 && it_per_id.start_frame < WINDOW_SIZE - 2))
            continue;

#if two_cam_test
        // the first optimize only depends on features in camesra0
        if (f_manager.main_cam == f_manager.CAM0)
        {
            if (it_per_id.feature_per_frame[0].camera_id != 0)
                continue;
        }
        else if (f_manager.main_cam == f_manager.CAM1)
            if (it_per_id.feature_per_frame[0].camera_id != 1)
                continue;
#endif
        ++feature_index;
        ++valid_feature; // obs

        int imu_i = it_per_id.start_frame, imu_j = imu_i - 1;

        Vector3d pts_i = it_per_id.feature_per_frame[0].point;

        //        std::cout << "feature_id: " << it_per_id.feature_id << std::endl;
        for (auto &it_per_frame : it_per_id.feature_per_frame)
        {
            imu_j++;
            if (imu_i == imu_j)
            {
                continue;
            }
            Vector3d pts_j = it_per_frame.point;

            if (ESTIMATE_TD)
            {
                ProjectionTdFactor *f_td = new ProjectionTdFactor(pts_i, pts_j, it_per_id.feature_per_frame[0].velocity, it_per_frame.velocity,
                                                                  it_per_id.feature_per_frame[0].cur_td, it_per_frame.cur_td,
                                                                  it_per_id.feature_per_frame[0].uv.y(), it_per_frame.uv.y());
#if two_cam_test
                problem.AddResidualBlock(f_td, loss_function, para_Pose[imu_i], para_Pose[imu_j], para_Ex_Pose[it_per_frame.camera_id], para_Feature[feature_index], para_Td[0]);
#else
                problem.AddResidualBlock(f_td, loss_function, para_Pose[imu_i], para_Pose[imu_j], para_Ex_Pose[0], para_Feature[feature_index], para_Td[0]);
#endif /*                               \
 double **para = new double *[5];       \
 para[0] = para_Pose[imu_i];            \
 para[1] = para_Pose[imu_j];            \
 para[2] = para_Ex_Pose[0];             \
 para[3] = para_Feature[feature_index]; \
 para[4] = para_Td[0];                  \
 f_td->check(para);                     \
 */
            }
            else
            {
                ProjectionFactor *f = new ProjectionFactor(pts_i, pts_j);
#if two_cam_test
                problem.AddResidualBlock(f, loss_function, para_Pose[imu_i], para_Pose[imu_j], para_Ex_Pose[it_per_frame.camera_id], para_Feature[feature_index]);
#else
                problem.AddResidualBlock(f, loss_function, para_Pose[imu_i], para_Pose[imu_j], para_Ex_Pose[0], para_Feature[feature_index]);
#endif
            }
            f_m_cnt++;
        }
    }
    // ROS_INFO("1st measurement count: %d", f_m_cnt);
    ROS_DEBUG("visual measurement count: %d", f_m_cnt);
    ROS_DEBUG("prepare for ceres: %f", t_prepare.toc());

    if (relocalization_info)
    {
        // printf("set relocalization factor! \n");
        ceres::LocalParameterization *local_parameterization = new PoseLocalParameterization();
        problem.AddParameterBlock(relo_Pose, SIZE_POSE, local_parameterization);
        int retrive_feature_index = 0;
        int feature_index = -1;
        for (auto &it_per_id : f_manager.feature)
        {
            it_per_id.used_num = it_per_id.feature_per_frame.size();
            if (!(it_per_id.used_num >= 2 && it_per_id.start_frame < WINDOW_SIZE - 2))
                continue;

#if two_cam_test
            // the first optimize only depends on features in camesra0
            if (f_manager.main_cam == f_manager.CAM0)
            {
                if (it_per_id.feature_per_frame[0].camera_id != 0)
                    continue;
            }
            else if (f_manager.main_cam == f_manager.CAM1)
                if (it_per_id.feature_per_frame[0].camera_id != 1)
                    continue;
#endif

            ++feature_index;
            int start = it_per_id.start_frame;
            if (start <= relo_frame_local_index)
            {
                while ((int)match_points[retrive_feature_index].z() < it_per_id.feature_id)
                {
                    retrive_feature_index++;
                }
                if ((int)match_points[retrive_feature_index].z() == it_per_id.feature_id)
                {
                    Vector3d pts_j = Vector3d(match_points[retrive_feature_index].x(), match_points[retrive_feature_index].y(), 1.0);
                    Vector3d pts_i = it_per_id.feature_per_frame[0].point;

                    ProjectionFactor *f = new ProjectionFactor(pts_i, pts_j);
#if two_cam_test
                    problem.AddResidualBlock(f, loss_function, para_Pose[start], relo_Pose, para_Ex_Pose[it_per_id.feature_per_frame[0].camera_id], para_Feature[feature_index]);
#else
                    problem.AddResidualBlock(f, loss_function, para_Pose[start], relo_Pose, para_Ex_Pose[0], para_Feature[feature_index]);
#endif
                    retrive_feature_index++;
                }
            }
        }
    }

    ceres::Solver::Options options;

    options.linear_solver_type = ceres::DENSE_SCHUR;
    options.num_threads = 2;
    options.trust_region_strategy_type = ceres::DOGLEG;
    options.max_num_iterations = NUM_ITERATIONS;
    // options.use_explicit_schur_complement = true;
    // options.minimizer_progress_to_stdout = true;
    // options.use_nonmonotonic_steps = true;
    if (marginalization_flag == MARGIN_OLD)
        options.max_solver_time_in_seconds = SOLVER_TIME * 4.0 / 5.0;
    else
        options.max_solver_time_in_seconds = SOLVER_TIME;
    TicToc t_solver;
    ceres::Solver::Summary summary;
    ceres::Solve(options, &problem, &summary);
    // cout << summary.BriefReport() << endl;
    ROS_DEBUG("Iterations : %d", static_cast<int>(summary.iterations.size()));
    ROS_DEBUG("solver costs: %f", t_solver.toc());

    double2vector();
}

void Estimator::optimization_obs()
{
    ceres::Problem problem;
    ceres::LossFunction *loss_function;
    // loss_function = new ceres::HuberLoss(1.0);
    loss_function = new ceres::CauchyLoss(1.0);
    for (int i = 0; i < WINDOW_SIZE + 1; i++)
    {
        ceres::LocalParameterization *local_parameterization = new PoseLocalParameterization();
        problem.AddParameterBlock(para_Pose[i], SIZE_POSE, local_parameterization);
        problem.AddParameterBlock(para_SpeedBias[i], SIZE_SPEEDBIAS);
    }
    for (int i = 0; i < NUM_OF_CAM; i++)
    {
        ceres::LocalParameterization *local_parameterization = new PoseLocalParameterization();
        problem.AddParameterBlock(para_Ex_Pose[i], SIZE_POSE, local_parameterization);
        if (!ESTIMATE_EXTRINSIC)
        {
            ROS_DEBUG("fix extinsic param");
            problem.SetParameterBlockConstant(para_Ex_Pose[i]);
        }
        else
            ROS_DEBUG("estimate extinsic param");
    }
    if (ESTIMATE_TD)
    {
        problem.AddParameterBlock(para_Td[0], 1);
        // problem.SetParameterBlockConstant(para_Td[0]);
    }

    TicToc t_whole, t_prepare;
    vector2double();

    if (last_marginalization_info)
    {
        // construct new marginlization_factor
        MarginalizationFactor *marginalization_factor = new MarginalizationFactor(last_marginalization_info);
        problem.AddResidualBlock(marginalization_factor, NULL,
                                 last_marginalization_parameter_blocks);
    }

    for (int i = 0; i < WINDOW_SIZE; i++)
    {
        int j = i + 1;
        if (pre_integrations[j]->sum_dt > 10.0)
            continue;
        IMUFactor *imu_factor = new IMUFactor(pre_integrations[j]);
        problem.AddResidualBlock(imu_factor, NULL, para_Pose[i], para_SpeedBias[i], para_Pose[j], para_SpeedBias[j]);
    }
    int f_m_cnt = 0;
    int feature_index = -1;
    // check switch main camera
    int feature_num = 0;
    int size = 0;
    size = f_manager.feature.size();
    for (auto &it_per_id : f_manager.feature)
    {
        it_per_id.used_num = it_per_id.feature_per_frame.size();
        if (!(it_per_id.used_num >= 2 && it_per_id.start_frame < WINDOW_SIZE - 2))
            continue;

#if two_cam_test
        // the first optimize only depends on features in camera0
        if (f_manager.main_cam == f_manager.CAM0)
        {
            if (it_per_id.feature_per_frame[0].camera_id != 0)
            {
                ++feature_index;
                continue;
            }
        }
        else if (f_manager.main_cam == f_manager.CAM1)
        {
            if (it_per_id.feature_per_frame[0].camera_id != 1)
            {
                ++feature_index;
                continue;
            }
        }
#endif

        ++feature_index;

        int imu_i = it_per_id.start_frame, imu_j = imu_i - 1;

        Vector3d pts_i = it_per_id.feature_per_frame[0].point;

        //        std::cout << "feature_id: " << it_per_id.feature_id << std::endl;
        for (auto &it_per_frame : it_per_id.feature_per_frame)
        {
            imu_j++;
            if (imu_i == imu_j)
            {
                continue;
            }
            Vector3d pts_j = it_per_frame.point;

            if (ESTIMATE_TD)
            {
                //                    ProjectionTdFactor *f_td = new ProjectionTdFactor(pts_i, pts_j, it_per_id.feature_per_frame[0].velocity, it_per_frame.velocity,
                //                                                                     it_per_id.feature_per_frame[0].cur_td, it_per_frame.cur_td,
                //                                                                     it_per_id.feature_per_frame[0].uv.y(), it_per_frame.uv.y());
                ProjectionTdFactor *f_td = new ProjectionTdFactor(pts_i, pts_j, it_per_id.feature_per_frame[0].velocity, it_per_frame.velocity,
                                                                  it_per_id.feature_per_frame[0].cur_td, it_per_frame.cur_td,
                                                                  it_per_id.feature_per_frame[0].uv.y(), it_per_frame.uv.y(),
                                                                  it_per_frame.obs_value, total_obs_val(), total_camera_residual());
#if two_cam_test
                problem.AddResidualBlock(f_td, loss_function, para_Pose[imu_i], para_Pose[imu_j], para_Ex_Pose[it_per_frame.camera_id], para_Feature[feature_index], para_Td[0]);
#else
                problem.AddResidualBlock(f_td, loss_function, para_Pose[imu_i], para_Pose[imu_j], para_Ex_Pose[0], para_Feature[feature_index], para_Td[0]);
#endif
                /*
                double **para = new double *[5];
                para[0] = para_Pose[imu_i];
                para[1] = para_Pose[imu_j];
                para[2] = para_Ex_Pose[0];
                para[3] = para_Feature[feature_index];
                para[4] = para_Td[0];
                f_td->check(para);
                */
            }
            else
            {
                // ProjectionFactor *f = new ProjectionFactor(pts_i, pts_j);
                ProjectionFactor *f = new ProjectionFactor(pts_i, pts_j, it_per_frame.obs_value, total_obs_val(), total_camera_residual());
#if two_cam_test
                // test
                problem.AddResidualBlock(f, loss_function, para_Pose[imu_i], para_Pose[imu_j], para_Ex_Pose[it_per_frame.camera_id], para_Feature[feature_index]);
#else
                problem.AddResidualBlock(f, loss_function, para_Pose[imu_i], para_Pose[imu_j], para_Ex_Pose[0], para_Feature[feature_index]);

#endif
            }
            f_m_cnt++;
        }
    }
    //    std::cout << "size of feature_index_obs: " << feature_index << std::endl;
    // ROS_INFO("num of features in second opt %d",feature_index);
    // ROS_INFO(" 2nd visual measurement count: %d", f_m_cnt);
    ROS_DEBUG("visual measurement count: %d", f_m_cnt);
    ROS_DEBUG("prepare for ceres: %f", t_prepare.toc());

    if (relocalization_info)
    {
        // printf("set relocalization factor! \n");
        ceres::LocalParameterization *local_parameterization = new PoseLocalParameterization();
        problem.AddParameterBlock(relo_Pose, SIZE_POSE, local_parameterization);
        int retrive_feature_index = 0;
        int feature_index = -1;
        for (auto &it_per_id : f_manager.feature)
        {
            it_per_id.used_num = it_per_id.feature_per_frame.size();
            if (!(it_per_id.used_num >= 2 && it_per_id.start_frame < WINDOW_SIZE - 2))
                continue;

#if two_cam_test
            // the first optimize only depends on features in camesra0
            if (f_manager.main_cam == f_manager.CAM0)
            {
                if (it_per_id.feature_per_frame[0].camera_id != 0)
                    continue;
            }
            else if (f_manager.main_cam == f_manager.CAM1)
                if (it_per_id.feature_per_frame[0].camera_id != 1)
                    continue;
#endif

            ++feature_index;
            int start = it_per_id.start_frame;
            if (start <= relo_frame_local_index)
            {
                while ((int)match_points[retrive_feature_index].z() < it_per_id.feature_id)
                {
                    retrive_feature_index++;
                }
                if ((int)match_points[retrive_feature_index].z() == it_per_id.feature_id)
                {
                    Vector3d pts_j = Vector3d(match_points[retrive_feature_index].x(), match_points[retrive_feature_index].y(), 1.0);
                    Vector3d pts_i = it_per_id.feature_per_frame[0].point;

                    ProjectionFactor *f = new ProjectionFactor(pts_i, pts_j);
#if two_cam_test
                    problem.AddResidualBlock(f, loss_function, para_Pose[start], relo_Pose, para_Ex_Pose[it_per_id.feature_per_frame[0].camera_id], para_Feature[feature_index]);
#else
                    problem.AddResidualBlock(f, loss_function, para_Pose[start], relo_Pose, para_Ex_Pose[0], para_Feature[feature_index]);
#endif
                    retrive_feature_index++;
                }
            }
        }
    }

    ceres::Solver::Options options;

    options.linear_solver_type = ceres::DENSE_SCHUR;
    options.num_threads = 2;
    options.trust_region_strategy_type = ceres::DOGLEG;
    options.max_num_iterations = NUM_ITERATIONS;
    // options.use_explicit_schur_complement = true;
    // options.minimizer_progress_to_stdout = true;
    // options.use_nonmonotonic_steps = true;
    if (marginalization_flag == MARGIN_OLD)
        options.max_solver_time_in_seconds = SOLVER_TIME * 4.0 / 5.0;
    else
        options.max_solver_time_in_seconds = SOLVER_TIME;
    TicToc t_solver;
    ceres::Solver::Summary summary;
    ceres::Solve(options, &problem, &summary);
    // cout << summary.BriefReport() << endl;
    ROS_DEBUG("Iterations : %d", static_cast<int>(summary.iterations.size()));
    ROS_DEBUG("solver costs: %f", t_solver.toc());

    double2vector();

    TicToc t_whole_marginalization;
    if (marginalization_flag == MARGIN_OLD)
    {
        MarginalizationInfo *marginalization_info = new MarginalizationInfo();
        vector2double();

        if (last_marginalization_info)
        {
            vector<int> drop_set;
            for (int i = 0; i < static_cast<int>(last_marginalization_parameter_blocks.size()); i++)
            {
                if (last_marginalization_parameter_blocks[i] == para_Pose[0] ||
                    last_marginalization_parameter_blocks[i] == para_SpeedBias[0])
                    drop_set.push_back(i);
            }
            // construct new marginlization_factor
            MarginalizationFactor *marginalization_factor = new MarginalizationFactor(last_marginalization_info);
            ResidualBlockInfo *residual_block_info = new ResidualBlockInfo(marginalization_factor, NULL,
                                                                           last_marginalization_parameter_blocks,
                                                                           drop_set);

            marginalization_info->addResidualBlockInfo(residual_block_info);
        }

        {
            if (pre_integrations[1]->sum_dt < 10.0)
            {
                IMUFactor *imu_factor = new IMUFactor(pre_integrations[1]);
                ResidualBlockInfo *residual_block_info = new ResidualBlockInfo(imu_factor, NULL,
                                                                               vector<double *>{para_Pose[0], para_SpeedBias[0], para_Pose[1], para_SpeedBias[1]},
                                                                               vector<int>{0, 1});
                marginalization_info->addResidualBlockInfo(residual_block_info);
            }
        }

        {
            int feature_index = -1;
            for (auto &it_per_id : f_manager.feature)
            {
                it_per_id.used_num = it_per_id.feature_per_frame.size();
                if (!(it_per_id.used_num >= 2 && it_per_id.start_frame < WINDOW_SIZE - 2))
                    continue;

#if two_cam_test
                // the first optimize only depends on features in camesra0
                if (f_manager.main_cam == f_manager.CAM0)
                {
                    if (it_per_id.feature_per_frame[0].camera_id != 0)
                        continue;
                }
                else if (f_manager.main_cam == f_manager.CAM1)
                    if (it_per_id.feature_per_frame[0].camera_id != 1)
                        continue;
#endif

                ++feature_index;

                int imu_i = it_per_id.start_frame, imu_j = imu_i - 1;
                if (imu_i != 0)
                    continue;

                Vector3d pts_i = it_per_id.feature_per_frame[0].point;

                for (auto &it_per_frame : it_per_id.feature_per_frame)
                {
                    imu_j++;
                    if (imu_i == imu_j)
                        continue;

                    Vector3d pts_j = it_per_frame.point;
                    if (ESTIMATE_TD)
                    {
                        ProjectionTdFactor *f_td = new ProjectionTdFactor(pts_i, pts_j, it_per_id.feature_per_frame[0].velocity, it_per_frame.velocity,
                                                                          it_per_id.feature_per_frame[0].cur_td, it_per_frame.cur_td,
                                                                          it_per_id.feature_per_frame[0].uv.y(), it_per_frame.uv.y());
#if two_cam_test
                        // test
                        ResidualBlockInfo *residual_block_info = new ResidualBlockInfo(f_td, loss_function,
                                                                                       vector<double *>{para_Pose[imu_i], para_Pose[imu_j], para_Ex_Pose[it_per_frame.camera_id], para_Feature[feature_index], para_Td[0]},
                                                                                       vector<int>{0, 3});
#else
                        ResidualBlockInfo *residual_block_info = new ResidualBlockInfo(f_td, loss_function,
                                                                                       vector<double *>{para_Pose[imu_i], para_Pose[imu_j], para_Ex_Pose[0], para_Feature[feature_index], para_Td[0]},
                                                                                       vector<int>{0, 3});
#endif
                        marginalization_info->addResidualBlockInfo(residual_block_info);
                    }
                    else
                    {
                        ProjectionFactor *f = new ProjectionFactor(pts_i, pts_j);
#if two_cam_test
                        // test
                        ResidualBlockInfo *residual_block_info = new ResidualBlockInfo(f, loss_function,
                                                                                       vector<double *>{para_Pose[imu_i], para_Pose[imu_j], para_Ex_Pose[it_per_frame.camera_id], para_Feature[feature_index]},
                                                                                       vector<int>{0, 3});
#else
                        ResidualBlockInfo *residual_block_info = new ResidualBlockInfo(f, loss_function,
                                                                                       vector<double *>{para_Pose[imu_i], para_Pose[imu_j], para_Ex_Pose[0], para_Feature[feature_index]},
                                                                                       vector<int>{0, 3});
#endif
                        marginalization_info->addResidualBlockInfo(residual_block_info);
                    }
                }
            }
        }

        TicToc t_pre_margin;
        marginalization_info->preMarginalize();
        ROS_DEBUG("pre marginalization %f ms", t_pre_margin.toc());

        TicToc t_margin;
        marginalization_info->marginalize();
        ROS_DEBUG("marginalization %f ms", t_margin.toc());

        std::unordered_map<long, double *> addr_shift;
        for (int i = 1; i <= WINDOW_SIZE; i++)
        {
            addr_shift[reinterpret_cast<long>(para_Pose[i])] = para_Pose[i - 1];
            addr_shift[reinterpret_cast<long>(para_SpeedBias[i])] = para_SpeedBias[i - 1];
        }
        for (int i = 0; i < NUM_OF_CAM; i++)
            addr_shift[reinterpret_cast<long>(para_Ex_Pose[i])] = para_Ex_Pose[i];
        if (ESTIMATE_TD)
        {
            addr_shift[reinterpret_cast<long>(para_Td[0])] = para_Td[0];
        }
        vector<double *> parameter_blocks = marginalization_info->getParameterBlocks(addr_shift);

        if (last_marginalization_info)
            delete last_marginalization_info;
        last_marginalization_info = marginalization_info;
        last_marginalization_parameter_blocks = parameter_blocks;
    }
    else
    {
        if (last_marginalization_info &&
            std::count(std::begin(last_marginalization_parameter_blocks), std::end(last_marginalization_parameter_blocks), para_Pose[WINDOW_SIZE - 1]))
        {

            MarginalizationInfo *marginalization_info = new MarginalizationInfo();
            vector2double();
            if (last_marginalization_info)
            {
                vector<int> drop_set;
                for (int i = 0; i < static_cast<int>(last_marginalization_parameter_blocks.size()); i++)
                {
                    ROS_ASSERT(last_marginalization_parameter_blocks[i] != para_SpeedBias[WINDOW_SIZE - 1]);
                    if (last_marginalization_parameter_blocks[i] == para_Pose[WINDOW_SIZE - 1])
                        drop_set.push_back(i);
                }
                // construct new marginlization_factor
                MarginalizationFactor *marginalization_factor = new MarginalizationFactor(last_marginalization_info);
                ResidualBlockInfo *residual_block_info = new ResidualBlockInfo(marginalization_factor, NULL,
                                                                               last_marginalization_parameter_blocks,
                                                                               drop_set);

                marginalization_info->addResidualBlockInfo(residual_block_info);
            }

            TicToc t_pre_margin;
            ROS_DEBUG("begin marginalization");
            marginalization_info->preMarginalize();
            ROS_DEBUG("end pre marginalization, %f ms", t_pre_margin.toc());

            TicToc t_margin;
            ROS_DEBUG("begin marginalization");
            marginalization_info->marginalize();
            ROS_DEBUG("end marginalization, %f ms", t_margin.toc());

            std::unordered_map<long, double *> addr_shift;
            for (int i = 0; i <= WINDOW_SIZE; i++)
            {
                if (i == WINDOW_SIZE - 1)
                    continue;
                else if (i == WINDOW_SIZE)
                {
                    addr_shift[reinterpret_cast<long>(para_Pose[i])] = para_Pose[i - 1];
                    addr_shift[reinterpret_cast<long>(para_SpeedBias[i])] = para_SpeedBias[i - 1];
                }
                else
                {
                    addr_shift[reinterpret_cast<long>(para_Pose[i])] = para_Pose[i];
                    addr_shift[reinterpret_cast<long>(para_SpeedBias[i])] = para_SpeedBias[i];
                }
            }
            for (int i = 0; i < NUM_OF_CAM; i++)
                addr_shift[reinterpret_cast<long>(para_Ex_Pose[i])] = para_Ex_Pose[i];
            if (ESTIMATE_TD)
            {
                addr_shift[reinterpret_cast<long>(para_Td[0])] = para_Td[0];
            }

            vector<double *> parameter_blocks = marginalization_info->getParameterBlocks(addr_shift);
            if (last_marginalization_info)
                delete last_marginalization_info;
            last_marginalization_info = marginalization_info;
            last_marginalization_parameter_blocks = parameter_blocks;
        }
    }
    ROS_DEBUG("whole marginalization costs: %f", t_whole_marginalization.toc());

    ROS_DEBUG("whole time for ceres: %f", t_whole.toc());
}

void Estimator::slideWindow()
{
    TicToc t_margin;
    if (marginalization_flag == MARGIN_OLD)
    {
        double t_0 = Headers[0].stamp.toSec();
        back_R0 = Rs[0];
        back_P0 = Ps[0];
        if (frame_count == WINDOW_SIZE)
        {
            for (int i = 0; i < WINDOW_SIZE; i++)
            {
                Rs[i].swap(Rs[i + 1]);

                std::swap(pre_integrations[i], pre_integrations[i + 1]);

                dt_buf[i].swap(dt_buf[i + 1]);
                linear_acceleration_buf[i].swap(linear_acceleration_buf[i + 1]);
                angular_velocity_buf[i].swap(angular_velocity_buf[i + 1]);

                Headers[i] = Headers[i + 1];
                Ps[i].swap(Ps[i + 1]);
                Vs[i].swap(Vs[i + 1]);
                Bas[i].swap(Bas[i + 1]);
                Bgs[i].swap(Bgs[i + 1]);
            }
            Headers[WINDOW_SIZE] = Headers[WINDOW_SIZE - 1];
            Ps[WINDOW_SIZE] = Ps[WINDOW_SIZE - 1];
            Vs[WINDOW_SIZE] = Vs[WINDOW_SIZE - 1];
            Rs[WINDOW_SIZE] = Rs[WINDOW_SIZE - 1];
            Bas[WINDOW_SIZE] = Bas[WINDOW_SIZE - 1];
            Bgs[WINDOW_SIZE] = Bgs[WINDOW_SIZE - 1];

            delete pre_integrations[WINDOW_SIZE];
            pre_integrations[WINDOW_SIZE] = new IntegrationBase{acc_0, gyr_0, Bas[WINDOW_SIZE], Bgs[WINDOW_SIZE]};

            dt_buf[WINDOW_SIZE].clear();
            linear_acceleration_buf[WINDOW_SIZE].clear();
            angular_velocity_buf[WINDOW_SIZE].clear();

            if (true || solver_flag == INITIAL)
            {
                map<double, ImageFrame>::iterator it_0;
                it_0 = all_image_frame.find(t_0);
                delete it_0->second.pre_integration;
                it_0->second.pre_integration = nullptr;

                for (map<double, ImageFrame>::iterator it = all_image_frame.begin(); it != it_0; ++it)
                {
                    if (it->second.pre_integration)
                        delete it->second.pre_integration;
                    it->second.pre_integration = NULL;
                }

                all_image_frame.erase(all_image_frame.begin(), it_0);
                all_image_frame.erase(t_0);
            }
            slideWindowOld();
        }
    }
    else
    {
        if (frame_count == WINDOW_SIZE)
        {
            for (unsigned int i = 0; i < dt_buf[frame_count].size(); i++)
            {
                double tmp_dt = dt_buf[frame_count][i];
                Vector3d tmp_linear_acceleration = linear_acceleration_buf[frame_count][i];
                Vector3d tmp_angular_velocity = angular_velocity_buf[frame_count][i];

                pre_integrations[frame_count - 1]->push_back(tmp_dt, tmp_linear_acceleration, tmp_angular_velocity);

                dt_buf[frame_count - 1].push_back(tmp_dt);
                linear_acceleration_buf[frame_count - 1].push_back(tmp_linear_acceleration);
                angular_velocity_buf[frame_count - 1].push_back(tmp_angular_velocity);
            }

            Headers[frame_count - 1] = Headers[frame_count];
            Ps[frame_count - 1] = Ps[frame_count];
            Vs[frame_count - 1] = Vs[frame_count];
            Rs[frame_count - 1] = Rs[frame_count];
            Bas[frame_count - 1] = Bas[frame_count];
            Bgs[frame_count - 1] = Bgs[frame_count];

            delete pre_integrations[WINDOW_SIZE];
            pre_integrations[WINDOW_SIZE] = new IntegrationBase{acc_0, gyr_0, Bas[WINDOW_SIZE], Bgs[WINDOW_SIZE]};

            dt_buf[WINDOW_SIZE].clear();
            linear_acceleration_buf[WINDOW_SIZE].clear();
            angular_velocity_buf[WINDOW_SIZE].clear();

            slideWindowNew();
        }
    }
}

// real marginalization is removed in solve_ceres()
void Estimator::slideWindowNew()
{
    sum_of_front++;
    f_manager.removeFront(frame_count);
}
// real marginalization is removed in solve_ceres()
void Estimator::slideWindowOld()
{
    sum_of_back++;

    bool shift_depth = solver_flag == NON_LINEAR ? true : false;
    if (shift_depth)
    {
        Matrix3d R0, R1;
        Vector3d P0, P1;

#if two_cam_test
        if (f_manager.main_cam == f_manager.CAM0)
        {
            R0 = back_R0 * ric[0];
            R1 = Rs[0] * ric[0];
            P0 = back_P0 + back_R0 * tic[0];
            P1 = Ps[0] + Rs[0] * tic[0];
        }
        else if (f_manager.main_cam == f_manager.CAM1)
        {
            R0 = back_R0 * ric[1];
            R1 = Rs[0] * ric[1];
            P0 = back_P0 + back_R0 * tic[1];
            P1 = Ps[0] + Rs[0] * tic[1];
        }
#else
        R0 = back_R0 * ric[0];
        R1 = Rs[0] * ric[0];
        P0 = back_P0 + back_R0 * tic[0];
        P1 = Ps[0] + Rs[0] * tic[0];
#endif

        f_manager.removeBackShiftDepth(R0, P0, R1, P1);
    }
    else
        f_manager.removeBack();
}

void Estimator::setReloFrame(double _frame_stamp, int _frame_index, vector<Vector3d> &_match_points, Vector3d _relo_t, Matrix3d _relo_r)
{
    relo_frame_stamp = _frame_stamp;
    relo_frame_index = _frame_index;
    match_points.clear();
    match_points = _match_points;
    prev_relo_t = _relo_t;
    prev_relo_r = _relo_r;
    for (int i = 0; i < WINDOW_SIZE; i++)
    {
        if (relo_frame_stamp == Headers[i].stamp.toSec())
        {
            relo_frame_local_index = i;
            relocalization_info = 1;
            for (int j = 0; j < SIZE_POSE; j++)
                relo_Pose[j] = para_Pose[i][j];
        }
    }
}

void Estimator::obs_trace(const map<int, vector<pair<int, Eigen::Matrix<double, 7, 1>>>> &image, const std_msgs::Header &header)
{
    // initialization step
#if two_cam_test
    sameId_pts[0].clear();
    sameId_pts[1].clear();
    pts_observability[0].clear();
    pts_observability[1].clear();
    pts_all_observability.clear();
    Eigen::MatrixXd H[2] = {MatrixXd(2 * f_manager.last_track_num, 9), MatrixXd(2 * f_manager.last_track_num, 9)}; // maybe the matrix is too large for one frame
    // pointwise method
    Eigen::MatrixXd H_all(2 * f_manager.last_track_num, 9);
    H_all.setZero();
    H[0].setZero();
    H[1].setZero();
    // check the number of filted feature point
    int num_filted_point = 0;
    obs_sum = 0;
#else
    sameId_pts.clear();
    pts_observability.clear();
    Eigen::MatrixXd H(2 * f_manager.last_track_num, 9);
    H.setZero();
#endif

    // 把f_manager中這一貞image有出現的image feature都提取出來存在tracked_feature
    // current image
    //vector2double();
    list<FeaturePerId> tracked_feature;
    for (auto &it_pts : image)
    {
        int feature_id = it_pts.first;
        auto it = find_if(f_manager.feature.begin(), f_manager.feature.end(), [feature_id](FeaturePerId &it)
                          { return it.feature_id == feature_id; });
        if (it->feature_id == feature_id)
        {
            tracked_feature.push_back(*it);
        }
    }

    int feature_index = -1;
#if two_cam_test
    int tracked_feature_num[2] = {0};
    int tracked_all_feature_num = 0;
#else
    int tracked_feature_num = 0; // the index is for H
#endif
    for (auto &it_per_id : f_manager.feature)
    {
        it_per_id.used_num = it_per_id.feature_per_frame.size();

#if two_cam_test
        int camera_id = it_per_id.feature_per_frame[0].camera_id;
        assert(camera_id == 0 || camera_id == 1);
#endif

        // 以下條件會導致當前貞中的某些（灰色）點被濾除
        if (!(it_per_id.used_num >= 2 && it_per_id.start_frame < WINDOW_SIZE - 2))
            continue;

        // current image
        int feature_id = it_per_id.feature_id;
        auto it = find_if(tracked_feature.begin(), tracked_feature.end(), [feature_id](FeaturePerId &it)
                          { return it.feature_id == feature_id; });
        if (it == tracked_feature.end()) // similar to (it->feature_id != feature_id)
        {
            feature_index++;
            continue;
        }

        int imu_i = it_per_id.start_frame;
        int imu_j = it_per_id.endFrame();
        Vector3d pts_i = it_per_id.feature_per_frame[imu_i].point;
        Vector3d pts_j = it_per_id.feature_per_frame[imu_j].point;

        Vector3d Pi(para_Pose[imu_i][0], para_Pose[imu_i][1], para_Pose[imu_i][2]);
        Quaterniond Qi(para_Pose[imu_i][6], para_Pose[imu_i][3], para_Pose[imu_i][4], para_Pose[imu_i][5]);

        Vector3d Pj(para_Pose[imu_j][0], para_Pose[imu_j][1], para_Pose[imu_j][2]);
        Quaterniond Qj(para_Pose[imu_j][6], para_Pose[imu_j][3], para_Pose[imu_j][4], para_Pose[imu_j][5]);

#if two_cam_test
        Vector3d tic0(para_Ex_Pose[0][0], para_Ex_Pose[0][1], para_Ex_Pose[0][2]);
        Vector3d tic1(para_Ex_Pose[1][0], para_Ex_Pose[1][1], para_Ex_Pose[1][2]);
        Quaterniond qic0(para_Ex_Pose[0][6], para_Ex_Pose[0][3], para_Ex_Pose[0][4], para_Ex_Pose[0][5]);
        Quaterniond qic1(para_Ex_Pose[1][6], para_Ex_Pose[1][3], para_Ex_Pose[1][4], para_Ex_Pose[1][5]);
        Matrix3d ric0 = qic0.toRotationMatrix();
        Matrix3d ric1 = qic1.toRotationMatrix();
#else
        Vector3d tic(para_Ex_Pose[0][0], para_Ex_Pose[0][1], para_Ex_Pose[0][2]);
        Quaterniond qic(para_Ex_Pose[0][6], para_Ex_Pose[0][3], para_Ex_Pose[0][4], para_Ex_Pose[0][5]);
        Matrix3d ric = qic.toRotationMatrix();
#endif

        Matrix3d Ri = Qi.toRotationMatrix();
        Matrix3d Rj = Qj.toRotationMatrix();

        // double inv_dep_i = para_Feature[feature_index][0];
        double inv_dep_i = 1;
        double inv_dep_j = 1;
        Vector3d pts_camera_i = pts_i / inv_dep_i;
        Vector3d pts_camera_j = pts_j / inv_dep_j;
        
#if two_cam_test
        // Vector3d pts_imu_i;
        // Vector3d pts_w;
        // Vector3d pts_imu_j;
        // Vector3d pts_camera_j;
        // if (camera_id == 0)
        // {
        //     pts_imu_i = qic0 * pts_camera_i + tic0;
        //     pts_w = Qi * pts_imu_i + Pi;
        //     pts_imu_j = Qj.inverse() * (pts_w - Pj);
        //     pts_camera_j = qic0.inverse() * (pts_imu_j - tic0);
        // }
        // else if (camera_id == 1)
        // {
        //     pts_imu_i = qic1 * pts_camera_i + tic1;
        //     pts_w = Qi * pts_imu_i + Pi;
        //     pts_imu_j = Qj.inverse() * (pts_w - Pj);
        //     pts_camera_j = qic1.inverse() * (pts_imu_j - tic1);
        // }
#else
        Vector3d pts_imu_i = qic * pts_camera_i + tic;
        Vector3d pts_w = Qi * pts_imu_i + Pi;
        Vector3d pts_imu_j = Qj.inverse() * (pts_w - Pj);
        Vector3d pts_camera_j = qic.inverse() * (pts_imu_j - tic);
#endif
        //double dep_j = pts_camera_j.z();
        double dep_j = 1;
        // 不知道為什麼會有 dep_j = 0
        //    if(dep_j == 0.0)
        //      continue;

        MatrixXd reduce(2, 3);

        reduce << 1 / dep_j, 0., -pts_camera_j(0) / (dep_j * dep_j),
            0., 1 / dep_j, -pts_camera_j(1) / (dep_j * dep_j);

#if two_cam_test
        Matrix3d R_c_b;
        if (camera_id == 0)
            R_c_b = ric0.transpose();
        else if (camera_id == 1)
            R_c_b = ric1.transpose();
#else
        Matrix3d R_c_b = ric.transpose();
#endif
        Matrix3d R_bj_w = Rj.transpose();
        Matrix3d R_w_bi = Ri;
        double lamda_l = inv_dep_i;
        debug_lamda_l = lamda_l; // debug

        Vector3d p_w_bi = Pi;
        Vector3d p_w_bj = Pj;
#if two_cam_test
        Vector3d P_b_c;
        if (camera_id == 0)
            P_b_c = tic0;
        else if (camera_id == 1)
            P_b_c = tic1;
#else
        Vector3d P_b_c = tic;
#endif

        // from rc matrix
        Eigen::Matrix3d temp_top3 = (-1.0 * R_c_b * R_bj_w).transpose();
        Eigen::Matrix3d temp_mid3 = (R_c_b * Utility::skewSymmetric(R_bj_w * (R_w_bi * (R_c_b.transpose() * 1.0 / lamda_l * pts_i + P_b_c) + p_w_bi - p_w_bj))).transpose();

        // construct 9 x 3 matrix;
        Eigen::MatrixXd temp_right_matrix(9, 3);
        temp_right_matrix.setZero();
        temp_right_matrix.block<3, 3>(0, 0) = temp_top3;
        temp_right_matrix.block<3, 3>(3, 0) = temp_mid3;

        // construct 2 x 9 matrix;
        Eigen::MatrixXd rc_Matrix(2, 9);
        rc_Matrix.setZero();
        rc_Matrix = reduce * temp_right_matrix.transpose();

        // Bug - nan value
        if (isnan((rc_Matrix.transpose() * rc_Matrix).trace()))
        {
            feature_index++;
            continue;
        }
        // filter the value outside the 3rd standard derivatives
#if two_cam_test
        // if (pts_observability[camera_id].size() >2 && abs((rc_Matrix.transpose() * rc_Matrix).trace() - mean[camera_id]) > 3*stdev[camera_id])
        if (pts_all_observability.size() > 2 && abs((rc_Matrix.transpose() * rc_Matrix).trace() - all_mean) > 3 * all_stdev)
#endif
        {
            feature_index++;
            continue;
        }
#if two_cam_test
        sameId_pts[camera_id].push_back(make_pair(feature_id, cv::Point2f(image.find(feature_id)->second[0].second(3, 0), image.find(feature_id)->second[0].second(4, 0))));
        // pts_observability[camera_id].push_back((rc_Matrix.transpose() * rc_Matrix).trace());
        pts_all_observability.push_back((rc_Matrix.transpose() * rc_Matrix).trace());
#else
        sameId_pts.push_back(make_pair(feature_id, cv::Point2f(image.find(feature_id)->second[0].second(3, 0), image.find(feature_id)->second[0].second(4, 0))));
        pts_observability.push_back((rc_Matrix.transpose() * rc_Matrix).trace());
#endif
        it_per_id.feature_per_frame[it_per_id.feature_per_frame.size() - 1].obs_value = (rc_Matrix.transpose() * rc_Matrix).trace();

        // save the current standard derivatives
#if two_cam_test
        // for pointwise method

        obs_sum = accumulate(pts_all_observability.begin(), pts_all_observability.end(), 0.0);
        all_mean = obs_sum / pts_all_observability.size();
        vector<double> diff_all(pts_all_observability.size());
        transform(pts_all_observability.begin(), pts_all_observability.end(), diff_all.begin(), bind2nd(minus<double>(), all_mean));
        double sq_all_sum = inner_product(diff_all.begin(), diff_all.end(), diff_all.begin(), 0.0);
        all_stdev = std::sqrt(sq_all_sum / pts_all_observability.size());

        H_all.block<2, 9>((2 * tracked_all_feature_num), 0) = rc_Matrix;
        tracked_all_feature_num++;
        feature_index++;

#else
        // double sum = accumulate(pts_observability.begin(), pts_observability.end(), 0.0);
        mean = sum / pts_observability.size();
        vector<double> diff(pts_observability.size());
        transform(pts_observability.begin(), pts_observability.end(), diff.begin(), bind2nd(minus<double>(), mean));
        double sq_sum = inner_product(diff.begin(), diff.end(), diff.begin(), 0.0);
        stdev = std::sqrt(sq_sum / pts_observability.size());

        H.block<2, 9>((2 * tracked_feature_num), 0) = rc_Matrix;
        tracked_feature_num++;
        feature_index++;
#endif
    }
#if two_cam_test
    // for pointwise method
    //double2vector();
    MatrixXd CostFunction_all(9, 9);
    CostFunction_all = H_all.transpose() * H_all / tracked_all_feature_num;
    double OBS_ALL = CostFunction_all.trace();
    // std::cout << "OBS_ALL= " << OBS_ALL << std::endl;
#else
    MatrixXd CostFunction(9, 9);
    CostFunction = H.transpose() * H / tracked_feature_num;
    Observation_matrix_trace = CostFunction.trace();
#endif
}
// calculate  the sum of all sliding window feature points
double Estimator::total_obs_val()
{
    double total_obs = 0.0;
    for (auto &it_per_id : f_manager.feature)
    {
        it_per_id.used_num = it_per_id.feature_per_frame.size();
        if (!(it_per_id.used_num >= 2 && it_per_id.start_frame < WINDOW_SIZE - 2))
            continue;

#if two_cam_test
        // the first optimize only depends on features in camesra0
        if (f_manager.main_cam == f_manager.CAM0)
        {
            if (it_per_id.feature_per_frame[0].camera_id != 0)
                continue;
        }
        else if (f_manager.main_cam == f_manager.CAM1)
            if (it_per_id.feature_per_frame[0].camera_id != 1)
                continue;
#endif

        for (auto &it_per_frame : it_per_id.feature_per_frame)
        {
            total_obs += it_per_frame.obs_value;
        }
    }

    return total_obs;
}
// calculate the number of feature in
int Estimator::total_camera_residual()
{
    int camera_residual = 0;
    for (auto &it_per_id : f_manager.feature)
    {
        it_per_id.used_num = it_per_id.feature_per_frame.size();
        if (!(it_per_id.used_num >= 2 && it_per_id.start_frame < WINDOW_SIZE - 2))
            continue;

#if two_cam_test
        // the first optimize only depends on features in camesra0
        if (f_manager.main_cam == f_manager.CAM0)
        {
            if (it_per_id.feature_per_frame[0].camera_id != 0)
                continue;
        }
        else if (f_manager.main_cam == f_manager.CAM1)
            if (it_per_id.feature_per_frame[0].camera_id != 1)
                continue;
#endif

        int imu_i = it_per_id.start_frame, imu_j = imu_i - 1;

        for (auto &it_per_frame : it_per_id.feature_per_frame)
        {
            imu_j++;
            if (imu_i == imu_j)
            {
                continue;
            }
            camera_residual++;
        }
    }
    return camera_residual;
}

void Estimator::optimization_cam0()
{
#if two_cam_test
#endif

    ceres::Problem problem;
    ceres::LossFunction *loss_function;
    // loss_function = new ceres::HuberLoss(1.0);
    loss_function = new ceres::CauchyLoss(1.0);
    for (int i = 0; i < WINDOW_SIZE + 1; i++)
    {
        ceres::LocalParameterization *local_parameterization = new PoseLocalParameterization();
        problem.AddParameterBlock(para_Pose_cam0[i], SIZE_POSE, local_parameterization);
        problem.AddParameterBlock(para_SpeedBias_cam0[i], SIZE_SPEEDBIAS);
    }
    for (int i = 0; i < NUM_OF_CAM; i++)
    {
        ceres::LocalParameterization *local_parameterization = new PoseLocalParameterization();
        problem.AddParameterBlock(para_Ex_Pose[i], SIZE_POSE, local_parameterization);
        if (!ESTIMATE_EXTRINSIC)
        {
            ROS_DEBUG("fix extinsic param");
            problem.SetParameterBlockConstant(para_Ex_Pose[i]);
        }
        else
            ROS_DEBUG("estimate extinsic param");
    }
    if (ESTIMATE_TD)
    {
        problem.AddParameterBlock(para_Td[0], 1);
    }

    TicToc t_whole, t_prepare;
    vector2double_init();

    if (last_marginalization_info)
    {
        // construct new marginlization_factor
        MarginalizationFactor *marginalization_factor = new MarginalizationFactor(last_marginalization_info);
        problem.AddResidualBlock(marginalization_factor, NULL,
                                 last_marginalization_parameter_blocks);
    }

    for (int i = 0; i < WINDOW_SIZE; i++)
    {
        int j = i + 1;
        if (pre_integrations[j]->sum_dt > 10.0)
            continue;
        IMUFactor *imu_factor = new IMUFactor(pre_integrations[j]);
        problem.AddResidualBlock(imu_factor, NULL, para_Pose_cam0[i], para_SpeedBias_cam0[i], para_Pose_cam0[j], para_SpeedBias_cam0[j]);
    }
    int f_m_cnt = 0;
    int feature_index = -1;
    valid_feature = 0; // obs
    for (auto &it_per_id : f_manager.feature)
    {
        it_per_id.used_num = it_per_id.feature_per_frame.size();
        if (!(it_per_id.used_num >= 2 && it_per_id.start_frame < WINDOW_SIZE - 2))
            continue;

#if two_cam_test
        // the first optimize only depends on features in camesra0
        if (f_manager.main_cam == f_manager.CAM0)
        {
            if (it_per_id.feature_per_frame[0].camera_id != 0)
                continue;
        }
        else if (f_manager.main_cam == f_manager.CAM1)
            if (it_per_id.feature_per_frame[0].camera_id != 1)
                continue;
#endif
        ++feature_index;
        ++valid_feature; // obs

        int imu_i = it_per_id.start_frame, imu_j = imu_i - 1;

        Vector3d pts_i = it_per_id.feature_per_frame[0].point;

        //        std::cout << "feature_id: " << it_per_id.feature_id << std::endl;
        for (auto &it_per_frame : it_per_id.feature_per_frame)
        {
            imu_j++;
            if (imu_i == imu_j)
            {
                continue;
            }
            Vector3d pts_j = it_per_frame.point;

            if (ESTIMATE_TD)
            {
                ProjectionTdFactor *f_td = new ProjectionTdFactor(pts_i, pts_j, it_per_id.feature_per_frame[0].velocity, it_per_frame.velocity,
                                                                  it_per_id.feature_per_frame[0].cur_td, it_per_frame.cur_td,
                                                                  it_per_id.feature_per_frame[0].uv.y(), it_per_frame.uv.y());
#if two_cam_test
                problem.AddResidualBlock(f_td, loss_function, para_Pose_cam0[imu_i], para_Pose_cam0[imu_j], para_Ex_Pose[it_per_frame.camera_id], para_Feature[feature_index], para_Td[0]);
#else
                problem.AddResidualBlock(f_td, loss_function, para_Pose[imu_i], para_Pose[imu_j], para_Ex_Pose[0], para_Feature[feature_index], para_Td[0]);
#endif /*                               \
 double **para = new double *[5];       \
 para[0] = para_Pose[imu_i];            \
 para[1] = para_Pose[imu_j];            \
 para[2] = para_Ex_Pose[0];             \
 para[3] = para_Feature[feature_index]; \
 para[4] = para_Td[0];                  \
 f_td->check(para);                     \
 */
            }
            else
            {
                ProjectionFactor *f = new ProjectionFactor(pts_i, pts_j);
#if two_cam_test
                problem.AddResidualBlock(f, loss_function, para_Pose_cam0[imu_i], para_Pose_cam0[imu_j], para_Ex_Pose[it_per_frame.camera_id], para_Feature_cam0[feature_index]);
#else
                problem.AddResidualBlock(f, loss_function, para_Pose[imu_i], para_Pose[imu_j], para_Ex_Pose[0], para_Feature[feature_index]);
#endif
            }
            f_m_cnt++;
        }
    }
    // ROS_INFO("1st measurement count: %d", f_m_cnt);
    ROS_DEBUG("visual measurement count: %d", f_m_cnt);
    ROS_DEBUG("prepare for ceres: %f", t_prepare.toc());

    ceres::Solver::Options options;

    options.linear_solver_type = ceres::DENSE_SCHUR;
    options.trust_region_strategy_type = ceres::DOGLEG;
    options.max_num_iterations = NUM_ITERATIONS;
    if (marginalization_flag == MARGIN_OLD)
    {
        options.max_solver_time_in_seconds = SOLVER_TIME * 4.0 / 5.0;
    }
    else
    {
        options.max_solver_time_in_seconds = SOLVER_TIME;
    }
    TicToc t_solver;
    ceres::Solver::Summary summary;
    ceres::Solve(options, &problem, &summary);
    // cout << summary.BriefReport() << endl;
    ROS_DEBUG("Iterations : %d", static_cast<int>(summary.iterations.size()));
    ROS_DEBUG("solver costs: %f", t_solver.toc());
    // double2vector_0();
    std::cout << "complete cam0 initialization" << std::endl;
}

void Estimator::optimization_cam1()
{
#if two_cam_test
#endif

    ceres::Problem problem;
    ceres::LossFunction *loss_function;
    // loss_function = new ceres::HuberLoss(1.0);
    loss_function = new ceres::CauchyLoss(1.0);
    for (int i = 0; i < WINDOW_SIZE + 1; i++)
    {
        ceres::LocalParameterization *local_parameterization = new PoseLocalParameterization();
        problem.AddParameterBlock(para_Pose_cam1[i], SIZE_POSE, local_parameterization);
        problem.AddParameterBlock(para_SpeedBias_cam1[i], SIZE_SPEEDBIAS);
    }
    for (int i = 0; i < NUM_OF_CAM; i++)
    {
        ceres::LocalParameterization *local_parameterization = new PoseLocalParameterization();
        problem.AddParameterBlock(para_Ex_Pose[i], SIZE_POSE, local_parameterization);
        if (!ESTIMATE_EXTRINSIC)
        {
            ROS_DEBUG("fix extinsic param");
            problem.SetParameterBlockConstant(para_Ex_Pose[i]);
        }
        else
            ROS_DEBUG("estimate extinsic param");
    }
    if (ESTIMATE_TD)
    {
        problem.AddParameterBlock(para_Td[0], 1);
    }

    TicToc t_whole, t_prepare;
    vector2double_init();

    if (last_marginalization_info)
    {
        // construct new marginlization_factor
        MarginalizationFactor *marginalization_factor = new MarginalizationFactor(last_marginalization_info);
        problem.AddResidualBlock(marginalization_factor, NULL,
                                 last_marginalization_parameter_blocks);
    }

    for (int i = 0; i < WINDOW_SIZE; i++)
    {
        int j = i + 1;
        if (pre_integrations[j]->sum_dt > 10.0)
            continue;
        IMUFactor *imu_factor = new IMUFactor(pre_integrations[j]);
        problem.AddResidualBlock(imu_factor, NULL, para_Pose_cam1[i], para_SpeedBias_cam1[i], para_Pose_cam1[j], para_SpeedBias_cam1[j]);
    }
    int f_m_cnt = 0;
    int feature_index = -1;
    valid_feature = 0; // obs
    for (auto &it_per_id : f_manager.feature)
    {
        it_per_id.used_num = it_per_id.feature_per_frame.size();
        if (!(it_per_id.used_num >= 2 && it_per_id.start_frame < WINDOW_SIZE - 2))
            continue;

#if two_cam_test
        // the first optimize only depends on features in camesra0
        if (f_manager.main_cam == f_manager.CAM0)
        {
            if (it_per_id.feature_per_frame[0].camera_id != 0)
                continue;
        }
        else if (f_manager.main_cam == f_manager.CAM1)
            if (it_per_id.feature_per_frame[0].camera_id != 1)
                continue;
#endif
        ++feature_index;
        ++valid_feature; // obs

        int imu_i = it_per_id.start_frame, imu_j = imu_i - 1;

        Vector3d pts_i = it_per_id.feature_per_frame[0].point;

        //        std::cout << "feature_id: " << it_per_id.feature_id << std::endl;
        for (auto &it_per_frame : it_per_id.feature_per_frame)
        {
            imu_j++;
            if (imu_i == imu_j)
            {
                continue;
            }
            Vector3d pts_j = it_per_frame.point;

            if (ESTIMATE_TD)
            {
                ProjectionTdFactor *f_td = new ProjectionTdFactor(pts_i, pts_j, it_per_id.feature_per_frame[0].velocity, it_per_frame.velocity,
                                                                  it_per_id.feature_per_frame[0].cur_td, it_per_frame.cur_td,
                                                                  it_per_id.feature_per_frame[0].uv.y(), it_per_frame.uv.y());
#if two_cam_test
                problem.AddResidualBlock(f_td, loss_function, para_Pose_cam1[imu_i], para_Pose_cam1[imu_j], para_Ex_Pose[it_per_frame.camera_id], para_Feature_cam1[feature_index], para_Td[0]);
#else
                problem.AddResidualBlock(f_td, loss_function, para_Pose[imu_i], para_Pose[imu_j], para_Ex_Pose[0], para_Feature[feature_index], para_Td[0]);
#endif /*                               \
 double **para = new double *[5];       \
 para[0] = para_Pose[imu_i];            \
 para[1] = para_Pose[imu_j];            \
 para[2] = para_Ex_Pose[0];             \
 para[3] = para_Feature[feature_index]; \
 para[4] = para_Td[0];                  \
 f_td->check(para);                     \
 */
            }
            else
            {
                ProjectionFactor *f = new ProjectionFactor(pts_i, pts_j);
#if two_cam_test
                problem.AddResidualBlock(f, loss_function, para_Pose_cam1[imu_i], para_Pose_cam1[imu_j], para_Ex_Pose[it_per_frame.camera_id], para_Feature_cam1[feature_index]);
#else
                problem.AddResidualBlock(f, loss_function, para_Pose[imu_i], para_Pose[imu_j], para_Ex_Pose[0], para_Feature[feature_index]);
#endif
            }
            f_m_cnt++;
        }
    }
    // ROS_INFO("1st measurement count: %d", f_m_cnt);
    ROS_DEBUG("visual measurement count: %d", f_m_cnt);
    ROS_DEBUG("prepare for ceres: %f", t_prepare.toc());

    ceres::Solver::Options options;

    options.linear_solver_type = ceres::DENSE_SCHUR;
    options.trust_region_strategy_type = ceres::DOGLEG;
    options.max_num_iterations = NUM_ITERATIONS;
    if (marginalization_flag == MARGIN_OLD)
    {
        options.max_solver_time_in_seconds = SOLVER_TIME * 4.0 / 5.0;
    }
    else
    {
        options.max_solver_time_in_seconds = SOLVER_TIME;
    }
    TicToc t_solver;
    ceres::Solver::Summary summary;
    ceres::Solve(options, &problem, &summary);
    // cout << summary.BriefReport() << endl;
    ROS_DEBUG("Iterations : %d", static_cast<int>(summary.iterations.size()));
    ROS_DEBUG("solver costs: %f", t_solver.toc());
    std::cout << "complete cam1 initialization" << std::endl;
}

void Estimator::obs_check(const map<int, vector<pair<int, Eigen::Matrix<double, 7, 1>>>> &image, const std_msgs::Header &header)
{
    // initialization step
#if two_cam_test
    sameId_pts[0].clear();
    sameId_pts[1].clear();
    pts_observability[0].clear();
    pts_observability[1].clear();
    pts_all_observability.clear();
    Eigen::MatrixXd H[2] = {MatrixXd(2 * f_manager.last_track_num, 9), MatrixXd(2 * f_manager.last_track_num, 9)}; // maybe the matrix is too large for one frame
    // pointwise method
    Eigen::MatrixXd H_all(2 * f_manager.last_track_num, 9);
    H_all.setZero();
    H[0].setZero();
    H[1].setZero();
    // check the number of filted feature point
    int num_filted_point = 0;
    obs_sum = 0;
#else
    sameId_pts.clear();
    pts_observability.clear();
    Eigen::MatrixXd H(2 * f_manager.last_track_num, 9);
    H.setZero();
#endif

    // 把f_manager中這一貞image有出現的image feature都提取出來存在tracked_feature
    // current image
    list<FeaturePerId> tracked_feature;
    for (auto &it_pts : image)
    {
        int feature_id = it_pts.first;
        auto it = find_if(f_manager.feature.begin(), f_manager.feature.end(), [feature_id](FeaturePerId &it)
                          { return it.feature_id == feature_id; });
        if (it->feature_id == feature_id)
        {
            tracked_feature.push_back(*it);
        }
    }

    int feature_index = -1;
#if two_cam_test
    int tracked_feature_num[2] = {0};
    int tracked_all_feature_num = 0;
#else
    int tracked_feature_num = 0; // the index is for H
#endif
    for (auto &it_per_id : f_manager.feature)
    {
        it_per_id.used_num = it_per_id.feature_per_frame.size();

#if two_cam_test
        int camera_id = it_per_id.feature_per_frame[0].camera_id;
        assert(camera_id == 0 || camera_id == 1);
#endif

        // 以下條件會導致當前貞中的某些（灰色）點被濾除
        if (!(it_per_id.used_num >= 2 && it_per_id.start_frame < WINDOW_SIZE - 2))
            continue;

        // current image
        int feature_id = it_per_id.feature_id;
        auto it = find_if(tracked_feature.begin(), tracked_feature.end(), [feature_id](FeaturePerId &it)
                          { return it.feature_id == feature_id; });
        if (it == tracked_feature.end()) // similar to (it->feature_id != feature_id)
        {
            feature_index++;
            continue;
        }

        int imu_i = it_per_id.start_frame;
        int imu_j = it_per_id.endFrame();
        Vector3d pts_i = it_per_id.feature_per_frame[imu_i].point;
        //    Vector3d pts_j = it_per_id.feature_per_frame[imu_j].point;
        // two initial guess
        Vector3d Pi_0(para_Pose_cam0[imu_i][0], para_Pose_cam0[imu_i][1], para_Pose_cam0[imu_i][2]);
        Quaterniond Qi_0(para_Pose_cam0[imu_i][6], para_Pose_cam0[imu_i][3], para_Pose_cam0[imu_i][4], para_Pose_cam0[imu_i][5]);

        Vector3d Pi_1(para_Pose_cam1[imu_i][0], para_Pose_cam1[imu_i][1], para_Pose_cam1[imu_i][2]);
        Quaterniond Qi_1(para_Pose_cam1[imu_i][6], para_Pose_cam1[imu_i][3], para_Pose_cam1[imu_i][4], para_Pose_cam1[imu_i][5]);

        Vector3d Pj_0(para_Pose_cam0[imu_j][0], para_Pose_cam0[imu_j][1], para_Pose_cam0[imu_j][2]);
        Quaterniond Qj_0(para_Pose_cam0[imu_j][6], para_Pose_cam0[imu_j][3], para_Pose_cam0[imu_j][4], para_Pose_cam0[imu_j][5]);

        Vector3d Pj_1(para_Pose_cam1[imu_j][0], para_Pose_cam1[imu_j][1], para_Pose_cam1[imu_j][2]);
        Quaterniond Qj_1(para_Pose_cam1[imu_j][6], para_Pose_cam1[imu_j][3], para_Pose_cam1[imu_j][4], para_Pose_cam1[imu_j][5]);

#if two_cam_test
        Vector3d tic0(para_Ex_Pose[0][0], para_Ex_Pose[0][1], para_Ex_Pose[0][2]);
        Vector3d tic1(para_Ex_Pose[1][0], para_Ex_Pose[1][1], para_Ex_Pose[1][2]);
        Quaterniond qic0(para_Ex_Pose[0][6], para_Ex_Pose[0][3], para_Ex_Pose[0][4], para_Ex_Pose[0][5]);
        Quaterniond qic1(para_Ex_Pose[1][6], para_Ex_Pose[1][3], para_Ex_Pose[1][4], para_Ex_Pose[1][5]);
        Matrix3d ric0 = qic0.toRotationMatrix();
        Matrix3d ric1 = qic1.toRotationMatrix();
#else
        Vector3d tic(para_Ex_Pose[0][0], para_Ex_Pose[0][1], para_Ex_Pose[0][2]);
        Quaterniond qic(para_Ex_Pose[0][6], para_Ex_Pose[0][3], para_Ex_Pose[0][4], para_Ex_Pose[0][5]);
        Matrix3d ric = qic.toRotationMatrix();
#endif

        Matrix3d Ri_0 = Qi_0.toRotationMatrix();
        Matrix3d Rj_0 = Qj_0.toRotationMatrix();

        Matrix3d Ri_1 = Qi_1.toRotationMatrix();
        Matrix3d Rj_1 = Qj_1.toRotationMatrix();

        double inv_dep_i_cam0 = para_Feature_cam0[feature_index][0];

        double inv_dep_i_cam1 = para_Feature_cam1[feature_index][0];

        Vector3d pts_camera_i_0 = pts_i / inv_dep_i_cam0;

        Vector3d pts_camera_i_1 = pts_i / inv_dep_i_cam1;
#if two_cam_test
        Vector3d pts_imu_i;
        Vector3d pts_w;
        Vector3d pts_imu_j;
        Vector3d pts_camera_j;

        if (camera_id == 0)
        {
            pts_imu_i = qic0 * pts_camera_i_0 + tic0;
            pts_w = Qi_0 * pts_imu_i + Pi_0;
            pts_imu_j = Qj_0.inverse() * (pts_w - Pj_0);
            pts_camera_j = qic0.inverse() * (pts_imu_j - tic0);
        }
        else if (camera_id == 1)
        {
            pts_imu_i = qic1 * pts_camera_i_1 + tic1;
            pts_w = Qi_1 * pts_imu_i + Pi_1;
            pts_imu_j = Qj_1.inverse() * (pts_w - Pj_1);
            pts_camera_j = qic1.inverse() * (pts_imu_j - tic1);
        }
#else
        Vector3d pts_imu_i = qic * pts_camera_i + tic;
        Vector3d pts_w = Qi * pts_imu_i + Pi;
        Vector3d pts_imu_j = Qj.inverse() * (pts_w - Pj);
        Vector3d pts_camera_j = qic.inverse() * (pts_imu_j - tic);
#endif
        double dep_j = pts_camera_j.z();

        MatrixXd reduce(2, 3);
        reduce << 1 / dep_j, 0., -pts_camera_j(0) / (dep_j * dep_j),
            0., 1 / dep_j, -pts_camera_j(1) / (dep_j * dep_j);

#if two_cam_test
        Matrix3d R_c_b;
        if (camera_id == 0)
            R_c_b = ric0.transpose();
        else if (camera_id == 1)
            R_c_b = ric1.transpose();
#else
        Matrix3d R_c_b = ric.transpose();
#endif
        Matrix3d R_bj_w;
        Matrix3d R_w_bi;
        double lamda_l;
        debug_lamda_l; // debug
        Vector3d p_w_bi;
        Vector3d p_w_bj;

        if (camera_id == 0)
        {
            R_bj_w = Rj_0.transpose();
            R_w_bi = Ri_0;
            lamda_l = inv_dep_i_cam0;
            debug_lamda_l = lamda_l; // debug

            p_w_bi = Pi_0;
            p_w_bj = Pj_0;
        }
        else if (camera_id == 1)
        {
            R_bj_w = Rj_1.transpose();
            R_w_bi = Ri_1;
            lamda_l = inv_dep_i_cam1;
            debug_lamda_l = lamda_l; // debug

            p_w_bi = Pi_1;
            p_w_bj = Pj_1;
        }

#if two_cam_test
        Vector3d P_b_c;
        if (camera_id == 0)
            P_b_c = tic0;
        else if (camera_id == 1)
            P_b_c = tic1;
#else
        Vector3d P_b_c = tic;
#endif

        // from rc matrix
        Eigen::Matrix3d temp_top3 = (-1.0 * R_c_b * R_bj_w).transpose();
        Eigen::Matrix3d temp_mid3 = (R_c_b * Utility::skewSymmetric(R_bj_w * (R_w_bi * (R_c_b.transpose() * 1.0 / lamda_l * pts_i + P_b_c) + p_w_bi - p_w_bj))).transpose();

        // construct 9 x 3 matrix;
        Eigen::MatrixXd temp_right_matrix(9, 3);
        temp_right_matrix.setZero();
        temp_right_matrix.block<3, 3>(0, 0) = temp_top3;
        temp_right_matrix.block<3, 3>(3, 0) = temp_mid3;

        // construct 2 x 9 matrix;
        Eigen::MatrixXd rc_Matrix(2, 9);
        rc_Matrix.setZero();
        rc_Matrix = reduce * temp_right_matrix.transpose();

        // Bug - nan value
        if (isnan((rc_Matrix.transpose() * rc_Matrix).trace()))
        {
            feature_index++;
            continue;
        }

#if two_cam_test
        pts_observability[camera_id].push_back((rc_Matrix.transpose() * rc_Matrix).trace());
#else
        sameId_pts.push_back(make_pair(feature_id, cv::Point2f(image.find(feature_id)->second[0].second(3, 0), image.find(feature_id)->second[0].second(4, 0))));
        pts_observability.push_back((rc_Matrix.transpose() * rc_Matrix).trace());
#endif
        it_per_id.feature_per_frame[it_per_id.feature_per_frame.size() - 1].obs_value = (rc_Matrix.transpose() * rc_Matrix).trace();

        // save the current standard derivatives
#if two_cam_test

        double sum = accumulate(pts_observability[camera_id].begin(), pts_observability[camera_id].end(), 0.0);
        mean[camera_id] = sum / pts_observability[camera_id].size();
        vector<double> diff(pts_observability[camera_id].size());
        transform(pts_observability[camera_id].begin(), pts_observability[camera_id].end(), diff.begin(), bind2nd(minus<double>(), mean[camera_id]));
        double sq_sum = inner_product(diff.begin(), diff.end(), diff.begin(), 0.0);
        stdev[camera_id] = std::sqrt(sq_sum / pts_observability[camera_id].size());

        H[camera_id].block<2, 9>((2 * tracked_feature_num[camera_id]), 0) = rc_Matrix;
        tracked_feature_num[camera_id]++;
        // H_all.block<2,9>((2 * tracked_all_feature_num),0) = rc_Matrix;
        tracked_all_feature_num++;
        feature_index++;

#else
        // double sum = accumulate(pts_observability.begin(), pts_observability.end(), 0.0);
        mean = sum / pts_observability.size();
        vector<double> diff(pts_observability.size());
        transform(pts_observability.begin(), pts_observability.end(), diff.begin(), bind2nd(minus<double>(), mean));
        double sq_sum = inner_product(diff.begin(), diff.end(), diff.begin(), 0.0);
        stdev = std::sqrt(sq_sum / pts_observability.size());

        H.block<2, 9>((2 * tracked_feature_num), 0) = rc_Matrix;
        tracked_feature_num++;
        feature_index++;
#endif
    }
#if two_cam_test

    for (int i = 0; i < NUM_OF_CAM; i++)
    {
        MatrixXd CostFunction(9, 9);
        VectorXd diagonal;
        CostFunction = H[i].transpose() * H[i] / tracked_feature_num[i];
        Observation_matrix_trace[i] = CostFunction.trace();
    }
    if (Observation_matrix_trace[0] >= Observation_matrix_trace[1])
        double2vector_init(0);
    else
        double2vector_init(1);
#else
    MatrixXd CostFunction(9, 9);
    CostFunction = H.transpose() * H / tracked_feature_num;
    Observation_matrix_trace = CostFunction.trace();
#endif
}