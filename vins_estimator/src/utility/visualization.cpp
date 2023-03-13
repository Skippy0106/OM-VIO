#include "visualization.h"
#include "std_msgs/Float64.h"
#include <cv_bridge/cv_bridge.h>

using namespace ros;
using namespace Eigen;
ros::Publisher pub_odometry, pub_latest_odometry;
ros::Publisher pub_path, pub_relo_path;
ros::Publisher pub_point_cloud, pub_margin_cloud;
ros::Publisher pub_cam0_point_cloud,pub_cam1_point_cloud;
ros::Publisher pub_key_poses;
ros::Publisher pub_relo_relative_pose;
ros::Publisher pub_camera_pose;
ros::Publisher pub_camera_pose_visual,pub_camera1_pose_visual;
ros::Publisher pub_observation_matrix;
ros::Publisher pub_sameId;
nav_msgs::Path path, relo_path;
#if two_cam_test
ros::Publisher pub_odometry_cam0, pub_odometry_cam1;
#endif

ros::Publisher pub_keyframe_pose;
ros::Publisher pub_keyframe_point;
ros::Publisher pub_extrinsic;

CameraPoseVisualization cameraposevisual(0, 1, 0, 1);
CameraPoseVisualization keyframebasevisual(0.0, 0.0, 1.0, 1.0);
static double sum_of_path = 0;
static Vector3d last_path(0.0, 0.0, 0.0);

void registerPub(ros::NodeHandle &n)
{
    pub_latest_odometry = n.advertise<nav_msgs::Odometry>("imu_propagate", 1000);
    pub_path = n.advertise<nav_msgs::Path>("path", 1000);
    pub_relo_path = n.advertise<nav_msgs::Path>("relocalization_path", 1000);
    pub_odometry = n.advertise<nav_msgs::Odometry>("odometry", 1000);
    //pub_point_cloud = n.advertise<sensor_msgs::PointCloud>("point_cloud", 1000);
    pub_cam0_point_cloud = n.advertise<sensor_msgs::PointCloud>("cam0_point_cloud", 1000);
    pub_cam1_point_cloud = n.advertise<sensor_msgs::PointCloud>("cam1_point_cloud", 1000);
    pub_margin_cloud = n.advertise<sensor_msgs::PointCloud>("history_cloud", 1000);
    pub_key_poses = n.advertise<visualization_msgs::Marker>("key_poses", 1000);
    pub_camera_pose = n.advertise<nav_msgs::Odometry>("camera_pose", 1000);
    pub_camera_pose_visual = n.advertise<visualization_msgs::MarkerArray>("camera_pose_visual", 1000);
    pub_keyframe_pose = n.advertise<nav_msgs::Odometry>("keyframe_pose", 1000);
    pub_keyframe_point = n.advertise<sensor_msgs::PointCloud>("keyframe_point", 1000);
    pub_extrinsic = n.advertise<nav_msgs::Odometry>("extrinsic", 1000);
    pub_relo_relative_pose=  n.advertise<nav_msgs::Odometry>("relo_relative_pose", 1000);
    pub_observation_matrix = n.advertise<std_msgs::Float64>("observation_matrix", 1000);
    pub_sameId = n.advertise<sensor_msgs::Image>("sameId_feature_img",1000);
#if two_cam_test
    pub_odometry_cam0 = n.advertise<nav_msgs::Odometry>("odometry_cam0", 1000);
    pub_odometry_cam1 = n.advertise<nav_msgs::Odometry>("odometry_cam1", 1000);
#endif

    cameraposevisual.setScale(1);
    cameraposevisual.setLineWidth(0.05);
    keyframebasevisual.setScale(0.1);
    keyframebasevisual.setLineWidth(0.01);
}

void pubLatestOdometry(const Eigen::Vector3d &P, const Eigen::Quaterniond &Q, const Eigen::Vector3d &V, const std_msgs::Header &header)
{
    Eigen::Quaterniond quadrotor_Q = Q ;

    nav_msgs::Odometry odometry;
    odometry.header = header;
    odometry.header.frame_id = "world";
    odometry.pose.pose.position.x = P.x();
    odometry.pose.pose.position.y = P.y();
    odometry.pose.pose.position.z = P.z();
    odometry.pose.pose.orientation.x = quadrotor_Q.x();
    odometry.pose.pose.orientation.y = quadrotor_Q.y();
    odometry.pose.pose.orientation.z = quadrotor_Q.z();
    odometry.pose.pose.orientation.w = quadrotor_Q.w();
    odometry.twist.twist.linear.x = V.x();
    odometry.twist.twist.linear.y = V.y();
    odometry.twist.twist.linear.z = V.z();
    pub_latest_odometry.publish(odometry);
}

void printStatistics(const Estimator &estimator, double t)
{
    if (estimator.solver_flag != Estimator::SolverFlag::NON_LINEAR)
        return;
    //printf("position: %f, %f, %f\r", estimator.Ps[WINDOW_SIZE].x(), estimator.Ps[WINDOW_SIZE].y(), estimator.Ps[WINDOW_SIZE].z());
    ROS_DEBUG_STREAM("position: " << estimator.Ps[WINDOW_SIZE].transpose());
    ROS_DEBUG_STREAM("orientation: " << estimator.Vs[WINDOW_SIZE].transpose());
    for (int i = 0; i < NUM_OF_CAM; i++)
    {
        //ROS_DEBUG("calibration result for camera %d", i);
        ROS_DEBUG_STREAM("extirnsic tic: " << estimator.tic[i].transpose());
        ROS_DEBUG_STREAM("extrinsic ric: " << Utility::R2ypr(estimator.ric[i]).transpose());
        if (ESTIMATE_EXTRINSIC)
        {
            cv::FileStorage fs(EX_CALIB_RESULT_PATH, cv::FileStorage::WRITE);
            Eigen::Matrix3d eigen_R;
            Eigen::Vector3d eigen_T;
            eigen_R = estimator.ric[i];
            eigen_T = estimator.tic[i];
            cv::Mat cv_R, cv_T;
            cv::eigen2cv(eigen_R, cv_R);
            cv::eigen2cv(eigen_T, cv_T);
            fs << "extrinsicRotation" << cv_R << "extrinsicTranslation" << cv_T;
            fs.release();
        }
    }

    static double sum_of_time = 0;
    static int sum_of_calculation = 0;
    sum_of_time += t;
    sum_of_calculation++;
    ROS_DEBUG("vo solver costs: %f ms", t);
    ROS_DEBUG("average of time %f ms", sum_of_time / sum_of_calculation);

    sum_of_path += (estimator.Ps[WINDOW_SIZE] - last_path).norm();
    last_path = estimator.Ps[WINDOW_SIZE];
    ROS_DEBUG("sum of path %f", sum_of_path);
    if (ESTIMATE_TD)
        ROS_INFO("td %f", estimator.td);
}

void pubOdometry(const Estimator &estimator, const std_msgs::Header &header)
{
    if (estimator.solver_flag == Estimator::SolverFlag::NON_LINEAR)
    {
        nav_msgs::Odometry odometry;
        odometry.header = header;
        odometry.header.frame_id = "world";
        odometry.child_frame_id = "world";
        Quaterniond tmp_Q;
        tmp_Q = Quaterniond(estimator.Rs[WINDOW_SIZE]);
        odometry.pose.pose.position.x = estimator.Ps[WINDOW_SIZE].x();
        odometry.pose.pose.position.y = estimator.Ps[WINDOW_SIZE].y();
        odometry.pose.pose.position.z = estimator.Ps[WINDOW_SIZE].z();
        odometry.pose.pose.orientation.x = tmp_Q.x();
        odometry.pose.pose.orientation.y = tmp_Q.y();
        odometry.pose.pose.orientation.z = tmp_Q.z();
        odometry.pose.pose.orientation.w = tmp_Q.w();
        odometry.twist.twist.linear.x = estimator.Vs[WINDOW_SIZE].x();
        odometry.twist.twist.linear.y = estimator.Vs[WINDOW_SIZE].y();
        odometry.twist.twist.linear.z = estimator.Vs[WINDOW_SIZE].z();
        pub_odometry.publish(odometry);

#if two_cam_test
        if (estimator.Observation_matrix_trace[0] > estimator.Observation_matrix_trace[1]) pub_odometry_cam0.publish(odometry);
        else pub_odometry_cam1.publish(odometry);
#endif

        geometry_msgs::PoseStamped pose_stamped;
        pose_stamped.header = header;
        pose_stamped.header.frame_id = "world";
        pose_stamped.pose = odometry.pose.pose;
        path.header = header;
        path.header.frame_id = "world";
        path.poses.push_back(pose_stamped);
        pub_path.publish(path);

        Vector3d correct_t;
        Vector3d correct_v;
        Quaterniond correct_q;
        correct_t = estimator.drift_correct_r * estimator.Ps[WINDOW_SIZE] + estimator.drift_correct_t;
        correct_q = estimator.drift_correct_r * estimator.Rs[WINDOW_SIZE];
        odometry.pose.pose.position.x = correct_t.x();
        odometry.pose.pose.position.y = correct_t.y();
        odometry.pose.pose.position.z = correct_t.z();
        odometry.pose.pose.orientation.x = correct_q.x();
        odometry.pose.pose.orientation.y = correct_q.y();
        odometry.pose.pose.orientation.z = correct_q.z();
        odometry.pose.pose.orientation.w = correct_q.w();

        pose_stamped.pose = odometry.pose.pose;
        relo_path.header = header;
        relo_path.header.frame_id = "world";
        relo_path.poses.push_back(pose_stamped);
        pub_relo_path.publish(relo_path);

        // write result to file
        ofstream foutC(VINS_RESULT_PATH, ios::app);
        foutC.setf(ios::fixed, ios::floatfield);
        foutC.precision(0);
        foutC << header.stamp.toSec() * 1e9 << ",";
        foutC.precision(5);
        foutC << estimator.Ps[WINDOW_SIZE].x() << ","
              << estimator.Ps[WINDOW_SIZE].y() << ","
              << estimator.Ps[WINDOW_SIZE].z() << ","
              << tmp_Q.w() << ","
              << tmp_Q.x() << ","
              << tmp_Q.y() << ","
              << tmp_Q.z() << ","
              << estimator.Vs[WINDOW_SIZE].x() << ","
              << estimator.Vs[WINDOW_SIZE].y() << ","
              << estimator.Vs[WINDOW_SIZE].z() << "," << endl;
        foutC.close();
    }
}

void pubKeyPoses(const Estimator &estimator, const std_msgs::Header &header)
{
    if (estimator.key_poses.size() == 0)
        return;
    visualization_msgs::Marker key_poses;
    key_poses.header = header;
    key_poses.header.frame_id = "world";
    key_poses.ns = "key_poses";
    key_poses.type = visualization_msgs::Marker::SPHERE_LIST;
    key_poses.action = visualization_msgs::Marker::ADD;
    key_poses.pose.orientation.w = 1.0;
    key_poses.lifetime = ros::Duration();

    //static int key_poses_id = 0;
    key_poses.id = 0; //key_poses_id++;
    key_poses.scale.x = 0.05;
    key_poses.scale.y = 0.05;
    key_poses.scale.z = 0.05;
    key_poses.color.r = 1.0;
    key_poses.color.a = 1.0;

    for (int i = 0; i <= WINDOW_SIZE; i++)
    {
        geometry_msgs::Point pose_marker;
        Vector3d correct_pose;
        correct_pose = estimator.key_poses[i];
        pose_marker.x = correct_pose.x();
        pose_marker.y = correct_pose.y();
        pose_marker.z = correct_pose.z();
        key_poses.points.push_back(pose_marker);
    }
    pub_key_poses.publish(key_poses);
}

void pubCameraPose(const Estimator &estimator, const std_msgs::Header &header)
{
    int idx2 = WINDOW_SIZE - 1;

    if (estimator.solver_flag == Estimator::SolverFlag::NON_LINEAR)
    {
        int i = idx2;
        //int main_cam = estimator.f_manager.main_cam;
        //int main_cam = 0;
        Vector3d P = estimator.Ps[i] + estimator.Rs[i] * estimator.tic[0];
        Quaterniond R = Quaterniond(estimator.Rs[i] * estimator.ric[0]);
        nav_msgs::Odometry odometry,odometry_1;
        odometry.header = header;
        odometry.header.frame_id = "world";
        odometry.pose.pose.position.x = P.x();
        odometry.pose.pose.position.y = P.y();
        odometry.pose.pose.position.z = P.z();
        odometry.pose.pose.orientation.x = R.x();
        odometry.pose.pose.orientation.y = R.y();
        odometry.pose.pose.orientation.z = R.z();
        odometry.pose.pose.orientation.w = R.w();

        pub_camera_pose.publish(odometry);

        cameraposevisual.reset();
        cameraposevisual.add_pose(P, R);
        cameraposevisual.publish_by(pub_camera_pose_visual, odometry.header);
    }
}
void pubPointCloud(const Estimator &estimator, const std_msgs::Header &header)
{
    sensor_msgs::PointCloud point_cloud, loop_point_cloud,cam0_point_cloud,cam1_point_cloud;
    point_cloud.header = header;
    cam0_point_cloud.header=header;
    cam1_point_cloud.header=header;
    loop_point_cloud.header = header;

    //int main_cam = estimator.f_manager.main_cam;
    int main_cam = 0;
    int feature_num=0;
    int cam0_feature_num=0;
    int cam1_feature_num=0;
    for (auto &it_per_id : estimator.f_manager.feature)
    {
        int used_num;
        used_num = it_per_id.feature_per_frame.size();
        if (!(used_num >= 2 && it_per_id.start_frame < WINDOW_SIZE - 2))
            continue;
        if (it_per_id.start_frame > WINDOW_SIZE * 3.0 / 4.0 || it_per_id.solve_flag != 1)
            continue;
        if (it_per_id.feature_per_frame[0].camera_id==0)
        {
            int imu_i = it_per_id.start_frame;
            Vector3d pts_i = it_per_id.feature_per_frame[0].point * it_per_id.estimated_depth;
            Vector3d w_pts_i = estimator.Rs[imu_i] * (estimator.ric[0] * pts_i + estimator.tic[0]) + estimator.Ps[imu_i];
            geometry_msgs::Point32 p;
            p.x = w_pts_i(0);
            p.y = w_pts_i(1);
            p.z = w_pts_i(2);
            cam0_point_cloud.points.push_back(p);
            ++ cam0_feature_num;
        }
        else
        {
            int imu_i = it_per_id.start_frame;
            Vector3d pts_i = it_per_id.feature_per_frame[0].point * it_per_id.estimated_depth;
            Vector3d w_pts_i = estimator.Rs[imu_i] * (estimator.ric[1] * pts_i + estimator.tic[1]) + estimator.Ps[imu_i];
            geometry_msgs::Point32 p;
            p.x = w_pts_i(0);
            p.y = w_pts_i(1);
            p.z = w_pts_i(2);
            cam1_point_cloud.points.push_back(p);
            ++ cam1_feature_num;
        }
/* 
        int imu_i = it_per_id.start_frame;
        Vector3d pts_i = it_per_id.feature_per_frame[0].point * it_per_id.estimated_depth;
        Vector3d w_pts_i = estimator.Rs[imu_i] * (estimator.ric[main_cam] * pts_i + estimator.tic[main_cam]) + estimator.Ps[imu_i];

        geometry_msgs::Point32 p;
        p.x = w_pts_i(0);
        p.y = w_pts_i(1);
        p.z = w_pts_i(2);
        point_cloud.points.push_back(p);
        ++ feature_num; */
        ++ feature_num;
    }
    //ROS_INFO("From CAM0 %d",cam0_feature_num);
    //ROS_INFO("From CAM1 %d",cam1_feature_num);
    //ROS_INFO("Feature num %d",feature_num);
    pub_cam0_point_cloud.publish(cam0_point_cloud);
    pub_cam1_point_cloud.publish(cam1_point_cloud);


    // pub margined potin
    sensor_msgs::PointCloud margin_cloud;
    margin_cloud.header = header;

    for (auto &it_per_id : estimator.f_manager.feature)
    { 
        int used_num;
        used_num = it_per_id.feature_per_frame.size();
        if (!(used_num >= 2 && it_per_id.start_frame < WINDOW_SIZE - 2))
            continue;
        //if (it_per_id->start_frame > WINDOW_SIZE * 3.0 / 4.0 || it_per_id->solve_flag != 1)
        //        continue;

        if (it_per_id.start_frame == 0 && it_per_id.feature_per_frame.size() <= 2 
            && it_per_id.solve_flag == 1 )
        {
            int imu_i = it_per_id.start_frame;
            Vector3d pts_i = it_per_id.feature_per_frame[0].point * it_per_id.estimated_depth;
            Vector3d w_pts_i = estimator.Rs[imu_i] * (estimator.ric[main_cam] * pts_i + estimator.tic[main_cam]) + estimator.Ps[imu_i];

            geometry_msgs::Point32 p;
            p.x = w_pts_i(0);
            p.y = w_pts_i(1);
            p.z = w_pts_i(2);
            margin_cloud.points.push_back(p);
        }
    }
    pub_margin_cloud.publish(margin_cloud);
}

void pubTF(const Estimator &estimator, const std_msgs::Header &header)
{
    if( estimator.solver_flag != Estimator::SolverFlag::NON_LINEAR)
        return;
    //int main_cam = estimator.f_manager.main_cam;
    int main_cam = 0;
    static tf::TransformBroadcaster br;
    tf::Transform transform;
    tf::Quaternion q;
    // body frame
    Vector3d correct_t;
    Quaterniond correct_q;
    correct_t = estimator.Ps[WINDOW_SIZE];
    correct_q = estimator.Rs[WINDOW_SIZE];

    transform.setOrigin(tf::Vector3(correct_t(0),
                                    correct_t(1),
                                    correct_t(2)));
    q.setW(correct_q.w());
    q.setX(correct_q.x());
    q.setY(correct_q.y());
    q.setZ(correct_q.z());
    transform.setRotation(q);
    br.sendTransform(tf::StampedTransform(transform, header.stamp, "world", "body"));

    // camera frame
    transform.setOrigin(tf::Vector3(estimator.tic[main_cam].x(),
                                    estimator.tic[main_cam].y(),
                                    estimator.tic[main_cam].z()));
    q.setW(Quaterniond(estimator.ric[main_cam]).w());
    q.setX(Quaterniond(estimator.ric[main_cam]).x());
    q.setY(Quaterniond(estimator.ric[main_cam]).y());
    q.setZ(Quaterniond(estimator.ric[main_cam]).z());
    transform.setRotation(q);
    br.sendTransform(tf::StampedTransform(transform, header.stamp, "body", "camera"));

    nav_msgs::Odometry odometry;
    odometry.header = header;
    odometry.header.frame_id = "world";
    odometry.pose.pose.position.x = estimator.tic[main_cam].x();
    odometry.pose.pose.position.y = estimator.tic[main_cam].y();
    odometry.pose.pose.position.z = estimator.tic[main_cam].z();
    Quaterniond tmp_q{estimator.ric[main_cam]};
    odometry.pose.pose.orientation.x = tmp_q.x();
    odometry.pose.pose.orientation.y = tmp_q.y();
    odometry.pose.pose.orientation.z = tmp_q.z();
    odometry.pose.pose.orientation.w = tmp_q.w();
    pub_extrinsic.publish(odometry);

}

void pubKeyframe(const Estimator &estimator)
{
    // pub camera pose, 2D-3D points of keyframe
    if (estimator.solver_flag == Estimator::SolverFlag::NON_LINEAR && estimator.marginalization_flag == 0)
    {
        int i = WINDOW_SIZE - 2;
        //Vector3d P = estimator.Ps[i] + estimator.Rs[i] * estimator.tic[0];
        Vector3d P = estimator.Ps[i];
        Quaterniond R = Quaterniond(estimator.Rs[i]);

        nav_msgs::Odometry odometry;
        odometry.header = estimator.Headers[WINDOW_SIZE - 2];
        odometry.header.frame_id = "world";
        odometry.pose.pose.position.x = P.x();
        odometry.pose.pose.position.y = P.y();
        odometry.pose.pose.position.z = P.z();
        odometry.pose.pose.orientation.x = R.x();
        odometry.pose.pose.orientation.y = R.y();
        odometry.pose.pose.orientation.z = R.z();
        odometry.pose.pose.orientation.w = R.w();
        //printf("time: %f t: %f %f %f r: %f %f %f %f\n", odometry.header.stamp.toSec(), P.x(), P.y(), P.z(), R.w(), R.x(), R.y(), R.z());

        pub_keyframe_pose.publish(odometry);


        sensor_msgs::PointCloud point_cloud;
        point_cloud.header = estimator.Headers[WINDOW_SIZE - 2];
        int main_cam = estimator.f_manager.main_cam;
        for (auto &it_per_id : estimator.f_manager.feature)
        {
            int frame_size = it_per_id.feature_per_frame.size();
            if(it_per_id.start_frame < WINDOW_SIZE - 2 && it_per_id.start_frame + frame_size - 1 >= WINDOW_SIZE - 2 && it_per_id.solve_flag == 1)
            {

                int imu_i = it_per_id.start_frame;
                Vector3d pts_i = it_per_id.feature_per_frame[0].point * it_per_id.estimated_depth;
                Vector3d w_pts_i = estimator.Rs[imu_i] * (estimator.ric[main_cam] * pts_i + estimator.tic[main_cam])
                                      + estimator.Ps[imu_i];
                geometry_msgs::Point32 p;
                p.x = w_pts_i(0);
                p.y = w_pts_i(1);
                p.z = w_pts_i(2);
                point_cloud.points.push_back(p);

                int imu_j = WINDOW_SIZE - 2 - it_per_id.start_frame;
                sensor_msgs::ChannelFloat32 p_2d;
                p_2d.values.push_back(it_per_id.feature_per_frame[imu_j].point.x());
                p_2d.values.push_back(it_per_id.feature_per_frame[imu_j].point.y());
                p_2d.values.push_back(it_per_id.feature_per_frame[imu_j].uv.x());
                p_2d.values.push_back(it_per_id.feature_per_frame[imu_j].uv.y());
                p_2d.values.push_back(it_per_id.feature_id);
                point_cloud.channels.push_back(p_2d);
            }

        }
        pub_keyframe_point.publish(point_cloud);
    }
}

void pubRelocalization(const Estimator &estimator)
{
    nav_msgs::Odometry odometry;
    odometry.header.stamp = ros::Time(estimator.relo_frame_stamp);
    odometry.header.frame_id = "world";
    odometry.pose.pose.position.x = estimator.relo_relative_t.x();
    odometry.pose.pose.position.y = estimator.relo_relative_t.y();
    odometry.pose.pose.position.z = estimator.relo_relative_t.z();
    odometry.pose.pose.orientation.x = estimator.relo_relative_q.x();
    odometry.pose.pose.orientation.y = estimator.relo_relative_q.y();
    odometry.pose.pose.orientation.z = estimator.relo_relative_q.z();
    odometry.pose.pose.orientation.w = estimator.relo_relative_q.w();
    odometry.twist.twist.linear.x = estimator.relo_relative_yaw;
    odometry.twist.twist.linear.y = estimator.relo_frame_index;

    pub_relo_relative_pose.publish(odometry);
}

void pubObservationMatrix(const Estimator &estimator)
{
  std_msgs::Float64 m;
//  m.data = estimator.inverse_observation_matrix_trace;
#if !two_cam_test
  m.data = estimator.Observation_matrix_trace;
#endif
  pub_observation_matrix.publish(m);
}

void pubSameIdImg(const Estimator &estimator, queue<sensor_msgs::ImageConstPtr> &img_buf)
{
    sensor_msgs::ImageConstPtr img_msg = img_buf.back();

    cv_bridge::CvImageConstPtr ptr;
    ptr = cv_bridge::toCvCopy(img_msg, sensor_msgs::image_encodings::MONO8);
    ptr = cv_bridge::cvtColor(ptr, sensor_msgs::image_encodings::BGR8);
    cv::Mat stereo_img = ptr->image;
#if two_cam_test
    cv::Mat tmp_img0 = stereo_img.colRange(0, COL);
    cv::Mat tmp_img1 = stereo_img.colRange(COL, COL*2);
#else
    cv::Mat tmp_img = stereo_img.rowRange(0, ROW);
#endif

    // Rotate the CV Image with 180 degree
    bool rotate_image = false;
    if (rotate_image)
    {
#if two_cam_test
        cv::Point2f pc0(tmp_img0.cols/2., tmp_img0.rows/2.);
        cv::Point2f pc1(tmp_img1.cols/2., tmp_img1.rows/2.);
        cv::Mat r0 = cv::getRotationMatrix2D(pc0, 180, 1.0);
        cv::Mat r1 = cv::getRotationMatrix2D(pc1, 180, 1.0);
        cv::warpAffine(tmp_img0, tmp_img0, r0, tmp_img0.size());
        cv::warpAffine(tmp_img1, tmp_img1, r1, tmp_img1.size());
#else
        cv::Point2f pc(tmp_img.cols/2., tmp_img.rows/2.);
        cv::Mat r = cv::getRotationMatrix2D(pc, 180, 1.0);
        cv::warpAffine(tmp_img, tmp_img, r, tmp_img.size());
#endif
    }

# if two_cam_test
    for (int i = 0; i < NUM_OF_CAM; i++)
    {
        for (unsigned int j = 0; j < estimator.sameId_pts[i].size(); j++)
        {
            // circle the feature point
            cv::Point2f tmp_pts;
            if (rotate_image)
            {
                tmp_pts.x = COL - estimator.sameId_pts[i][j].second.x;
                tmp_pts.y = ROW - estimator.sameId_pts[i][j].second.y;
            }
            else
            {
                tmp_pts.x = estimator.sameId_pts[i][j].second.x;
                tmp_pts.y = estimator.sameId_pts[i][j].second.y;
            }
            double len = std::min(1.0, (estimator.pts_observability[i][j] - 2.0) / 1.5); // coler range: 2.0 ~ 3.5
            if (i == 0)
            {
//                cv::circle(tmp_img0, tmp_pts, 1, cv::Scalar(255 * (1 - len), 0, 255 * len), 2);
                cv::circle(tmp_img0, tmp_pts, 1, cv::Scalar(0, 255, 0), 2);
            }
            else if (i == 1)
            {
//                cv::circle(tmp_img1, tmp_pts, 1, cv::Scalar(255 * (1 - len), 0, 255 * len), 2);
                cv::circle(tmp_img1, tmp_pts, 1, cv::Scalar(0, 255, 0), 2);
            }

            // text the feature point
//            std::stringstream ss;
//            double obs_val = estimator.pts_observability[i][j];
//            ss << std::setprecision(3) << obs_val; // 對obs_val只取到小數點後兩位
//            if (i == 0) cv::putText(tmp_img0, ss.str(), tmp_pts, cv::FONT_HERSHEY_COMPLEX, 0.5, cv::Scalar(0, 255, 0), 1, 8, 0);
//            else if (i == 1) cv::putText(tmp_img1, ss.str(), tmp_pts, cv::FONT_HERSHEY_COMPLEX, 0.5, cv::Scalar(0, 255, 0), 1, 8, 0);
        }
    }

    // show the main camera edge
    if (estimator.f_manager.main_cam == estimator.f_manager.CAM0)
    {
        cv::Rect rect;
        rect.width = tmp_img0.cols;
        rect.height = tmp_img0.rows;
        rect.x = 0.;
        rect.y = 0.;
        cv::rectangle(tmp_img0, rect, cv::Scalar(0, 0, 255), 3);
    }
    else if (estimator.f_manager.main_cam == estimator.f_manager.CAM1)
    {
        cv::Rect rect;
        rect.width = tmp_img1.cols;
        rect.height = tmp_img1.rows;
        rect.x = 0.;
        rect.y = 0.;
        cv::rectangle(tmp_img1, rect, cv::Scalar(0, 255, 255), 3);
    }
#else
    for (unsigned int j = 0; j < estimator.sameId_pts.size(); j++)
    {
        // circle the feature point
        cv::Point2f tmp_pts;
        if (rotate_image)
        {
            tmp_pts.x = COL - estimator.sameId_pts[j].second.x;
            tmp_pts.y = ROW - estimator.sameId_pts[j].second.y;
        }
        else
        {
            tmp_pts.x = estimator.sameId_pts[j].second.x;
            tmp_pts.y = estimator.sameId_pts[j].second.y;
        }
        double len = std::min(1.0, (estimator.pts_observability[j] - 2.0) / 1.5); // coler range: 2.0 ~ 3.5
        cv::circle(tmp_img, tmp_pts, 1, cv::Scalar(255 * (1 - len), 0, 255 * len), 2);

        // text the feature point
        std::stringstream ss;
        double obs_val = estimator.pts_observability[j];
        ss << std::setprecision(3) << obs_val; // 對obs_val只取到小數點後兩位
        cv::putText(tmp_img, ss.str(), tmp_pts, cv::FONT_HERSHEY_COMPLEX, 0.5, cv::Scalar(0, 255, 0), 1, 8, 0);
    }
#endif
    // ===== publish image msg to feature_img ====
    pub_sameId.publish(ptr->toImageMsg());
}
