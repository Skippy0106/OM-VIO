#include <ros/ros.h>
#include <sensor_msgs/Image.h>
#include <sensor_msgs/image_encodings.h>
#include <sensor_msgs/PointCloud.h>
#include <sensor_msgs/Imu.h>
#include <std_msgs/Bool.h>
#include <cv_bridge/cv_bridge.h>
#include <message_filters/subscriber.h>

#include "feature_tracker.h"

#define SHOW_UNDISTORTION 0

#include <thread>
#include <mutex>
std::mutex m_buf;

vector<uchar> r_status;
vector<float> r_err;
queue<sensor_msgs::ImageConstPtr> img_buf;
#if two_cam_test
queue<sensor_msgs::ImageConstPtr> img0_buf;
queue<sensor_msgs::ImageConstPtr> img1_buf;
#endif

ros::Publisher pub_img,pub_match;
ros::Publisher pub_restart;

#if two_cam_test
FeatureTracker trackerData[2];
#else
FeatureTracker trackerData[NUM_OF_CAM];
#endif
double first_image_time;
int pub_count = 1;
bool first_image_flag = true;
double last_image_time = 0;
bool init_pub = 0;

#if two_cam_test
void img0_callback(const sensor_msgs::ImageConstPtr &img_msg)
{
    m_buf.lock();
    img0_buf.push(img_msg);
    m_buf.unlock();
}

void img1_callback(const sensor_msgs::ImageConstPtr &img_msg)
{
    m_buf.lock();
    img1_buf.push(img_msg);
    m_buf.unlock();
}

cv::Mat getImageFromMsg(const sensor_msgs::ImageConstPtr &img_msg)
{
    cv_bridge::CvImageConstPtr ptr;
    if (img_msg->encoding == "8UC1")
    {
        sensor_msgs::Image img;
        img.header = img_msg->header;
        img.height = img_msg->height;
        img.width = img_msg->width;
        img.is_bigendian = img_msg->is_bigendian;
        img.step = img_msg->step;
        img.data = img_msg->data;
        img.encoding = "mono8";
        ptr = cv_bridge::toCvCopy(img, sensor_msgs::image_encodings::MONO8);
    }
    else
        ptr = cv_bridge::toCvCopy(img_msg, sensor_msgs::image_encodings::MONO8);

    cv::Mat img = ptr->image.clone();
    return img;
}

void sync_process()
{
    STEREO_TRACK = false; // maybe wrong
    while(1)
    {
        cv::Mat image0, image1;
        std_msgs::Header header;
        double time = 0;
        m_buf.lock();
        if (!img0_buf.empty() && !img1_buf.empty())
        {
            double time0 = img0_buf.front()->header.stamp.toSec();
            double time1 = img1_buf.front()->header.stamp.toSec();
            // 0.003s sync tolerance
            if(time0 < time1 - 0.003)
            {
                img0_buf.pop();
                printf("throw img0\n");
            }
            else if(time0 > time1 + 0.003)
            {
                img1_buf.pop();
                printf("throw img1\n");
            }
            else
            {
                time = img0_buf.front()->header.stamp.toSec();
                header = img0_buf.front()->header;
                image0 = getImageFromMsg(img0_buf.front());
                img0_buf.pop();
                image1 = getImageFromMsg(img1_buf.front());
                img1_buf.pop();
                //printf("find img0 and img1\n");
            }
        }
        m_buf.unlock();
        if(!image0.empty())
        {
            if(first_image_flag)
            {
                first_image_flag = false;
                first_image_time = time;
                continue;
            }
            // frequency control
            if (round(1.0 * pub_count / (time - first_image_time)) <= FREQ)
            {
                PUB_THIS_FRAME = true;
                // reset the frequency control
                if (abs(1.0 * pub_count / (time - first_image_time) - FREQ) < 0.01 * FREQ)
                {
                    first_image_time = time;
                    pub_count = 0;
                }
            }
            else
                PUB_THIS_FRAME = false;

            TicToc t_r;
            for (int i = 0; i < NUM_OF_CAM; i++)
            {
                ROS_DEBUG("processing camera %d", i);
                if (i != 1 || !STEREO_TRACK)
                {
                    //std::cout << "ptr->image: " << ptr->image.size() << std::endl;
                    if (i==0)
                    {
                        trackerData[i].readImage(image0, header.stamp.toSec());
                    }
                    else if (i==1)
                    {
                        trackerData[i].readImage(image1, header.stamp.toSec());
                    }
                }
                else
                {
                    /*if (EQUALIZE)
                    {
                        cv::Ptr<cv::CLAHE> clahe = cv::createCLAHE();
                        clahe->apply(ptr->image.rowRange(ROW * i, ROW * (i + 1)), trackerData[i].cur_img);
                    }
                    else
                        trackerData[i].cur_img = ptr->image.rowRange(ROW * i, ROW * (i + 1));*/
                }
            }

            for (unsigned int i = 0;; i++)
            {
                bool completed = false;
                for (int j = 0; j < NUM_OF_CAM; j++)
                    if (j != 1 || !STEREO_TRACK)
                        completed |= trackerData[j].updateID(i);
                if (!completed)
                    break;
            }

           if (PUB_THIS_FRAME)
           {
                pub_count++;
                sensor_msgs::PointCloudPtr feature_points(new sensor_msgs::PointCloud);
                sensor_msgs::ChannelFloat32 id_of_point;
                sensor_msgs::ChannelFloat32 u_of_point;
                sensor_msgs::ChannelFloat32 v_of_point;
                sensor_msgs::ChannelFloat32 velocity_x_of_point;
                sensor_msgs::ChannelFloat32 velocity_y_of_point;

                feature_points->header = header;
                feature_points->header.frame_id = "world";

                vector<set<int>> hash_ids(NUM_OF_CAM);
                for (int i = 0; i < NUM_OF_CAM; i++)
                {
                    auto &un_pts = trackerData[i].cur_un_pts;
                    auto &cur_pts = trackerData[i].cur_pts;
                    auto &ids = trackerData[i].ids;
                    auto &pts_velocity = trackerData[i].pts_velocity;
                    for (unsigned int j = 0; j < ids.size(); j++)
                    {
                        if (trackerData[i].track_cnt[j] > 1)
                        {
                            int p_id = ids[j];
                            hash_ids[i].insert(p_id);
                            geometry_msgs::Point32 p;
                            p.x = un_pts[j].x;
                            p.y = un_pts[j].y;
                            p.z = 1;

                            feature_points->points.push_back(p);
                            id_of_point.values.push_back(p_id * NUM_OF_CAM + i);
                            u_of_point.values.push_back(cur_pts[j].x);
                            v_of_point.values.push_back(cur_pts[j].y);
                            velocity_x_of_point.values.push_back(pts_velocity[j].x);
                            velocity_y_of_point.values.push_back(pts_velocity[j].y);
                        }
                    }
                }
                feature_points->channels.push_back(id_of_point);
                feature_points->channels.push_back(u_of_point);
                feature_points->channels.push_back(v_of_point);
                feature_points->channels.push_back(velocity_x_of_point);
                feature_points->channels.push_back(velocity_y_of_point);
                ROS_DEBUG("publish %f, at %f", feature_points->header.stamp.toSec(), ros::Time::now().toSec());
                // skip the first image; since no optical speed on frist image
                if (!init_pub)
                {
                    init_pub = 1;
                }
                else
                    pub_img.publish(feature_points); // the place should check

                if (SHOW_TRACK)
                {
                    cv::Mat tmp_img0, tmp_img1;
                    cv::cvtColor(image0, tmp_img0, CV_GRAY2RGB);
                    cv::cvtColor(image1, tmp_img1, CV_GRAY2RGB);

                    for (int i = 0; i < NUM_OF_CAM; i++)
                    {
                        for (unsigned int j = 0; j < trackerData[i].cur_pts.size(); j++)
                        {
                            double len = std::min(1.0, 1.0 * trackerData[i].track_cnt[j] / WINDOW_SIZE);
                            if (i==0)
                            {
                                cv::circle(tmp_img0, trackerData[i].cur_pts[j], 2, cv::Scalar(255 * len, 0, 255 * (1 - len)), 2);
                                // visualize feature id
//                                char name[10];
//                                sprintf(name, "%d", trackerData[i].ids[j]);
//                                cv::putText(tmp_img0, name, trackerData[i].cur_pts[j], cv::FONT_HERSHEY_SIMPLEX, 0.5, cv::Scalar(0, 255, 0));
                            }
                            else if (i==1)
                            {
                                cv::circle(tmp_img1, trackerData[i].cur_pts[j], 2, cv::Scalar(255 * len, 0, 255 * (1 - len)), 2);
                                // visualize feature id
//                                char name[10];
//                                sprintf(name, "%d", trackerData[i].ids[j]);
//                                cv::putText(tmp_img1, name, trackerData[i].cur_pts[j], cv::FONT_HERSHEY_SIMPLEX, 0.5, cv::Scalar(0, 255, 0));
                            }
                        }
                    }
                    // ===== publish image msg to feature_img ====
                    cv::Mat imTrack;
                    cv::hconcat(tmp_img0, tmp_img1, imTrack);

                    cv_bridge::CvImage out_msg;
                    out_msg.header = header;
                    out_msg.encoding = sensor_msgs::image_encodings::BGR8;
                    out_msg.image = imTrack;
                    pub_match.publish(out_msg.toImageMsg());
                }
            }
            ROS_INFO("whole feature tracker processing costs: %f", t_r.toc());
        }
    }
}
#endif

void img_callback(const sensor_msgs::ImageConstPtr &img_msg)
{
    if(first_image_flag)
    {
        first_image_flag = false;
        first_image_time = img_msg->header.stamp.toSec();
        last_image_time = img_msg->header.stamp.toSec();
        return;
    }
    // detect unstable camera stream
    if (img_msg->header.stamp.toSec() - last_image_time > 1.0 || img_msg->header.stamp.toSec() < last_image_time)
    {
        ROS_WARN("image discontinue! reset the feature tracker!");
        first_image_flag = true;
        last_image_time = 0;
        pub_count = 1;
        std_msgs::Bool restart_flag;
        restart_flag.data = true;
        pub_restart.publish(restart_flag);
        return;
    }
    last_image_time = img_msg->header.stamp.toSec();
    // frequency control
    if (round(1.0 * pub_count / (img_msg->header.stamp.toSec() - first_image_time)) <= FREQ)
    {
        PUB_THIS_FRAME = true;
        // reset the frequency control
        if (abs(1.0 * pub_count / (img_msg->header.stamp.toSec() - first_image_time) - FREQ) < 0.01 * FREQ)
        {
            first_image_time = img_msg->header.stamp.toSec();
            pub_count = 0;
        }
    }
    else
        PUB_THIS_FRAME = false;

    cv_bridge::CvImageConstPtr ptr;
    if (img_msg->encoding == "8UC1")
    {
        sensor_msgs::Image img;
        img.header = img_msg->header;
        img.height = img_msg->height;
        img.width = img_msg->width;
        img.is_bigendian = img_msg->is_bigendian;
        img.step = img_msg->step;
        img.data = img_msg->data;
        img.encoding = "mono8";
        ptr = cv_bridge::toCvCopy(img, sensor_msgs::image_encodings::MONO8);
    }
    else
        ptr = cv_bridge::toCvCopy(img_msg, sensor_msgs::image_encodings::MONO8);

    cv::Mat show_img = ptr->image;
    TicToc t_r;
    for (int i = 0; i < NUM_OF_CAM; i++)
    {
        ROS_DEBUG("processing camera %d", i);
        if (i != 1 || !STEREO_TRACK)
        {
            //std::cout << "ptr->image: " << ptr->image.size() << std::endl;
            trackerData[i].readImage(ptr->image.rowRange(ROW * i, ROW * (i + 1)), img_msg->header.stamp.toSec()); //error
        }
        else
        {
            if (EQUALIZE)
            {
                cv::Ptr<cv::CLAHE> clahe = cv::createCLAHE();
                clahe->apply(ptr->image.rowRange(ROW * i, ROW * (i + 1)), trackerData[i].cur_img);
            }
            else
                trackerData[i].cur_img = ptr->image.rowRange(ROW * i, ROW * (i + 1));
        }

#if SHOW_UNDISTORTION
        trackerData[i].showUndistortion("undistrotion_" + std::to_string(i));
#endif
    }

    for (unsigned int i = 0;; i++)
    {
        bool completed = false;
        for (int j = 0; j < NUM_OF_CAM; j++)
            if (j != 1 || !STEREO_TRACK)
                completed |= trackerData[j].updateID(i);
        if (!completed)
            break;
    }

   if (PUB_THIS_FRAME)
   {
        pub_count++;
        sensor_msgs::PointCloudPtr feature_points(new sensor_msgs::PointCloud);
        sensor_msgs::ChannelFloat32 id_of_point;
        sensor_msgs::ChannelFloat32 u_of_point;
        sensor_msgs::ChannelFloat32 v_of_point;
        sensor_msgs::ChannelFloat32 velocity_x_of_point;
        sensor_msgs::ChannelFloat32 velocity_y_of_point;

        feature_points->header = img_msg->header;
        feature_points->header.frame_id = "world";

        vector<set<int>> hash_ids(NUM_OF_CAM);
        for (int i = 0; i < NUM_OF_CAM; i++)
        {
            auto &un_pts = trackerData[i].cur_un_pts;
            auto &cur_pts = trackerData[i].cur_pts;
            auto &ids = trackerData[i].ids;
            auto &pts_velocity = trackerData[i].pts_velocity;
            for (unsigned int j = 0; j < ids.size(); j++)
            {
                if (trackerData[i].track_cnt[j] > 1)
                {
                    int p_id = ids[j];
                    hash_ids[i].insert(p_id);
                    geometry_msgs::Point32 p;
                    p.x = un_pts[j].x;
                    p.y = un_pts[j].y;
                    p.z = 1;

                    feature_points->points.push_back(p);
                    id_of_point.values.push_back(p_id * NUM_OF_CAM + i);
                    u_of_point.values.push_back(cur_pts[j].x);
                    v_of_point.values.push_back(cur_pts[j].y);
                    velocity_x_of_point.values.push_back(pts_velocity[j].x);
                    velocity_y_of_point.values.push_back(pts_velocity[j].y);
                }
            }
        }
        feature_points->channels.push_back(id_of_point);
        feature_points->channels.push_back(u_of_point);
        feature_points->channels.push_back(v_of_point);
        feature_points->channels.push_back(velocity_x_of_point);
        feature_points->channels.push_back(velocity_y_of_point);
        ROS_DEBUG("publish %f, at %f", feature_points->header.stamp.toSec(), ros::Time::now().toSec());
        // skip the first image; since no optical speed on frist image
        if (!init_pub)
        {
            init_pub = 1;
        }
        else
            pub_img.publish(feature_points);

        if (SHOW_TRACK)
        {
            ptr = cv_bridge::cvtColor(ptr, sensor_msgs::image_encodings::BGR8);
            //cv::Mat stereo_img(ROW * NUM_OF_CAM, COL, CV_8UC3);
            cv::Mat stereo_img = ptr->image;

            for (int i = 0; i < NUM_OF_CAM; i++)
            {
                cv::Mat tmp_img = stereo_img.rowRange(i * ROW, (i + 1) * ROW);
                cv::cvtColor(show_img, tmp_img, CV_GRAY2RGB);

                for (unsigned int j = 0; j < trackerData[i].cur_pts.size(); j++)
                {
                    double len = std::min(1.0, 1.0 * trackerData[i].track_cnt[j] / WINDOW_SIZE);
//                    cv::circle(tmp_img, trackerData[i].cur_pts[j], 2, cv::Scalar(255 * (1 - len), 0, 255 * len), 2);
                    cv::circle(tmp_img, trackerData[i].cur_pts[j], 2, cv::Scalar(255 * len, 0, 255 * (1 - len)), 2);
                    //draw speed line
                    /*
                    Vector2d tmp_cur_un_pts (trackerData[i].cur_un_pts[j].x, trackerData[i].cur_un_pts[j].y);
                    Vector2d tmp_pts_velocity (trackerData[i].pts_velocity[j].x, trackerData[i].pts_velocity[j].y);
                    Vector3d tmp_prev_un_pts;
                    tmp_prev_un_pts.head(2) = tmp_cur_un_pts - 0.10 * tmp_pts_velocity;
                    tmp_prev_un_pts.z() = 1;
                    Vector2d tmp_prev_uv;
                    trackerData[i].m_camera->spaceToPlane(tmp_prev_un_pts, tmp_prev_uv);
                    cv::line(tmp_img, trackerData[i].cur_pts[j], cv::Point2f(tmp_prev_uv.x(), tmp_prev_uv.y()), cv::Scalar(255 , 0, 0), 1 , 8, 0);
                    */
                    //char name[10];
                    //sprintf(name, "%d", trackerData[i].ids[j]);
                    //cv::putText(tmp_img, name, trackerData[i].cur_pts[j], cv::FONT_HERSHEY_SIMPLEX, 0.5, cv::Scalar(0, 0, 0));
                }
            }
            //cv::imshow("vis", stereo_img);
            //cv::waitKey(5);

            // ===== publish image msg to feature_img ====
            pub_match.publish(ptr->toImageMsg());

            // inverse original image msg
//            auto inv_ptr = ptr->toImageMsg();
//            std::reverse(inv_ptr->data.begin(), inv_ptr->data.end());
//            pub_match.publish(inv_ptr);
        }
    }
    ROS_INFO("whole feature tracker processing costs: %f", t_r.toc());
}

int main(int argc, char **argv)
{
    ros::init(argc, argv, "feature_tracker");
    ros::NodeHandle n("~");
    ros::console::set_logger_level(ROSCONSOLE_DEFAULT_NAME, ros::console::levels::Info);
    readParameters(n);

    for (int i = 0; i < NUM_OF_CAM; i++)
        trackerData[i].readIntrinsicParameter(CAM_NAMES[i]);

    if(FISHEYE)
    {
        for (int i = 0; i < NUM_OF_CAM; i++)
        {
            trackerData[i].fisheye_mask = cv::imread(FISHEYE_MASK, 0);
            if(!trackerData[i].fisheye_mask.data)
            {
                ROS_INFO("load mask fail");
                ROS_BREAK();
            }
            else
                ROS_INFO("load mask success");
        }
    }

#if two_cam_test
    ros::Subscriber sub_img0 = n.subscribe(IMAGE0_TOPIC, 100, img0_callback);
    ros::Subscriber sub_img1 = n.subscribe(IMAGE1_TOPIC, 100, img1_callback);
    std::thread sync_thread{sync_process};
#else
    ros::Subscriber sub_img = n.subscribe(IMAGE_TOPIC, 100, img_callback);
#endif

    pub_img = n.advertise<sensor_msgs::PointCloud>("feature", 1000);
    pub_match = n.advertise<sensor_msgs::Image>("feature_img",1000);
    pub_restart = n.advertise<std_msgs::Bool>("restart",1000);
    /*
    if (SHOW_TRACK)
        cv::namedWindow("vis", cv::WINDOW_NORMAL);
    */
    ros::spin();
    return 0;
}


// new points velocity is 0, pub or not?
// track cnt > 1 pub?
