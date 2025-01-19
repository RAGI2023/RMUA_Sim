#pragma once

#include "ros/console.h"
#include "ros/node_handle.h"
#include <ctime>
#include <opencv2/core/mat.hpp>
#include <opencv2/core/persistence.hpp>
#include <opencv2/core/types.hpp>
#include <opencv2/highgui.hpp>
#include <opencv2/imgproc.hpp>
#include <string>
#include <vector>
#include "ros/publisher.h"
#include "sensor_msgs/Image.h"
#include "cv_bridge/cv_bridge.h"
#include  "sensor_msgs/Imu.h"
#include "airsim_ros/Scene.h"

class CameraParameter
{
public:
    CameraParameter(const std::string &file_path) : file_path_(file_path) {};
    CameraParameter(const CameraParameter &other) : file_path_(other.file_path_), cameraMatrix_l(other.cameraMatrix_l), cameraMatrix_r(other.cameraMatrix_r), 
        distCoeffs_l(other.distCoeffs_l), R(other.R), T(other.T) {};
    ~CameraParameter() {};

    bool readParameters()
    {
        fs_.open(file_path_, cv::FileStorage::READ);
        if (!fs_.isOpened())
        {
            ROS_ERROR("Failed to open camera parameters file: %s\n", file_path_.c_str());
            return false; 
        }

        fs_["camera_matrix_left"] >> cameraMatrix_l;
        fs_["camera_matrix_right"] >> cameraMatrix_r;
        fs_["dist_coeff_left"] >> distCoeffs_l;
        fs_["dist_coeff_right"] >> distCoeffs_r;
        fs_["R"] >> R;
        fs_["T"] >> T;

        fs_.release();
        
        if (cameraMatrix_l.empty() || cameraMatrix_r.empty() || distCoeffs_l.empty() || distCoeffs_r.empty() || R.empty() || T.empty()){
            ROS_ERROR("Failed to read camera parameters\n");
            return false;
        }

        ROS_INFO("Read camera parameters successfully\n");
        return true;
    }

    cv::Mat cameraMatrix_l, cameraMatrix_r; //相机内参
    cv::Mat distCoeffs_l, distCoeffs_r; //畸变参数
    cv::Mat R, T; //旋转矩阵和平移矩阵

private:
    std::string file_path_;
    cv::FileStorage fs_;
};

class DepthGenerator
{
public:
    DepthGenerator(const std::string &right_topic, const std::string &left_topic, const std::string depth_topic, const CameraParameter &parameters) ;
    DepthGenerator(const std::string &scene_topic, const CameraParameter &parameters) ;
    ~DepthGenerator() {};

private:
    ros::NodeHandle nh_;
    ros::Subscriber sub_left_, sub_right_, sub_imu_, sub_scene_;
    ros::Publisher pub_depth_;
    std::string left_image_topic_, right_image_topic_,scene_topic_,  depth_topic_;
    cv::Mat depth_image_;
    CameraParameter parameters_;
    cv_bridge::CvImageConstPtr last_left_img_, last_right_img_;
    airsim_ros::Scene last_scene_;
    double right_time_, left_time_;
    double timestamp_;

    const double max_time_diff = 1e-4;

    //处理image消息
    void process_image();
    //处理scene消息
    void process_scene(); 
    void callback_right(const sensor_msgs::ImageConstPtr &msg)
    {
        last_right_img_ = cv_bridge::toCvCopy(msg, sensor_msgs::image_encodings::TYPE_8UC3);
        right_time_ = get_currenttime(msg);
        // right_time_ = timestamp_;
        if(!last_right_img_->image.empty())
        {
            ROS_INFO("Get front right image.: %f", msg->header.stamp.sec + msg->header.stamp.nsec*1e-9);
        }
        process_image();
    }
    void callback_left(const sensor_msgs::ImageConstPtr &msg)
    {
        last_left_img_ = cv_bridge::toCvCopy(msg, sensor_msgs::image_encodings::TYPE_8UC3);
        left_time_ = get_currenttime(msg);
        // left_time_ = timestamp_;
        if(!last_left_img_->image.empty())
        {
            ROS_INFO("Get front left image.: %f", msg->header.stamp.sec + msg->header.stamp.nsec*1e-9);
        }
        process_image();
    }
    void callback_imu(const sensor_msgs::Imu::ConstPtr& msg)
    {
        timestamp_ = msg->header.stamp.toSec() + msg->header.stamp.toNSec() * 1e-9;
    }

    void callback_scene(const airsim_ros::Scene::ConstPtr& msg)
    {
        last_scene_ = *msg;
        ROS_INFO("Get scene: %f, %f", msg->left.header.stamp.sec + msg->left.header.stamp.nsec*1e-9, msg->right.header.stamp.sec + msg->right.header.stamp.nsec*1e-9);
        process_scene();
    }

    // 通过图像头获取时间戳
    double get_currenttime(const sensor_msgs::ImageConstPtr &msg);
    // 通过imu头获取时间戳
    double get_currenttime();

    void stereoRectification(const cv::Mat& cameraMatrix_l, const cv::Mat& distCoeffs_l,
                         const cv::Mat& cameraMatrix_r, const cv::Mat& distCoeffs_r,
                         const cv::Mat& R, const cv::Mat& T,
                         const cv::Size& img_size, const cv::Mat& img_l, const cv::Mat& img_r,
                         cv::Mat& rectified_img_l, cv::Mat& rectified_img_r);

    void ExtractOrangeMask(cv::Mat &img, cv::Mat &mask);
    void ExtractOrange(cv::Mat &img, cv::Mat &Output);

    double CalculateAspectRatio(const std::vector<cv::Point> &countour)
    {
        cv::RotatedRect minRect = cv::minAreaRect(countour);
        double width = minRect.size.width > minRect.size.height ? minRect.size.width : minRect.size.height;
        double height = minRect.size.width < minRect.size.height ? minRect.size.width : minRect.size.height;
        return width / height;
    }

};

class VideoPlayer
{
public:
    VideoPlayer(const std::string &video_topic, const std::string &window_name, int delay = 10) : window_name_(window_name), delay_(delay)
    {
        sub_video_ = nh_.subscribe<sensor_msgs::Image>(video_topic, 1, 
            std::bind(&VideoPlayer::callback_video, this, std::placeholders::_1));
    }
private:
    ros::NodeHandle nh_;
    ros::Subscriber sub_video_;
    cv::Mat frame_;
    cv_bridge::CvImageConstPtr last_video_;
    std::string window_name_;
    int delay_;
    void callback_video(const sensor_msgs::ImageConstPtr &msg)
    {
        last_video_ = cv_bridge::toCvCopy(msg, sensor_msgs::image_encodings::TYPE_8UC3);
        if(!last_video_->image.empty())
        {
            cv:: Mat image_ = last_video_->image;
            std::string time = std::to_string(msg->header.stamp.sec + msg->header.stamp.toNSec() * 1e-9);
            cv::putText(image_, time, cv::Point(10, 30), cv::FONT_HERSHEY_SIMPLEX, 1.0, cv::Scalar(255, 255, 255), 2);
            ROS_INFO("%s get video frame.: %f", window_name_.c_str(), msg->header.stamp.sec + msg->header.stamp.nsec*1e-9);
            cv::imshow(window_name_, image_);
            cv::waitKey(delay_);
        }else {
            ROS_WARN("Video frame is empty.");
        }
    }
};

