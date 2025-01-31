#pragma once

#include "ros/console.h"
#include "ros/node_handle.h"
#include <ctime>
#include <exception>
#include <memory>
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
#include <message_filters/subscriber.h>
#include <message_filters/synchronizer.h>
#include <message_filters/sync_policies/approximate_time.h>
#include <message_filters/sync_policies/exact_time.h>
#include <message_filters/time_synchronizer.h>


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

private:
    ros::NodeHandle nh_;
    ros::Subscriber sub_left_, sub_right_, sub_imu_, sub_scene_;
    ros::Publisher pub_depth_;
    std::string left_image_topic_, right_image_topic_, depth_topic_;
    cv::Mat depth_image_;
    CameraParameter parameters_;
    airsim_ros::Scene last_scene_;
    std::unique_ptr<message_filters::Synchronizer<message_filters::sync_policies::ExactTime<sensor_msgs::Image, sensor_msgs::Image>>> sync_handler_ptr;
    double timestamp_;
    message_filters::Subscriber<sensor_msgs::Image> fl_image_suber_, fr_image_suber_;

    //处理场景
    void process_scene(cv::Mat &left_image, cv::Mat &right_image); 
    
    void callback_imu(const sensor_msgs::Imu::ConstPtr& msg)
    {
        timestamp_ = msg->header.stamp.toSec() + msg->header.stamp.toNSec() * 1e-9;
    }

    void stereo_view_cb(const sensor_msgs::ImageConstPtr& fl_img_msg, const sensor_msgs::ImageConstPtr& fr_img_msg)
    {
        ROS_INFO("Get stereo images");
        try {
            auto cv_ptrl = cv_bridge::toCvCopy(fl_img_msg, sensor_msgs::image_encodings::BGR8);
            auto cv_ptrr = cv_bridge::toCvCopy(fr_img_msg, sensor_msgs::image_encodings::BGR8);
            cv::Mat image_l  = cv_ptrl->image.clone();
            cv::Mat image_r  = cv_ptrr->image.clone();
            ROS_INFO("processing...");
            process_scene(image_l, image_r);
        } catch (std::exception &e) {
            ROS_ERROR("Exception occurred while processing stereo images: %s", e.what());
        }
    }

    // 通过imu头获取时间戳
    double get_currenttime_imu(const sensor_msgs::Imu::ConstPtr &msg)
    {
        return msg->header.stamp.toSec() + msg->header.stamp.toNSec() * 1e-9;
    }

    void stereoRectification(const cv::Mat& cameraMatrix_l, const cv::Mat& distCoeffs_l,
                         const cv::Mat& cameraMatrix_r, const cv::Mat& distCoeffs_r,
                         const cv::Mat& R, const cv::Mat& T,
                         const cv::Size& img_size, const cv::Mat& img_l, const cv::Mat& img_r,
                         cv::Mat& rectified_img_l, cv::Mat& rectified_img_r);

    void ExtractOrangeMask(cv::Mat &img, cv::Mat &mask);
    void ExtractOrange(cv::Mat &img, cv::Mat &Output);

    // 计算轮廓的长宽比 高/宽
    double CalculateAspectRatio(const std::vector<cv::Point> &countour)
    {
        cv::RotatedRect minRect = cv::minAreaRect(countour);
        // double width = minRect.size.width > minRect.size.height ? minRect.size.width : minRect.size.height;
        // double height = minRect.size.width < minRect.size.height ? minRect.size.width : minRect.size.height;
        return minRect.size.height / minRect.size.width;
        // return height / width;
    }

    double CalculateAspectRatio(const cv::RotatedRect &rect)
    {
        return rect.size.height / rect.size.width;
    }

    double CalculateAspectRatio(const cv::Rect &rect)
    {
        return static_cast<double>(rect.height) / rect.width;
    }

    void DrawRotatedRect(cv::Mat &image, const std::vector<cv::Point> &contour, cv::Scalar color = cv::Scalar(255, 0, 0), int thickness = 2)
    {
        cv::RotatedRect minRect = cv::minAreaRect(contour);
        cv::Point2f vertices[4];
        minRect.points(vertices);
        for (int i = 0; i < 4; i++){
            cv::line(image, vertices[i], vertices[(i + 1) % 4], color, thickness);
        }
    }

    void DrawRect(cv::Mat &image, const std::vector<cv::Point> &contour, cv::Scalar color = cv::Scalar(255, 0, 0), int thickness = 2)
    {
        cv::Rect rect = cv::boundingRect(contour);
        cv::rectangle(image, rect, color, thickness);
    }

    double ContourSimilarity(const std::vector<cv::Point> &contour1, const std::vector<cv::Point> &contour2);
    double ContourSimilarity(const cv::Rect &rect1, const cv::Rect &rect2);

    // 计算轮廓质心
    cv::Point2f CalculateCentroid(const std::vector<cv::Point> &contour)
    {
        cv::Moments mu = cv::moments(contour);
        return cv::Point2f(mu.m10 / mu.m00, mu.m01 / mu.m00);
    }

    // 计算轮廓面积相似度
    double CalculateAreaSimilarity(const std::vector<cv::Point> &contour1, const std::vector<cv::Point> &contour2)
    {
        double area1 = cv::contourArea(contour1);
        double area2 = cv::contourArea(contour2);
        return 1.0 - std::abs(area1 - area2) / std::max(area1, area2);
    }

    double CalculateAreaSimilarity(const cv::Rect &rect1, const cv::Rect &rect2)
    {
        double area1 = rect1.area();
        double area2 = rect2.area();
        return 1.0 - std::abs(area1 - area2) / std::max(area1, area2);
    }

    // 计算质心相似度
    double CalculatePositionSimilarity(const cv::Point2f &centroid1, const cv::Point2f &centroid2) 
    {
        double dist;
        // dist = cv::norm(centroid1 - centroid2); // 计算质心之间的欧几里得距离
        const static float xWeight = 0.2;
        const static float yWeight = 0.8;
        dist = xWeight * std::abs(centroid1.x - centroid2.x) + yWeight * std::abs(centroid1.y - centroid2.y);
        return 1.0 / (1.0 + dist); // 距离越小，相似度越高
    }

    // 计算质心相似度
    double CalculatePositionSimilarity(const std::vector<cv::Point> &contour1, const std::vector<cv::Point> &contour2)
    {
        cv::Point2f centroid1 = CalculateCentroid(contour1);
        cv::Point2f centroid2 = CalculateCentroid(contour2);
        return CalculatePositionSimilarity(centroid1, centroid2);
    }

    double CalculatePositionSimilarity(const cv::Rect &rect1, const cv::Rect &rect2) 
    {
        cv::Point2f centroid1 = rect1.tl();
        centroid1.x += 0.5 * rect1.width;
        centroid1.y += 0.5 * rect1.height;
        cv::Point2f centroid2 = rect2.tl();
        centroid2.x += 0.5 * rect2.width;
        centroid2.y += 0.5 * rect2.height;
        return CalculatePositionSimilarity(centroid1, centroid2);
    }

    std::vector<std::pair<int, int>> FindContourCorrespondence(
    const std::vector<std::vector<cv::Point>>& contours1, 
    const std::vector<std::vector<cv::Point>>& contours2);

    void DrawCountourID(cv::Mat &image, const std::vector<cv::Point> &contour, int ID, double fontscale = 1.0, cv::Scalar color = cv::Scalar(255, 0, 0), double thickness = 2)
    {
        cv::putText(image, std::to_string(ID), CalculateCentroid(contour), cv::FONT_HERSHEY_SIMPLEX, 
            fontscale, color,  thickness);
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

