#pragma once

#include "ros/console.h"
#include "ros/node_handle.h"
#include <opencv2/core/mat.hpp>
#include <opencv2/core/persistence.hpp>
#include <string>
#include "sensor_msgs/Image.h"
#include "cv_bridge/cv_bridge.h"

class CameraParameter
{
public:
    CameraParameter(const std::string &file_path) : file_path_(file_path) {};
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
    DepthGenerator(const std::string &right_topic, const std::string &left_topic, const CameraParameter &paremeters);
    ~DepthGenerator() {};

private:
    ros::NodeHandle nh_;
    ros::Subscriber sub_left_, sub_right_;
    ros::Publisher pub_depth_;
    std::string left_image_topic_, right_image_topic_;
    
    CameraParameter paremeters_;

    sensor_msgs::ImageConstPtr last_left_msg_, last_right_msg_;

    cv::Mat depth_image_;

    const double max_time_diff = 0.1;

};