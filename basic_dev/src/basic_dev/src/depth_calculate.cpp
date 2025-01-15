#include "depth_calculate.hpp"
#include "ros/ros.h"
#include "sensor_msgs/Image.h"
#include "sensor_msgs/Imu.h"
#include <functional>
#include <opencv2/calib3d.hpp>
#include <opencv2/core.hpp>
#include <opencv2/core/cvstd_wrapper.hpp>
#include <opencv2/core/mat.hpp>
#include <opencv2/core/types.hpp>
#include <opencv2/highgui.hpp>
#include <opencv2/imgproc.hpp>

int main(int argc, char **argv)
{
    ros::init(argc, argv, "depth_calculate_node"); // 初始化ros 节点，命名为 basic
    ros::NodeHandle nh;
    // 读取相机参数
    CameraParameter pr("/home/yin/RMUA_Sim/basic_dev/calibration_parameters.yaml");
    if (!pr.readParameters()){
        return -1;
    }

    
    DepthGenerator dg("/airsim_node/drone_1/front_right/Scene", "/airsim_node/drone_1/front_left/Scene", "/depth_image", pr);
    // ros::Subscriber sub_front_left = nh.subscribe<sensor_msgs::Image>("/airsim_node/drone_1/front_left/Scene", 1, std::bind(&CameraParameter::front_left_view_cb, &pr, std::placeholders::_1));
    ros::spin();
    
    return 0;
}

DepthGenerator::DepthGenerator(const std::string &right_topic, const std::string &left_topic, const std::string depth_topic, const CameraParameter &parameters) 
        : right_image_topic_(right_topic), left_image_topic_(left_topic),depth_topic_(depth_topic), parameters_(parameters) 
{
    sub_left_ = nh_.subscribe<sensor_msgs::Image>(left_image_topic_, 1, 
        std::bind(&DepthGenerator::callback_left, this, std::placeholders::_1));
    sub_right_ = nh_.subscribe<sensor_msgs::Image>(right_image_topic_, 1, 
        std::bind(&DepthGenerator::callback_right, this, std::placeholders::_1));
    sub_imu_ = nh_.subscribe<sensor_msgs::Imu>("airsim_node/drone_1/imu/imu", 1, 
        std::bind(&DepthGenerator::callback_imu, this, std::placeholders::_1));
    pub_depth_ = nh_.advertise<sensor_msgs::Image>(depth_topic_, 1);

    depth_image_ = cv::Mat(480, 640, CV_32F, cv::Scalar(0));

}

void DepthGenerator::process_image()
{
    // 是否为空？
    if (left_image_topic_.empty() || right_image_topic_.empty()){
        ROS_WARN("Image topic is empty\n");
        return;
    }
    // 检查时间戳
    if (fabs(right_time_ - left_time_) > max_time_diff){
        ROS_WARN("Time difference between left and right image is too large. Skip this process. %lf %lf", right_time_, left_time_);
        return;
    }

    cv::Mat left_image, right_image;
    try {
        left_image = last_left_img_->image;
        right_image = last_right_img_->image;
    } catch (const cv_bridge::Exception& e) {
        ROS_ERROR("cv_bridge exception: %s", e.what());
        return;
    } catch (const std::exception& e) {
        ROS_ERROR("Standard exception: %s", e.what());
        return;
    } catch (...) {
        ROS_ERROR("Unknown exception occurred while processing images.");
        return;
    }
    
    ROS_INFO("Processing...");

    cv::Mat rectified_left, rectified_right;
    stereoRectification(parameters_.cameraMatrix_l, parameters_.distCoeffs_l,
                        parameters_.cameraMatrix_r, parameters_.distCoeffs_r,
                        parameters_.R, parameters_.T,
                        left_image.size(), left_image, right_image,
                        rectified_left, rectified_right);

    // 提取橙红色区域
    cv::Mat hsv_left_rectified, hsv_right_rectified;
    cv::cvtColor(rectified_left, hsv_left_rectified, cv::COLOR_BGR2HSV);
    cv::cvtColor(rectified_right, hsv_right_rectified, cv::COLOR_BGR2HSV);
    cv::Mat mask_left, mask_right;
    cv::Scalar low_bound(0, 50, 50);
    cv::Scalar high_bound(15, 255, 255);
    cv::inRange(hsv_left_rectified, low_bound, high_bound, mask_left);
    cv::inRange(hsv_right_rectified, low_bound, high_bound, mask_right);

    
    // 调试用，显示变换后的图像
    cv::Mat combined_img;
    cv::hconcat(mask_left, mask_right, combined_img);
    cv::imshow("rectified", combined_img);
    // cv::waitKey(10);

    // 计算视差,合成深度图
    // cv:: Mat rectified_left_gray, rectified_right_gray;
    // cv::cvtColor(rectified_right, rectified_right_gray, cv::COLOR_BGR2GRAY);
    // cv::cvtColor(rectified_left, rectified_left_gray, cv::COLOR_BGR2GRAY);
    cv::Mat disparity;
    
    //BM算法 
    int blockSize = 45; //必须是奇数
    int numDisparity = 80; // 必须是16的倍数
    int uniquenessRatio = 15; //
    cv::Ptr<cv::StereoBM> bm = cv::StereoBM::create(numDisparity, blockSize);
    // bm->compute(rectified_left_gray, rectified_right_gray, disparity);
    bm->compute(mask_left, mask_right, disparity);

    // //SGBM算法
    // int blockSize = 35; //必须是奇数
    // int numDisparity = 128; // 必须是16的倍数
    // int uniquenessRatio = 15; 
    // cv::Ptr<cv::StereoSGBM> sgbm = cv::StereoSGBM::create(
    //     0,                           // 最小视差
    //     numDisparity,                // 视差范围
    //     blockSize,                   // 块大小
    //     0,   // P1：控制平滑度
    //     0,  // P2：控制平滑度
    //     20,                          // 视差图允许的最大差异
    //     15,             // 唯一性比率
    //     uniquenessRatio,                         
    //     32,                          
    //     1                            // speckleRange：孤立点范围
    // );
    // // sgbm->compute(rectified_left_gray, rectified_right_gray, disparity);
    // sgbm->compute(mask_left, mask_right, disparity);

    cv::Mat disparity_normalized;
    cv::normalize(disparity, disparity_normalized, 0, 255, cv::NORM_MINMAX, CV_8U);
    cv::imshow("disparity", disparity_normalized);
    cv::waitKey(10);
}

double DepthGenerator::get_currenttime(const sensor_msgs::ImageConstPtr &msg)
{
    return msg->header.stamp.toSec() + msg->header.stamp.toNSec() * 1e-9;
}

double DepthGenerator::get_currenttime()
{
    return timestamp_;
}

void DepthGenerator::stereoRectification(const cv::Mat& cameraMatrix_l, const cv::Mat& distCoeffs_l,
                         const cv::Mat& cameraMatrix_r, const cv::Mat& distCoeffs_r,
                         const cv::Mat& R, const cv::Mat& T,
                         const cv::Size& img_size, const cv::Mat& img_l, const cv::Mat& img_r,
                         cv::Mat& rectified_img_l, cv::Mat& rectified_img_r)
{
    // 立体校正
    cv::Mat R1, R2, P1, P2, Q;
    cv::stereoRectify(cameraMatrix_l, distCoeffs_l,
                      cameraMatrix_r, distCoeffs_r,
                      img_size, R, T, R1, R2, P1, P2, Q);

    // 计算矫正映射
    cv::Mat map1x, map1y, map2x, map2y;
    cv::initUndistortRectifyMap(cameraMatrix_l, distCoeffs_l, R1, P1, img_size, CV_32FC1, map1x, map1y);
    cv::initUndistortRectifyMap(cameraMatrix_r, distCoeffs_r, R2, P2, img_size, CV_32FC1, map2x, map2y);

    // 应用映射矩阵，校正图像
    cv::remap(img_l, rectified_img_l, map1x, map1y, cv::INTER_LINEAR);
    cv::remap(img_r, rectified_img_r, map2x, map2y, cv::INTER_LINEAR);
}
