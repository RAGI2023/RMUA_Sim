#include "depth_calculate.hpp"
#include "ros/console.h"
#include "sensor_msgs/Image.h"
#include "sensor_msgs/Imu.h"
#include <functional>
#include <opencv2/calib3d.hpp>
#include <opencv2/core.hpp>
#include <opencv2/core/cvstd_wrapper.hpp>
#include <opencv2/core/mat.hpp>
#include <opencv2/core/types.hpp>
#include <opencv2/features2d.hpp>
#include <opencv2/highgui.hpp>
#include <opencv2/imgcodecs.hpp>
#include <opencv2/imgproc.hpp>
#include <string>
#include <vector>

int main(int argc, char **argv)
{
    ros::init(argc, argv, "depth_calculate_node"); // 初始化ros 节点，命名为 basic
    ros::NodeHandle nh;
    // 读取相机参数
    CameraParameter pr("/home/yin/RMUA_Sim/basic_dev/calibration_parameters.yaml");
    if (!pr.readParameters()){
        return -1;
    }

    
    // VideoPlayer right_player("airsim_node/drone_1/front_right/Scene", "right");
    // VideoPlayer left_player("airsim_node/drone_1/front_left/Scene", "left");


    // DepthGenerator dg("/airsim_node/drone_1/front_right/Scene", "/airsim_node/drone_1/front_left/Scene", 
        // "/depth_image", pr);
    DepthGenerator dg("/airsim_node/drone_1/front_scene", pr);
    
    ros::spin();
    
    return 0;
}

DepthGenerator::DepthGenerator(const std::string &right_topic, const std::string &left_topic, const std::string depth_topic, const CameraParameter &parameters) 
        : right_image_topic_(right_topic), left_image_topic_(left_topic),depth_topic_(depth_topic), parameters_(parameters) 
{
    int queue_size = 5;
    sub_left_ = nh_.subscribe<sensor_msgs::Image>(left_image_topic_, queue_size, 
        std::bind(&DepthGenerator::callback_left, this, std::placeholders::_1));
    sub_right_ = nh_.subscribe<sensor_msgs::Image>(right_image_topic_, queue_size, 
        std::bind(&DepthGenerator::callback_right, this, std::placeholders::_1));
    sub_imu_ = nh_.subscribe<sensor_msgs::Imu>("airsim_node/drone_1/imu/imu", 1, 
        std::bind(&DepthGenerator::callback_imu, this, std::placeholders::_1));
    pub_depth_ = nh_.advertise<sensor_msgs::Image>(depth_topic_, 1);

    depth_image_ = cv::Mat(480, 640, CV_32F, cv::Scalar(0));

}

DepthGenerator::DepthGenerator(const std::string &scene_topic, const CameraParameter &camera_parameter) : scene_topic_(scene_topic), parameters_(camera_parameter)
{
    sub_scene_ = nh_.subscribe<airsim_ros::Scene>(scene_topic_, 1, 
        std::bind(&DepthGenerator::callback_scene, this, std::placeholders::_1));
    // pub_depth_ = nh_.advertise<sensor_msgs::Image>("/depth_image", 1);
}

void DepthGenerator::process_image()
{

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

    
    // // 调试用，显示变换后的图像
    // cv::Mat combined_img;
    // cv::hconcat(mask_left, mask_right, combined_img);
    // cv::imshow("rectified", combined_img);
    // cv::waitKey(10);

    // // 计算视差,合成深度图
    // // cv:: Mat rectified_left_gray, rectified_right_gray;
    // // cv::cvtColor(rectified_right, rectified_right_gray, cv::COLOR_BGR2GRAY);
    // // cv::cvtColor(rectified_left, rectified_left_gray, cv::COLOR_BGR2GRAY);
    // cv::Mat disparity;
    // // //BM算法 
    // // int blockSize = 45; //必须是奇数
    // // int numDisparity = 80; // 必须是16的倍数
    // // int uniquenessRatio = 15; //
    // // cv::Ptr<cv::StereoBM> bm = cv::StereoBM::create(numDisparity, blockSize);
    // // // bm->compute(rectified_left_gray, rectified_right_gray, disparity);
    // // bm->compute(mask_left, mask_right, disparity);
    // // //SGBM算法
    // // int blockSize = 35; //必须是奇数
    // // int numDisparity = 128; // 必须是16的倍数
    // // int uniquenessRatio = 15; 
    // // cv::Ptr<cv::StereoSGBM> sgbm = cv::StereoSGBM::create(
    // //     0,                           // 最小视差
    // //     numDisparity,                // 视差范围
    // //     blockSize,                   // 块大小
    // //     0,   // P1：控制平滑度
    // //     0,  // P2：控制平滑度
    // //     20,                          // 视差图允许的最大差异
    // //     15,             // 唯一性比率
    // //     uniquenessRatio,                         
    // //     32,                          
    // //     1                            // speckleRange：孤立点范围
    // // );
    // // // sgbm->compute(rectified_left_gray, rectified_right_gray, disparity);
    // // sgbm->compute(mask_left, mask_right, disparity);
    // cv::Mat disparity_normalized;
    // cv::normalize(disparity, disparity_normalized, 0, 255, cv::NORM_MINMAX, CV_8U);
    // cv::imshow("disparity", disparity_normalized);
    // cv::waitKey(10);

    // 寻找特征点
    // ORB检测特征点
    cv::Ptr<cv::ORB> orb = cv::ORB::create();
    std::vector<cv::KeyPoint> keypoints_left, keypoints_right;
    cv::Mat descriptors_left, descriptors_right;
    orb->detectAndCompute(mask_left, cv::noArray(), keypoints_left, descriptors_left);
    orb->detectAndCompute(mask_right, cv::noArray(), keypoints_right, descriptors_right);
    // 匹配特征点
    cv::Ptr<cv::BFMatcher> bf = cv::BFMatcher::create(cv::NORM_HAMMING, true);
    std::vector<cv::DMatch> matches;
    bf->match(descriptors_left, descriptors_right, matches);
    // 画出匹配点
    cv::Mat img_matches;
    cv::drawMatches(mask_left, keypoints_left, mask_right, keypoints_right, matches, img_matches, 
                cv::Scalar::all(-1), cv::Scalar::all(-1), 
                std::vector<char>(), cv::DrawMatchesFlags::DRAW_RICH_KEYPOINTS);
    cv::imshow("Matches", img_matches);
    cv::waitKey(10);

}

void DepthGenerator::process_scene()
{
    //是否为空？
    if (last_scene_.left.data.empty() || last_scene_.right.data.empty()){
        ROS_WARN("Scene is empty. Skip this process.");
        return;
    }

    cv::Mat left_image, right_image;
    left_image = cv_bridge::toCvCopy(last_scene_.left, sensor_msgs::image_encodings::TYPE_8UC3)->image;
    right_image = cv_bridge::toCvCopy(last_scene_.right, sensor_msgs::image_encodings::TYPE_8UC3)->image;


    cv::Mat rectified_left, rectified_right;
    stereoRectification(parameters_.cameraMatrix_l, parameters_.distCoeffs_l,
                        parameters_.cameraMatrix_r, parameters_.distCoeffs_r,
                        parameters_.R, parameters_.T,
                        left_image.size(), left_image, right_image,
                        rectified_left, rectified_right);
    
    // 提取橙红色区域
    cv::Mat orange_left, orange_right;
    // ExtractOrange(rectified_left, orange_left);
    // ExtractOrange(rectified_right, orange_right);
    ExtractOrangeMask(rectified_left, orange_left);
    ExtractOrangeMask(rectified_right, orange_right);

    // 去除噪点 闭操作
    // cv::Mat closed_rihgt, closed_left;
    cv::Mat kernal = cv::Mat::ones(3, 3, CV_8U);
    cv::morphologyEx(orange_left, orange_left, cv::MORPH_CLOSE, kernal);
    cv::morphologyEx(orange_right, orange_right, cv::MORPH_CLOSE, kernal);

    // // 寻找特征点
    // cv::Ptr<cv::ORB> orb = cv::ORB::create();
    // std::vector<cv::KeyPoint> keypoints_left, keypoints_right;
    // cv::Mat descriptors_left, descriptors_right;
    // orb->detectAndCompute(orange_left, cv::noArray(), keypoints_left, descriptors_left);
    // orb->detectAndCompute(orange_right, cv::noArray(), keypoints_right, descriptors_right);
    // // 匹配特征点
    // cv::Ptr<cv::BFMatcher> bf = cv::BFMatcher::create(cv::NORM_HAMMING, true);
    // std::vector<cv::DMatch> matches;
    // bf->match(descriptors_left, descriptors_right, matches);

    // // 画出匹配点
    // cv::Mat img_matches;
    // cv::drawMatches(orange_left, keypoints_left, orange_right, keypoints_right, matches, img_matches, 
    //             cv::Scalar::all(-1), cv::Scalar::all(-1), 
    //             std::vector<char>(), cv::DrawMatchesFlags::DRAW_RICH_KEYPOINTS);
    // cv::imshow("Matches", img_matches);
    // if (cv::waitKey(10) == 's'){
    //     cv::imwrite("test.jpg", img_matches);
    // }

    // 提取直线， 赛道框
    // ROS_INFO("Lines detecting...");
    std::vector<cv::Vec4f> lines_left, lines_right;
    cv::Mat edges_left, edges_right;
    cv::Canny(orange_left, edges_left, 50, 150, 3);
    cv::Canny(orange_right, edges_right, 50, 150, 3);
    
    double rho = 1;
    double theta = CV_PI/180;
    int hf_threshold = 50;
    int minLineLength = 70, maxLineGap = 10;
    cv::HoughLinesP(edges_left, lines_left, rho, theta, hf_threshold, minLineLength, maxLineGap); 
    cv::HoughLinesP(edges_right, lines_right, rho, theta, hf_threshold, minLineLength, maxLineGap);

    std::vector<std::vector<cv::Point>> contours_left, contours_right;
    cv::findContours(edges_left, contours_left, cv::RETR_LIST, cv::CHAIN_APPROX_SIMPLE);
    cv::findContours(edges_right, contours_right, cv::RETR_LIST, cv::CHAIN_APPROX_SIMPLE);

    // // 显示轮廓，调试用
    // cv::Mat orange_left_color, orange_right_color;
    // cv::cvtColor(orange_left, orange_left_color, cv::COLOR_GRAY2BGR);
    // cv::cvtColor(orange_right, orange_right_color, cv::COLOR_GRAY2BGR);
    // cv::drawContours(orange_right_color, contours_right, -1, cv::Scalar(0, 255, 0), 2);

    // 滤除过小的轮廓
    std::vector<std::vector<cv::Point>> barrrier_left, barriers_right;
    double ratio_thresh = 8;
    double area_thresh = 200;
    for (const auto &contour : contours_left){
        if(cv::contourArea(contour) > area_thresh && CalculateAspectRatio(contour) < ratio_thresh){
            barrrier_left.push_back(contour);
        }
        // else { //显示过滤掉的轮廓的数据
        //     double area = cv::contourArea(contour);
        //     double ratio = CalculateAspectRatio(contour);
        //     cv::Moments m = cv::moments(contour);
        //     cv::Point center(m.m10 / m.m00, m.m01 / m.m00);
        //     std::ostringstream areaStream;
        //     areaStream << std::fixed << std::setprecision(2) << area; // 控制小数点后 2 位
        //     std::ostringstream ratioStream;
        //     ratioStream << std::fixed << std::setprecision(2) << ratio; // 控制小数点后 2 位
        //     if (ratio > ratio_thresh){
        //         cv::drawContours(orange_left_color, std::vector<std::vector<cv::Point>> {contour}, -1, cv::Scalar(255, 0, 0), 2);
        //         cv::putText(orange_left_color, ratioStream.str(), center,cv::FONT_HERSHEY_SIMPLEX, 0.3, cv::Scalar(255, 0, 0), 1);
        //     }else{
        //         cv::drawContours(orange_left_color, std::vector<std::vector<cv::Point>> {contour}, -1, cv::Scalar(0, 0, 255), 2);
        //         cv::putText(orange_left_color, areaStream.str(), center,cv::FONT_HERSHEY_SIMPLEX, 0.3, cv::Scalar(0, 0, 255), 1);
        //     }
        // }
    }
    for (const auto &contour : contours_right){
        if(cv::contourArea(contour) > area_thresh && CalculateAspectRatio(contour) < ratio_thresh){
            barriers_right.push_back(contour);
        }
    }

    // // 画出直线
    // cv::Mat line_img_left = cv::Mat::zeros(orange_left.size(), CV_8UC3);
    // cv::Mat line_img_right = cv::Mat::zeros(orange_right.size(), CV_8UC3);
    // for (size_t i = 0; i < lines_left.size(); i++){
    //     cv::Vec4f l = lines_left[i];
    //     cv::line(line_img_left, cv::Point(l[0], l[1]), cv::Point(l[2], l[3]), cv::Scalar(0, 0, 255), 3, cv::LINE_AA);
    // }
    // for (size_t i = 0; i < lines_right.size(); i++){
    //     cv::Vec4f l = lines_right[i];
    //     cv::line(line_img_right, cv::Point(l[0], l[1]), cv::Point(l[2], l[3]), cv::Scalar(0, 0, 255), 3, cv::LINE_AA);
    // }
    // // 画出轮廓
    // for(const auto &contour : barrrier_left){
    //     cv::drawContours(line_img_left, std::vector<std::vector<cv::Point>>{contour}, -1, cv::Scalar(0, 255, 0), 2);
        
    //     // // 在质心位置标注面积
    //     // cv::Moments m = cv::moments(contour);
    //     // cv::Point center(m.m10 / m.m00, m.m01 / m.m00);
    //     // double area = cv::contourArea(contour);
    //     // std::string areaText = std::to_string(static_cast<int>(area));
    //     // cv::putText(line_img_left, areaText, center, cv::FONT_HERSHEY_SIMPLEX, 0.3, cv::Scalar(255, 255, 255), 1);
    // }
    // for(const auto &contour : barriers_right){
    //     cv::drawContours(line_img_right, std::vector<std::vector<cv::Point>>{contour}, -1, cv::Scalar(0, 255, 0), 2);
    //     // // 在质心位置标注面积
    //     // cv::Moments m = cv::moments(contour);
    //     // cv::Point center(m.m10 / m.m00, m.m01 / m.m00);
    //     // double area = cv::contourArea(contour);
    //     // std::string areaText = std::to_string(static_cast<int>(area));
    //     // cv::putText(line_img_right, areaText, center, cv::FONT_HERSHEY_SIMPLEX, 0.3, cv::Scalar(255, 255, 255), 1);
    // }

    // cv::Mat combined_canny;
    // cv::hconcat(line_img_left, line_img_right, combined_canny);
    // cv::imshow("canny", combined_canny);


    // cv::Mat combined_img;
    // cv::hconcat(orange_left_color, orange_right_color, combined_img);
    // cv::imshow("rectified", combined_img);
    // cv::waitKey(10);


}

void DepthGenerator::ExtractOrangeMask(cv::Mat &img, cv::Mat &mask)
{
    cv::Mat hsv;
    cv::cvtColor(img, hsv, cv::COLOR_BGR2HSV);
    cv::Scalar low_bound(0, 50, 50);
    cv::Scalar high_bound(15, 255, 255);
    cv::inRange(hsv, low_bound, high_bound, mask);
}

void DepthGenerator::ExtractOrange(cv::Mat &img, cv::Mat &Output)
{
    cv::Mat mask;
    ExtractOrangeMask(img, mask);
    cv::Mat output;
    cv::cvtColor(img, Output, cv::COLOR_BGR2GRAY);
    Output = mask & Output;
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

double DepthGenerator::ContourSimilarity(const std::vector<cv::Point> &contour1, const std::vector<cv::Point> &contour2)
{
    // 面积 位置的权重
    double area_weight = 0.5;
    double position_weight = 0.5;

    // 权重归一化
    double weight_sum = area_weight + position_weight;
    area_weight /= weight_sum;
    position_weight /= weight_sum;

    return area_weight * CalculateAreaSimilarity(contour1, contour2) + position_weight * CalculatePositionSimilarity(contour1, contour2);
}

std::vector<std::pair<int, int>> DepthGenerator::FindContourCorrespondence(
    const std::vector<std::vector<cv::Point>>& contours1, 
    const std::vector<std::vector<cv::Point>>& contours2) 
{
    int n1 = contours1.size();
    int n2 = contours2.size();

    // 创建一个相似度矩阵
    std::vector<std::vector<double>> similarityMatrix(n1, std::vector<double>(n2));

    // 填充相似度矩阵
    for (int i = 0; i < n1; ++i) {
        for (int j = 0; j < n2; ++j) {
            similarityMatrix[i][j] = ContourSimilarity(contours1[i], contours2[j]);
        }
    }

    // 存储最终的匹配结果
    std::vector<std::pair<int, int>> correspondences;

    // 简单的贪心算法：根据相似度选择最佳匹配
    std::vector<bool> matched2(n2, false);  // 用于标记 contour2 中是否已匹配

    for (int i = 0; i < n1; ++i) {
        // 寻找与 contour1[i] 最相似的 contour2[j]
        int bestMatchIndex = -1;
        double bestSimilarity = -1.0;

        for (int j = 0; j < n2; ++j) {
            // 如果该轮廓尚未匹配，并且相似度较高
            if (!matched2[j] && similarityMatrix[i][j] > bestSimilarity) {
                bestMatchIndex = j;
                bestSimilarity = similarityMatrix[i][j];
            }
        }

        if (bestMatchIndex != -1) {
            correspondences.push_back({i, bestMatchIndex});
            matched2[bestMatchIndex] = true;  // 标记为已匹配
        }
    }

    return correspondences;
}
