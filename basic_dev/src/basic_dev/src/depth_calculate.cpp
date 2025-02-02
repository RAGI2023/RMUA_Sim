#include "depth_calculate.hpp"
#include "sensor_msgs/Image.h"
#include <cmath>
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
#include <opencv4/opencv2/core/mat.hpp>
#include <string>
#include <utility>
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

    DepthGenerator dg("/airsim_node/drone_1/front_right/Scene", "/airsim_node/drone_1/front_left/Scene", 
        "/depth_image", pr);
    
    ros::spin();
    
    return 0;
}

DepthGenerator::DepthGenerator(const std::string &right_topic, const std::string &left_topic, const std::string depth_topic, const CameraParameter &parameters) 
        : right_image_topic_(right_topic), left_image_topic_(left_topic),depth_topic_(depth_topic), parameters_(parameters), 
        fl_image_suber_(nh_, left_image_topic_, 10), fr_image_suber_(nh_, right_image_topic_, 10)
{
    int queue_size = 5;
    // sub_left_ = nh_.subscribe<sensor_msgs::Image>(left_image_topic_, queue_size, 
    //     std::bind(&DepthGenerator::callback_left, this, std::placeholders::_1));
    // sub_right_ = nh_.subscribe<sensor_msgs::Image>(right_image_topic_, queue_size, 
    //     std::bind(&DepthGenerator::callback_right, this, std::placeholders::_1));
    sync_handler_ptr = std::make_unique<message_filters::Synchronizer<message_filters::sync_policies::ExactTime<sensor_msgs::Image, sensor_msgs::Image>>>(
            message_filters::sync_policies::ExactTime<sensor_msgs::Image, sensor_msgs::Image>(1), fl_image_suber_, fr_image_suber_);
    sync_handler_ptr->registerCallback(std::bind(&DepthGenerator::stereo_view_cb, this, std::placeholders::_1, std::placeholders::_2));
    // sub_imu_ = nh_.subscribe<sensor_msgs::Imu>("airsim_node/drone_1/imu/imu", 1, 
    //     std::bind(&DepthGenerator::callback_imu, this, std::placeholders::_1));
    pub_depth_ = nh_.advertise<sensor_msgs::Image>(depth_topic_, 1);

    depth_image_ = cv::Mat(480, 640, CV_32F, cv::Scalar(0));

}

void DepthGenerator::process_scene(cv::Mat &left_image, cv::Mat &right_image)
{
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
    cv::Mat kernal = cv::Mat::ones(3, 3, CV_8U);
    cv::morphologyEx(orange_left, orange_left, cv::MORPH_CLOSE, kernal);
    cv::morphologyEx(orange_right, orange_right, cv::MORPH_CLOSE, kernal);

    cv::Mat kernerl_erose = cv::Mat::ones(3, 3, CV_8U);
    cv::erode(orange_left, orange_left, kernerl_erose);
    cv::erode(orange_right, orange_right, kernerl_erose);

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

    // 膨胀，避免轮廓不闭合
    // cv::Mat kernel = cv::Mat::ones(4, 4, CV_8U);
    // cv::dilate(edges_left, edges_left, kernel);
    // cv::dilate(edges_right, edges_right, kernel);
    
    cv::Mat orange_left_color, orange_right_color;
    cv::cvtColor(orange_left, orange_left_color, cv::COLOR_GRAY2BGR);
    cv::cvtColor(orange_right, orange_right_color, cv::COLOR_GRAY2BGR);
    
    std::vector<std::vector<cv::Point>> contours_left, contours_right;
    std::vector<cv::Rect> rects_left, rects_right;
    std::vector<std::vector<cv::Point>> contours_hierarchy_left, contours_hierarchy_right;
    std::vector<cv::Vec4i> hierarchy_left, hierarchy_right;
    cv::findContours(edges_left, contours_hierarchy_left, hierarchy_left, cv::RETR_TREE, cv::CHAIN_APPROX_SIMPLE);
    cv::findContours(edges_right, contours_hierarchy_right, hierarchy_right, cv::RETR_TREE, cv::CHAIN_APPROX_SIMPLE);
    // cv::findContours(edges_left, contours_left, cv::RETR_LIST, cv::CHAIN_APPROX_SIMPLE);
    // cv::findContours(edges_right, contours_right, cv::RETR_LIST, cv::CHAIN_APPROX_SIMPLE);

    for (size_t i = 0; i < contours_hierarchy_left.size(); i++) {
        if (hierarchy_left[i][3] == -1) { // 没有父轮廓，是最外层轮廓
            // 处理最外层轮廓
            // cv::drawContours(image, contours, i, cv::Scalar(0, 255, 0), 2); // 绘制轮廓
            // std::cout << "Contour " << i << " is an outer contour." << std::endl;
            contours_left.push_back(contours_hierarchy_left[i]);
        }
    }
    for (size_t i = 0; i < contours_hierarchy_right.size(); i++) {
        if (hierarchy_right[i][3] == -1) { // 没有父轮廓，是最外层轮廓
            // 处理最外层轮廓
            // cv::drawContours(image, contours, i, cv::Scalar(0, 255, 0), 2); // 绘制轮廓
            // std::cout << "Contour " << i << " is an outer contour." << std::endl;
            contours_right.push_back(contours_hierarchy_right[i]);
        }
    }

    // 轮廓近似
    std::vector<std::vector<cv::Point>> approxContours_left(contours_left.size());
    std::vector<std::vector<cv::Point>> approxContours_right(contours_right.size());
    const static double epsilon = 3; // 近似精度，值越小越接近原始轮廓
    for (size_t i = 0; i < contours_left.size(); i++) {
        cv::approxPolyDP(contours_left[i], approxContours_left[i], epsilon, true);
    }
    for (size_t i = 0; i < contours_right.size(); i++) {
        cv::approxPolyDP(contours_right[i], approxContours_right[i], epsilon, true);
    }
    contours_left = approxContours_left;
    contours_right = approxContours_right;
    

    // 滤除过小的轮廓 瘦长 竖直的矩形
    std::vector<std::vector<cv::Point>> barrriers_left, barriers_right;
    // static const double ratio_thresh = 2;
    // static const double area_thresh = 100;
    // static const double area_ratio_thresh = 0.5;
    for (const auto &contour : contours_left){
        // if(rect.area() > area_thresh && CalculateAspectRatio(rect) < ratio_thresh){
        //     barrriers_left.push_back(contour);
        //     rects_left.push_back(rect);
        // }

        if (BarrierFileter(contour)){
            barrriers_left.push_back(contour);
            rects_left.push_back(cv::boundingRect(contour));
        }

        // if(cv::contourArea(contour) > area_thresh && CalculateAspectRatio(cv::boundingRect(contour)) < ratio_thresh){
        //     barrriers_left.push_back(contour);
        //     rects_left.push_back(cv::boundingRect(contour));
        // }
    }
    for (const auto &contour : contours_right){
        // if(cv::contourArea(contour) > area_thresh && CalculateAspectRatio(cv::boundingRect(contour)) < ratio_thresh){
        //     barriers_right.push_back(contour);
        //     rects_right.push_back(cv::boundingRect(contour));
        // }

        // double area = cv::contourArea(contour);
        // cv::Rect rect = cv::boundingRect(contour);
        // if(rect.area() > area_thresh && CalculateAspectRatio(rect) < ratio_thresh && area > area_ratio_thresh * rect.area()){
        //     barriers_right.push_back(contour);
        //     rects_right.push_back(rect);
        // }

        if (BarrierFileter(contour)){
            barriers_right.push_back(contour);
            rects_right.push_back(cv::boundingRect(contour));
        }
    }

    // 显示轮廓，调试用
    
    cv::drawContours(orange_left_color, contours_left, -1, cv::Scalar(0, 0, 255), 2);
    cv::drawContours(orange_left_color, barrriers_left, -1, cv::Scalar(0, 255, 0), 2);
    cv::drawContours(orange_right_color, barriers_right, -1, cv::Scalar(0, 255, 0), 2);
    for(const auto &contour : contours_left){
        // DrawRotatedRect(orange_left_color, contour);
        DrawRect(orange_left_color, contour);
    }
    for(const auto &rect : rects_right)
    {
        cv::rectangle(orange_right_color, rect, cv::Scalar(255, 0, 0), 2);
    }
    // 画出直线
    cv::Mat line_img_left = cv::Mat::zeros(orange_left.size(), CV_8UC3);
    cv::Mat line_img_right = cv::Mat::zeros(orange_right.size(), CV_8UC3);
    // for (size_t i = 0; i < lines_left.size(); i++){
    //     cv::Vec4f l = lines_left[i];
    //     cv::line(line_img_left, cv::Point(l[0], l[1]), cv::Point(l[2], l[3]), cv::Scalar(0, 0, 255), 3, cv::LINE_AA);
    // }
    // for (size_t i = 0; i < lines_right.size(); i++){
    //     cv::Vec4f l = lines_right[i];
    //     cv::line(line_img_right, cv::Point(l[0], l[1]), cv::Point(l[2], l[3]), cv::Scalar(0, 0, 255), 3, cv::LINE_AA);
    // }
    // 画出轮廓
    for(const auto &contour : barrriers_left){
        cv::drawContours(line_img_left, std::vector<std::vector<cv::Point>>{contour}, -1, cv::Scalar(0, 255, 0), 2);
    }
    for(const auto &contour : barriers_right){
        cv::drawContours(line_img_right, std::vector<std::vector<cv::Point>>{contour}, -1, cv::Scalar(0, 255, 0), 2);
    }
    
    cv::Mat combined_img;
    cv::hconcat(orange_left_color, orange_right_color, combined_img);
    std::vector<cv::Point2f> left_points, right_points;
    for (const auto &contour : barrriers_left){
        cv::Point2f centroid = CalculateCentroid(contour);
        left_points.push_back(centroid);
    }
    for (const auto &contour : barriers_right){
        cv::Point2f centroid = CalculateCentroid(contour);
        right_points.push_back(centroid);
    }
    std::vector<std::pair<int , int >> correspondences = FindContourCorrespondence(barrriers_left, barriers_right);
    for (const auto &correspondence : correspondences){
        cv::Point2f right_tmp = right_points[correspondence.second];
        right_tmp.x += line_img_left.cols;
        cv::line(combined_img, left_points[correspondence.first], right_tmp, cv::Scalar(0, 0, 255), 1);
    }
    
    cv::imshow("rectified", combined_img);
    cv::waitKey(10);


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
    double position_weight = 0.7;

    // 权重归一化
    double weight_sum = area_weight + position_weight;
    area_weight /= weight_sum;
    position_weight /= weight_sum;

    return area_weight * CalculateAreaSimilarity(contour1, contour2) + position_weight * CalculatePositionSimilarity(contour1, contour2);
}

double DepthGenerator::ContourSimilarity(const cv::Rect &rect1, const cv::Rect &rect2)
{
    // 面积 位置的权重
    double area_weight = 0.5;
    double position_weight = 0.7;

    // 权重归一化
    double weight_sum = area_weight + position_weight;
    area_weight /= weight_sum;
    position_weight /= weight_sum;

    return area_weight * CalculateAreaSimilarity(rect1, rect2) + position_weight * CalculatePositionSimilarity(rect1, rect2);
}

std::vector<std::pair<int, int>> DepthGenerator::FindContourCorrespondence(
    const std::vector<std::vector<cv::Point>>& contours1, 
    const std::vector<std::vector<cv::Point>>& contours2) 
{
    static const int Y_THRESH = 20;
    static const int INF = -2;
    int n1 = contours1.size();
    int n2 = contours2.size();

    // 创建一个相似度矩阵
    std::vector<std::vector<double>> similarityMatrix(n1, std::vector<double>(n2));

    // 填充相似度矩阵
    for (int i = 0; i < n1; ++i) {
        for (int j = 0; j < n2; ++j) {
            if (std::fabs(CalculateCentroid(contours1[i]).y - CalculateCentroid(contours2[j]).y) > Y_THRESH) {
                similarityMatrix[i][j] = INF;
                continue;
            }
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
            if (!matched2[j] && similarityMatrix[i][j] != INF && similarityMatrix[i][j] > bestSimilarity) {
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
