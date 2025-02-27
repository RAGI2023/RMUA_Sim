#include <cmath>
#include <memory>
#include <opencv2/core/hal/interface.h>
#include <opencv2/core/mat.hpp>
#include <opencv2/core/types.hpp>
#include <opencv2/imgproc.hpp>
#include <ros/ros.h>
#include <string>
#include <utility>
#include <vector>
#include "detection_msgs/BoundingBox.h"
#include "detection_msgs/BoundingBoxes.h"
#include "message_filters/subscriber.h"
#include "message_filters/sync_policies/exact_time.h"
#include "message_filters/synchronizer.h"
#include "depth_calculate.hpp"
#include "ros/console.h"
#include "ros/package.h"

class Box
{
public:
    std::string class_name;
    cv::Rect rect;
};

cv::Rect Box2Rect(const detection_msgs::BoundingBox &box){
    return cv::Rect(box.xmin, box.ymin, box.xmax - box.xmin, box.ymax - box.ymin);
}

cv::Point RectCenter(const cv::Rect &rect) {return cv::Point(rect.x+rect.width/2, rect.y+rect.height/2);}

double BoxSimilarity(const Box &box1, const Box &box2)
{
    // 面积 位置的权重
    double area_weight = 0.9;
    double position_weight = 5;
    double ratio_weight = 1.0;

    // 权重归一化
    double weight_sum = area_weight + position_weight + ratio_weight;
    area_weight /= weight_sum;
    position_weight /= weight_sum;
    ratio_weight /= weight_sum;
    
    // double area_score = double(std::min(box1.rect.area(), box2.rect.area())) / std::max(box1.rect.area(), box2.rect.area());
    // area_score *= area_weight;
   
    double position_score = 1 / (1 + cv::norm(RectCenter(box1.rect) - RectCenter(box2.rect)));
    position_score *= position_weight;


    return position_score ;
}


std::vector<std::pair<int , int>> MatchRect(const std::vector<Box> &boxes_l, const std::vector<Box> &boxes_r)
{
    static const int MAX_DIST = 50;
    static const int INF = -2;
    static const int Y_THRESH = 2;
    static const int X_THRESH = 30;
    int n1 = boxes_l.size();
    int n2 = boxes_r.size();
    ROS_INFO("Matching...");
    // 创建一个相似度矩阵
    std::vector<std::vector<double>> similarityMatrix(n1, std::vector<double>(n2));

    // 填充相似度矩阵
    for (int i = 0; i < n1; ++i) {
        for (int j = 0; j < n2; ++j) {
            if (boxes_l[i].class_name != boxes_r[j].class_name 
                    || std::fabs(RectCenter(boxes_l[i].rect).y - RectCenter(boxes_r[j].rect).y) > Y_THRESH
                    || std::fabs(RectCenter(boxes_l[i].rect).x - RectCenter(boxes_r[j].rect).x) > X_THRESH) {
                similarityMatrix[i][j] = INF;
                continue;
            }
            similarityMatrix[i][j] = BoxSimilarity(boxes_l[i], boxes_r[j]);
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

void callback(const detection_msgs::BoundingBoxes::ConstPtr& msg_l, const detection_msgs::BoundingBoxes::ConstPtr& msg_r) 
{
    ROS_INFO("Received detection.");
    cv::Mat left(720, 960, CV_8UC3, cv::Scalar(0, 0, 0)), 
        right(720, 960, CV_8UC3, cv::Scalar(0, 0, 0));
    std::vector<Box> boxes_l, boxes_r;
    std::vector<cv::Point> center_l, center_r;
    
    for (detection_msgs::BoundingBox box : msg_l->bounding_boxes){
        Box b{box.Class, Box2Rect(box)};
        if (box.Class != "Track") continue;
        boxes_l.push_back(b);
        // center_l.push_back(RectCenter(b.rect));

        cv::Scalar color;
        if (box.Class == "Car"){
            color = cv::Scalar(255, 0, 0);
        }else if (box.Class == "Barrier") {
            color = cv::Scalar(0, 255, 0);
        }else { //赛道
            color = cv::Scalar(0, 0, 255);
        }
        cv::rectangle(left, b.rect, color);
        std::string str = std::to_string(RectCenter(b.rect).x) + ' ' + std::to_string(RectCenter(b.rect).y);
        cv::putText(right, str, RectCenter(b.rect), cv::FONT_HERSHEY_SIMPLEX, 1, cv::Scalar(255, 0, 0));
    }
    for (detection_msgs::BoundingBox box : msg_r->bounding_boxes){
        Box b{box.Class, Box2Rect(box)};
        if (box.Class != "Track") continue;
        
        boxes_r.push_back(b);
        // center_r.push_back(RectCenter(b.rect));

        cv::Scalar color;
        if (box.Class == "Car"){
            color = cv::Scalar(255, 0, 0);
        }else if (box.Class == "Barrier") {
            color = cv::Scalar(0, 255, 0);
        }else { //赛道
            color = cv::Scalar(0, 0, 255);
        }
        cv::rectangle(right, b.rect, color);
        std::string str = std::to_string(RectCenter(b.rect).x) + ' ' + std::to_string(RectCenter(b.rect).y);
        cv::putText(left, str, RectCenter(b.rect), cv::FONT_HERSHEY_SIMPLEX, 1, cv::Scalar(255, 0, 0));
    }

    
    cv::Mat combined;
    cv::hconcat(left, right, combined);
    std::vector<std::pair<int , int >> correspondences = MatchRect(boxes_l, boxes_r);
    for (const auto &correspondence : correspondences){
        cv::Point2f right_tmp = RectCenter(boxes_r[correspondence.second].rect);
        right_tmp.x += left.cols;
        cv::line(combined, RectCenter(boxes_l[correspondence.second].rect), right_tmp, cv::Scalar(255, 255, 255), 1);
    }
    cv::imshow("detection", combined);
    cv::waitKey(1);

}

int main(int argc, char **argv) {
    ros::init(argc, argv, "detection_process");
    ros::NodeHandle nh;
    
    // 获取basic_dev包中的相机参数
    std::string pack_path = ros::package::getPath("basic_dev");
    CameraParameter pr(pack_path + "/../../calibration_parameters.yaml");
    if (!pr.readParameters()){
        ROS_ERROR("Failed to read camera parameters.");
        return -1;
    }

    message_filters::Subscriber<detection_msgs::BoundingBoxes> sub_l(nh, "/yolov5/detections_left", 1);
    message_filters::Subscriber<detection_msgs::BoundingBoxes> sub_r(nh, "/yolov5/detections_right", 1);
    std::unique_ptr<message_filters::Synchronizer<message_filters::sync_policies::ExactTime<detection_msgs::BoundingBoxes, detection_msgs::BoundingBoxes>>>sync_handler_ptr
        = std::make_unique<message_filters::Synchronizer<message_filters::sync_policies::ExactTime<detection_msgs::BoundingBoxes, detection_msgs::BoundingBoxes>>>(
        message_filters::sync_policies::ExactTime<detection_msgs::BoundingBoxes, detection_msgs::BoundingBoxes>(1), sub_l, sub_r);
    sync_handler_ptr->registerCallback(callback);

    ros::spin();
    
    return 0;
}

