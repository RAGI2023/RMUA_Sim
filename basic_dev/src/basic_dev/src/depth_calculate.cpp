#include "depth_calculate.hpp"
#include "ros/ros.h"
#include "sensor_msgs/Image.h"

int main(int argc, char **argv)
{
    ros::init(argc, argv, "depth_calculate_node"); // 初始化ros 节点，命名为 basic
    ros::NodeHandle nh;
    // 读取相机参数
    CameraParameter pr("/home/yin/RMUA_Sim/basic_dev/calibration_parameters.yaml");
    if (!pr.readParameters()){
        return -1;
    }

    ros::Subscriber sub_front_left = nh.subscribe<sensor_msgs::Image>("/airsim_node/drone_1/front_left/Scene", 1, std::bind(&CameraParameter::front_left_view_cb, &pr, std::placeholders::_1));
    ros::spin();
    
    return 0;
}