#include "depth_calculate.hpp"

int main(int argc, char **argv)
{
    ros::init(argc, argv, "depth_calculate_node"); // 初始化ros 节点，命名为 basic
    ros::NodeHandle nh;
    // 读取相机参数
    CameraParameter pr("/home/yin/RMUA_Sim/basic_dev/calibration_parameters.yaml");
    if (!pr.readParameters()){
        return -1;
    }
    
    std::string topic;
    if (!nh.getParam("topic", topic)) {
        ROS_ERROR("Could not get 'topic' parameter from launch file, using default.");
        return -1;
    }
    std::string file_path;
    if (!nh.getParam("file_path", file_path)) {
        ROS_ERROR("Could not get 'file_path' parameter from launch file, using default.");
        return -1;
    }
    int index;
    if (!nh.getParam("index", index)) {
        ROS_WARN("Could not get 'index' parameter from launch file, using default.");
        index = 0;  // 默认值
    }

    VideoPlayer player(topic, "player");
    player.SetFilePath(file_path);
    player.set_index(index);
    
    ros::spin();
    
    return 0;
}