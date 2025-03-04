
#include "ros/console.h"
#include "ros/init.h"
#include "Eigen/Dense"
#include <iostream>
#include <string>
#include <vector>
#include <fstream>

int main(int argc, char **argv)
{
    ros::init(argc, argv, "pathReader");

    std::string path = "/home/yin/RMUA_Sim/basic_dev/src/basic_dev/paths/"; //路径文件夹
    int TrackNum = 1; //赛道序号
    std::string filePath = path + std::to_string(TrackNum) + ".csv";
    ROS_INFO("Reading path from %s", filePath.c_str());

    std::vector<Eigen::Vector3d> pathPoints; // 保存路径点
    
    std::ifstream file(filePath);
    if (!file){
        ROS_ERROR("failed to open file.");
        return -1;
    }
    
    std::string line;
    while (std::getline(file, line)) { // 读取一整行
        std::istringstream iss(line);
        std::vector<float> numbers;
        std::string value;
        Eigen::Vector3d position;
        
        // 读取前三个逗号分隔的浮点数
        for (int i = 0; i < 3 && std::getline(iss, value, ','); ++i) {
            numbers.push_back(std::stof(value));  // 将字符串转换为 float

        }

        // 确保成功读取了 3 个浮点数
        if (numbers.size() == 3) {
            position << numbers[0], numbers[1], numbers[2];
            pathPoints.push_back(position);
            std::cout << numbers[0] << ", " << numbers[1] << ", " << numbers[2] << std::endl;
        } else {
            std::cerr << "Error: Could not read three numbers from line: " << line << std::endl;
        }
    }
    file.close();

    return 0;

}