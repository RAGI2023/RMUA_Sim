#include "geometry_msgs/PoseStamped.h"
#include "nav_msgs/Odometry.h"
#include "nav_msgs/Path.h"
#include "ros/console.h"
#include "ros/init.h"
#include "ros/node_handle.h"
#include "ros/publisher.h"
#include "ros/subscriber.h"
#include <string>
#include <vector>
#include <fstream>

class PathRecorder
{
public:
    PathRecorder(std::string odom_topic = "/airsim_node/drone_1/debug/pose_gt", const int dense = 5, std::string path_topic = "/airsim_node/drone_1/path", const std::string &file_path = "/home/yin/RMUA_Sim/basic_dev/src/basic_dev/paths/01.csv")
    : file_path_(file_path)
    {
        sub_odom_ = nh_.subscribe<geometry_msgs::PoseStamped>(odom_topic, 1, std::bind(&PathRecorder::odom_cb, this, std::placeholders::_1));
        pub_path_ = nh_.advertise<nav_msgs::Path>(path_topic, 1);
        path_.poses.clear();
        path_.header.frame_id = "map";
        this->dense = dense;
        count_ = 1;
    }

    void setFilePath(const std::string &file_path)
    {
        file_path_ = file_path;
    }

private:
    int dense;
    ros::NodeHandle nh_;
    ros::Subscriber sub_odom_;
    ros::Publisher pub_path_;
    nav_msgs::Path path_;
    unsigned int count_;
    std::string file_path_;
    geometry_msgs::PoseStamped last_pose_;
    bool first_ = true;

    double distance(const geometry_msgs::PoseStamped &p1, const geometry_msgs::PoseStamped &p2)
    {
        return sqrt(pow(p1.pose.position.x - p2.pose.position.x, 2) + pow(p1.pose.position.y - p2.pose.position.y, 2) + pow(p1.pose.position.z - p2.pose.position.z, 2));
    }

    void odom_cb(const geometry_msgs::PoseStampedConstPtr &msg)
    {
        // ROS_INFO("callback.");
        if (count_++ % dense != 0){
            return;
        }
        // ROS_INFO("Recording path...");
        geometry_msgs::PoseStamped pose_stamped;
        pose_stamped.header = msg->header;  // 设置时间戳和坐标系
        pose_stamped.header.frame_id = "map";  // 设置坐标系
        pose_stamped.pose = msg->pose;  // 提取位置和姿态

        if (first_){
            last_pose_ = pose_stamped;
            first_ = false;
        }

        // 将该位姿添加到路径中
        if (distance(last_pose_, pose_stamped) > 10){
            first_ = false;
            path_.poses.push_back(pose_stamped);
            path_.header.stamp = msg->header.stamp;
            ROS_INFO("Write in.");
            last_pose_ = pose_stamped;

            std::ofstream path_file_;
            path_file_.open(file_path_, std::ios::out | std::ios::app);
            if (!path_file_.is_open()){
                ROS_ERROR("Failed to open file.");
                return;
            }
            path_file_ << pose_stamped.pose.position.x << ","      // x坐标
                       << pose_stamped.pose.position.y << ","      // y坐标
                       << pose_stamped.pose.position.z << ","      // z坐标
                       << pose_stamped.pose.orientation.x << ","   // roll
                       << pose_stamped.pose.orientation.y << ","   // pitch
                       << pose_stamped.pose.orientation.z << "\n"; // yaw
            path_file_.close();
        }
        // pub_path_.publish(path_);
    }

};

int main(int argc, char** argv)
{
    ros::init(argc, argv, "path_recorder");

    PathRecorder pr;
    ros::spin();
    return 0;
}