#include "octomap/OcTree.h"
#include "Eigen/Dense"
#include "ros/console.h"
#include "ros/init.h"
#include "ros/node_handle.h"
#include "ros/publisher.h"
#include "ros/subscriber.h"
#include "sensor_msgs/PointCloud2.h"

#include <cmath>
#include <memory>
#include <mutex>
#include <octomap_msgs/OctomapWithPose.h>
#include <octomap_msgs/Octomap.h>
#include <geometry_msgs/Pose.h>

#include <octomap/octomap.h>
#include <octomap_msgs/Octomap.h>
#include <octomap_msgs/conversions.h>

#include <pcl_conversions/pcl_conversions.h>

/**
 * @brief 订阅雷达消息，CheckAndFind函数 查看该点附近是否被占用，若被占用则寻找新点
 * 
 */
class OctreeProcess
{
public:
    OctreeProcess(const float resolution = 0.05) : resolution_(resolution)
    {
        sub_ = nh_.subscribe("/airsim_node/drone_1/lidar", 1, &OctreeProcess::pointCloudCallback, this);
        pub_ = nh_.advertise<octomap_msgs::Octomap>("/octomap", 1);
    }
    
    void publishOctomap(const octomap::OcTree &tree)
    {
        octomap_msgs::Octomap octomap_msg;
        octomap_msg.header.frame_id = "lidar";
        octomap_msg.binary = true;
        octomap_msg.id = "OcTree";
        octomap_msg.resolution = resolution_;
        octomap_msgs::fullMapToMsg(tree, octomap_msg);
        pub_.publish(octomap_msg);

    }

    void setResolution(const float resolution) {resolution_ = resolution;}

    /**
     * @brief 查看该点附近是否被占用，若被占用则寻找新点
     * 
     * @param target_point 
     * @return Eigen::Vector3d Eigen::Vector3d (0, 0, 0)表示未找到新点
     */
    Eigen::Vector3d CheckAndFind(Eigen::Vector3d target_point)
    {
        // 转化到机体中心
        target_point.z() -= 0.05;
        Eigen::Vector3d ret;
        if (Detect(target_point)) {
            // return FindNewPoint(target_point);
            ret = FindNewPoint(target_point);
        } else {
            ret = target_point;
            // return target_point;
        }

        return ret;
    }

private:
    ros::NodeHandle nh_;
    ros::Subscriber sub_;
    ros::Publisher pub_;
    float resolution_;
    std::mutex mutex_;
    std::shared_ptr<octomap::OcTree> octree_ptr;
    
    void pointCloudCallback(const sensor_msgs::PointCloud2::ConstPtr &msg)
    {
        std::lock_guard<std::mutex> lock(mutex_);

        ROS_INFO("Received point cloud message");
        pcl::PointCloud<pcl::PointXYZ>::Ptr pts(new pcl::PointCloud<pcl::PointXYZ>);
        pcl::fromROSMsg(*msg, *pts);
        
        octree_ptr = std::make_shared<octomap::OcTree>(resolution_);
        // octomap::OcTree tree(resolution_);
        for (const auto &point : pts->points){
            octree_ptr->updateNode(point.x, point.y, point.z, true);
        }

        publishOctomap(*octree_ptr);


    }

    /**
     * @brief 查看目标点附近是否有占据的栅格，雷达坐标系
     * 
     * @param target_point 查看的目标点
     * @param radius_xy 水平面半径
     * @param radius_z 竖直方向半径
     * @return true 有占据的栅格
     * @return false 无占据的栅格
     */
    bool Detect(Eigen::Vector3d target_point, const double radius_xy = 1.0, const double radius_z = 0.3)
    {
        std::lock_guard<std::mutex> lock(mutex_);


        bool occupied = false;
        for (double x = target_point.x() - radius_xy; x <= target_point.x() + radius_xy; x += resolution_) {
            for (double y = target_point.y() - radius_xy; y <= target_point.y() + radius_xy; y += resolution_) {
                for (double z = target_point.z() - radius_z; z <= target_point.z() + radius_z; z += resolution_) {
                    Eigen::Vector3d current_point(x, y, z);
                    // 检查该位置是否被占据
                    octomap::OcTreeNode* node = octree_ptr->search(x, y, z);
                    if (node != nullptr && octree_ptr->isNodeOccupied(node)) {
                        occupied = true;
                        break; // 一旦找到占据的栅格，退出
                    }
                }
                if (occupied) break;
            }
            if (occupied) break;
        }

        return occupied;


    }

    /**
     * @brief 寻找目标点附近的一个新点
     * 
     * @param target_point 目标点
     * @return Eigen::Vector3d (0, 0, 0)未找到新点
     */
    Eigen::Vector3d FindNewPoint(const Eigen::Vector3d& target_point)
    {
        Eigen::Vector3d target = target_point;

        const static double delta = 0.05;
        double detect = delta;
        target.y() += delta;
        while (Detect(target)) {
            //恢复到原来的位置
            target.y() -= detect;
            // 翻转并增大 detect
            detect = -detect + (detect > 0 ? delta : -delta);
            target.y() += detect;

            if (std::fabs(detect) > 7.0){
                ROS_WARN("No available grid found near the target point.");
                return Eigen::Vector3d(0, 0, 0);

            }
        }
        return target;
    }
};

int main(int argc, char **argv)
{
    ros::init(argc, argv, "occupancyGrid");
    ros::NodeHandle nh;
    
    OctreeProcess occupancyGrid(0.5);
    ros::spin();

    return 0;

}