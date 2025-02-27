#pragma once

#include "pcl/point_cloud.h"
#include "ros/console.h"
#include "ros/node_handle.h"
#include "ros/publisher.h"
#include "ros/subscriber.h"
#include <ostream>
#include <string>
#include "sensor_msgs/PointCloud2.h"
#include <pcl_conversions/pcl_conversions.h>
#include "airsim_ros/Barrier.h"
#include "airsim_ros/Barriers.h"
#include "std_msgs/Header.h"
#include <pcl/ModelCoefficients.h>
#include <pcl/point_types.h>
#include <pcl/io/pcd_io.h>
#include <pcl/filters/extract_indices.h>
#include <pcl/filters/voxel_grid.h>
#include <pcl/features/normal_3d.h>
#include <pcl/search/kdtree.h>
#include <pcl/sample_consensus/method_types.h>
#include <pcl/sample_consensus/model_types.h>
#include <pcl/segmentation/sac_segmentation.h>
#include <pcl/segmentation/extract_clusters.h>
#include <vector>
#include <visualization_msgs/MarkerArray.h>

airsim_ros::Barrier Obj3D2Barrier(const Eigen::Vector4f &min_pt, const Eigen::Vector4f &max_pt)
{
    airsim_ros::Barrier barrier;

    barrier.minx = min_pt(0);
    barrier.miny = min_pt(1);
    barrier.minz = min_pt(2);
    barrier.maxx = max_pt(0);
    barrier.maxy = max_pt(1);
    barrier.maxz = max_pt(2);
    // ROS_INFO("%3f %3f %3f %3f %3f %3f", min_pt(0), min_pt(1), min_pt(2)
    //     , max_pt(0), max_pt(1), max_pt(2));
    return barrier;
}

airsim_ros::Barriers Barrier2Barriers(const std::vector<airsim_ros::Barrier> &barriers, const std_msgs::Header &header)
{
    airsim_ros::Barriers bs;
    bs.header = header;
    bs.barriers = barriers;
    return bs;
}

class LidarProbe
{
public:
    LidarProbe(const std::string &topic, std::string pcl_topic = "/rmua_dev/pcl")
    {
        sub_ = nh_.subscribe<sensor_msgs::PointCloud2>("airsim_node/drone_1/lidar", 1, std::bind(&LidarProbe::lidar_cb, this, std::placeholders::_1));
        pub_pcl_ = nh_.advertise<sensor_msgs::PointCloud2>(pcl_topic, 1);
        pub_barrier_ = nh_.advertise<airsim_ros::Barriers>(topic, 1);
        pub_barrier_marker_ = nh_.advertise<visualization_msgs::MarkerArray>("/barriers_marker", 1);
    }

private:
    ros::NodeHandle nh_;
    ros::Subscriber sub_;
    ros::Publisher pub_pcl_;
    ros::Publisher pub_barrier_;
    ros::Publisher pub_barrier_marker_;

    void lidar_cb(const sensor_msgs::PointCloud2::ConstPtr &msg)
    {
        ROS_INFO("Get lidar data. time: %f", msg->header.stamp.sec + msg->header.stamp.nsec * 1e-9);
        
        pcl::PointCloud<pcl::PointXYZ>::Ptr cloud_ptr;
        pcl::PointCloud<pcl::PointXYZ> cloud;
        pcl::fromROSMsg(*msg, cloud);
        cloud_ptr = cloud.makeShared();

        // pcl::search::KdTree<pcl::PointXYZ>::Ptr tree(new pcl::search::KdTree<pcl::PointXYZ>);
        pcl::search::KdTree<pcl::PointXYZ>::Ptr tree (new pcl::search::KdTree<pcl::PointXYZ>);
        tree->setInputCloud(cloud_ptr);

        std::vector<pcl::PointIndices> cluster_indices;
        pcl::EuclideanClusterExtraction<pcl::PointXYZ> ec;
        ec.setClusterTolerance(1); // 设置临近搜索的搜索半径（搜索容差）为20cm
        ec.setMinClusterSize(1);    // 每个簇（集群）的最小大小
        ec.setMaxClusterSize(25000);  // 每个簇（集群）的最大大小
        ec.setSearchMethod(tree);     // 设置点云搜索算法
        ec.setInputCloud(cloud_ptr);   // 设置输入点云
        ec.extract(cluster_indices);        // 设置提取到的簇，将每个簇以索引的形式保存到cluster_indices;
        
        int cluster_id = 0;
        std::vector<pcl::PCLPointCloud2::Ptr> cluster_clouds;
        std::vector<airsim_ros::Barrier> barriers;
        std::cout << "-----Frame-----" << std::endl;
        for (const auto& cluster : cluster_indices)
        {   
            std::cout << "Cluster " << std::endl;
            for (const auto& idx : cluster.indices)
            {
                std::cout << cloud_ptr->points[idx].x << " " << cloud_ptr->points[idx].y << " " << cloud_ptr->points[idx].z << ' ';
            }
            std::cout << "\n-------" <<std::endl;
            // 创建一个新的 pcl::PCLPointCloud2 对象
            pcl::PCLPointCloud2::Ptr cloud_cluster(new pcl::PCLPointCloud2);
        
            // 设置点云的一些基本参数
            cloud_cluster->width = cluster.indices.size();
            cloud_cluster->height = 1;  // 点云的高度为 1，表示它是一个点集
            cloud_cluster->is_dense = true;  // 假设点云是密集的
        
            // 现在定义一个字段来表示点数据，类似于 pcl::PointXYZ
            pcl::PCLPointField field;
            field.name = "x";  // 设置字段名称
            field.offset = 0;  // 坐标的偏移量
            field.datatype = pcl::PCLPointField::FLOAT32;  // 数据类型是 float32
            field.count = 1;  // 每个点的 x 坐标字段有一个值
        
            cloud_cluster->fields.push_back(field);
        
            field.name = "y";
            field.offset = sizeof(float);  // y 坐标的偏移量
            cloud_cluster->fields.push_back(field);
        
            field.name = "z";
            field.offset = 2 * sizeof(float);  // z 坐标的偏移量
            cloud_cluster->fields.push_back(field);
        
            // 为点云分配内存
            cloud_cluster->data.resize(cloud_cluster->width * cloud_cluster->height * cloud_cluster->fields.size() * sizeof(float));
        
            // 现在将每个点的数据填充到 pcl::PCLPointCloud2 中
            for (size_t i = 0; i < cluster.indices.size(); ++i)
            {
                const auto& point = cloud_ptr->points[cluster.indices[i]];
                int offset = i * cloud_cluster->fields.size() * sizeof(float);
        
                // 填充 x, y, z 坐标
                memcpy(&cloud_cluster->data[offset], &point.x, sizeof(float));
                memcpy(&cloud_cluster->data[offset + sizeof(float)], &point.y, sizeof(float));
                memcpy(&cloud_cluster->data[offset + 2 * sizeof(float)], &point.z, sizeof(float));
            }
            Eigen::Vector4f min_point, max_point;
            pcl::getMinMax3D(cloud_cluster, 0, 1, 2, min_point, max_point); // 获取 min 和 max 坐标
            std::cout << min_point << max_point << std::endl;
            barriers.push_back(Obj3D2Barrier(min_point, max_point));
            // 将每个聚类的 pcl::PCLPointCloud2 对象添加到 vector 中
            cluster_clouds.push_back(cloud_cluster);
        
            cluster_id++;
        }
        airsim_ros::Barriers barriers_msg = Barrier2Barriers(barriers, msg->header);
        pub_barrier_.publish(barriers_msg);

        visualization_msgs::MarkerArray marker_array;
        int marker_id = 0;
        for (const auto& barrier : barriers)
        {
            visualization_msgs::Marker marker;
            marker.header = msg->header;
            marker.ns = "barriers";
            marker.id = marker_id++;
            marker.type = visualization_msgs::Marker::CUBE;
            marker.action = visualization_msgs::Marker::ADD;

            // Set the position and size of the marker (barrier's min and max points)
            marker.pose.position.x = (barrier.minx + barrier.maxx) / 2.0;
            marker.pose.position.y = (barrier.miny + barrier.maxy) / 2.0;
            marker.pose.position.z = (barrier.minz + barrier.maxz) / 2.0;

            marker.scale.x = barrier.maxx - barrier.minx;
            marker.scale.y = barrier.maxy - barrier.miny;
            marker.scale.z = barrier.maxz - barrier.minz;

            marker.color.r = 1.0f;
            marker.color.g = 0.0f;
            marker.color.b = 0.0f;
            marker.color.a = 0.5f;  // semi-transparent red

            marker_array.markers.push_back(marker);
        }

    // Publish the marker array to RViz
    pub_barrier_marker_.publish(marker_array);


    }

    void publish(const pcl::PointCloud<pcl::PointXYZ> &cloud)
    {
        sensor_msgs::PointCloud2 msg;
        pcl::toROSMsg(cloud, msg);
        pub_pcl_.publish(msg);
    }
    void publish(const pcl::PointCloud<pcl::PointXYZ>::Ptr &cloud_ptr)
    {
        publish(*cloud_ptr);
    }


};
