#include "ros/console.h"
#include "ros/init.h"
#include "ros/node_handle.h"
#include "ros/ros.h"
#include "ros/subscriber.h"
#include "sensor_msgs/PointCloud2.h"

void callback(const sensor_msgs::PointCloud2::ConstPtr& msg)
{
    ROS_INFO("Lidar call back");
    
}

int main(int argc, char **argv)
{
    ros::init(argc, argv, "occupancyGrid");
    ros::NodeHandle nh;
    ros::Subscriber sub = nh.subscribe("/airsim_node/drone_1/lidar", callback);

    ros::spin();

    return 0;

}