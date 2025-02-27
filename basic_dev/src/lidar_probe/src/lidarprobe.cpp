#include "lidar_probe/lidarprobe.hpp"
#include "ros/init.h"

int main(int argc, char **argv)
{
    ros::init(argc, argv, "lidar_probe");
    
    LidarProbe lp("/lidar_out");
    ros::spin();
    return 0;
}
