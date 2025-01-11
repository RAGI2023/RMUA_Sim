#include "ros/console.h"
#include "ros/node_handle.h"
#include "ros/publisher.h"
#include "ros/subscriber.h"
#include "geometry_msgs/Twist.h"
#include "airsim_ros/VelCmd.h"

class KeyboardCtrl
{
public:
    KeyboardCtrl()
    {
        puber = nh.advertise<airsim_ros::VelCmd>("airsim_node/drone_1/vel_cmd_body_frame", 1);
        suber = nh.subscribe("/cmd_vel", 1, &KeyboardCtrl::publish_vel, this);
        ROS_INFO("KeyboardCtrl init");
    }
    ~KeyboardCtrl() {};
    
private:
    ros::NodeHandle nh;
    ros::Subscriber suber;
    ros::Publisher puber;
    airsim_ros::VelCmd velcmd;

    void publish_vel(const geometry_msgs::Twist &twist)
    {
        // ROS_INFO("received velcmd: %f, %f, %f, %f, %f, %f", 
        //     twist.angular.x, twist.angular.y, twist.angular.z, twist.linear.x, twist.linear.y, twist.linear.z);
        
        velcmd.twist.angular.x = twist.angular.x;
        velcmd.twist.angular.y = twist.angular.y;
        velcmd.twist.angular.z = twist.angular.z;
        velcmd.twist.linear.x = twist.linear.x;
        velcmd.twist.linear.y = twist.linear.y;
        velcmd.twist.linear.z = twist.linear.z;
        
        ROS_INFO("publishing velcmd: %.2f, %.2f, %.2f, %.2f, %.2f, %.2f\n", 
            velcmd.twist.angular.x, velcmd.twist.angular.y, velcmd.twist.angular.z, velcmd.twist.linear.x, velcmd.twist.linear.y, velcmd.twist.linear.z);

        puber.publish(velcmd);
    }
};

int main(int argc, char **argv)
{
    ros::init(argc, argv, "keyboard_ctrl");
    KeyboardCtrl key_ctrl;
    ros::spin();
    return 0;
}