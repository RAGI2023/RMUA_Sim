#include "ros/init.h"
#include "ros/console.h"
#include "ros/node_handle.h"
#include <ctime>
#include <opencv2/core/mat.hpp>
#include <opencv2/core/persistence.hpp>
#include <opencv2/core/types.hpp>
#include <opencv2/highgui.hpp>
#include <opencv2/imgproc.hpp>
#include <string>
#include "ros/publisher.h"
#include "sensor_msgs/Image.h"
#include "airsim_ros/Scene.h"

class FramePacker
{
public:
    FramePacker(const std::string &left_topic, const std::string &right_topic, const std::string &scene_topic)
    {
        sub_left_ = nh_.subscribe<sensor_msgs::Image>(left_topic, 1, 
            std::bind(&FramePacker::callback_left, this, std::placeholders::_1));
        sub_right_ = nh_.subscribe<sensor_msgs::Image>(right_topic, 1, 
            std::bind(&FramePacker::callback_right, this, std::placeholders::_1));
        puber_ = nh_.advertise<airsim_ros::Scene>(scene_topic, 1);

    }
private:
    ros::NodeHandle nh_;
    ros::Publisher puber_;
    ros::Subscriber sub_left_, sub_right_;
    sensor_msgs::Image left_image_, right_image_;
    double left_time_, right_time_;

    bool right_ready_ = false;
    bool left_ready_ = false;

    bool publish_scene()
    {
        double time_max_diff = 1e-4;

        if (fabs(left_time_ - right_time_) > time_max_diff)
        {
            ROS_WARN("Time difference between left and right image is too large. Skip this process. %lf %lf", right_time_, left_time_);
            return false;
        }
        airsim_ros::Scene scene;
        scene.left = left_image_;
        scene.right = right_image_;
        puber_.publish(scene);
        ROS_INFO("Publish Scene. %lf, %lf", left_time_, right_time_);
        return true;
    }

    void callback_left(const sensor_msgs::ImageConstPtr &msg)
    {
        left_image_ = *msg;
        left_time_ = msg->header.stamp.toSec() + msg->header.stamp.toNSec() * 1e-9;
        ROS_INFO("Get left image.: %f", left_time_);
        left_ready_ = true;
        if (right_ready_){
            if (publish_scene()){
                right_ready_ = false;
                left_ready_ = false;
            }
        }
    }

    void callback_right(const sensor_msgs::ImageConstPtr &msg)
    {
        right_image_ = *msg;
        right_time_ = msg->header.stamp.toSec() + msg->header.stamp.toNSec() * 1e-9;
        ROS_INFO("Get right image.: %f", right_time_);
        right_ready_ = true;
        if (left_ready_){
            if (publish_scene()){
                right_ready_ = false;
                left_ready_ = false;
            }
        }
    }

};

int main(int argc, char **argv)
{
    ros::init(argc, argv, "scene_packer_node");
    FramePacker fp("/airsim_node/drone_1/front_left/Scene", "/airsim_node/drone_1/front_right/Scene", "airsim_node/drone_1/front_scene");
    ros::spin();
    return 0;

}