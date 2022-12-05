#include "ros/ros.h"
#include "geometry_msgs/Twist.h"

void keyCallback(const geometry_msgs::Twist::ConstPtr& msg);

int main(int argc, char **argv)
{
    ros::init(argc, argv, "key_receiver");
    ros::NodeHandle n;
    ros::Subscriber sub = n.subscribe("turtle1/cmd_vel", 1000, keyCallback);
    ros::spin();
    return 0;
}

void keyCallback(const geometry_msgs::Twist::ConstPtr& msg)
{
    ROS_INFO("velocity: %f", msg->linear.x);
    ROS_INFO("turn angle: %f", msg->angular.z);
}