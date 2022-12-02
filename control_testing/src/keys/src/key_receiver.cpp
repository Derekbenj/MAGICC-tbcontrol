#include "ros/ros.h"
#include "keys/Key.h"

void keyCallback(const keys::Key::ConstPtr &key);

int main(int argc, char **argv)
{
    ros::init(argc, argv, "key_receiver");
    ros::NodeHandle n;
    ros::Subscriber sub = n.subscribe("keys_heard", 1000, keyCallback);
    ros::spin();
    return 0;
}

void keyCallback(const keys::Key::ConstPtr &key)
{
    switch (key->keypress)
    {
    case 1:
        ROS_INFO("I heard key [up]");
        break;
    case 2:
        ROS_INFO("I heard key [down]");
        break;
    case 3:
        ROS_INFO("I heard key [left]");
        break;
    case 4:
        ROS_INFO("I heard key [right]");
        break;
    default:
        ROS_INFO("something's wrong...");
        break;
    }
}