#include "ros/ros.h"
#include "keys/Key.h"

void keyCallback(const keys::Key::ConstPtr& key);

int main(int argc, char** argv)
{
    ros::init(argc, argv, "key_receiver");
    ros::NodeHandle n;
    ros::Subscriber sub = n.subscribe("keys_heard", 1000, keyCallback);
    ros::spin();
    return 0;
}

void keyCallback(const keys::Key::ConstPtr &key)
{
    ROS_INFO("I heard key [%c]", (char)key->keypress);
}