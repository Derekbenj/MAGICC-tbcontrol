#include "ros/ros.h"
#include "keys/Key.h"

int main(int argc, char** argv)
{
    ros::init(argc, argv, "key_listener_sender");
    ros::NodeHandle n;

    ros::Publisher keys_pub = n.advertise<keys::Key>("keys_heard", 1000);
    ros::Rate loop_rate(10);

    keys::Key new_key;
    while(ros::ok())
    {
        new_key.keypress = getchar();
        if (new_key.keypress >= 0)
        {
            ROS_INFO("Saw Keypress: [%c]", new_key.keypress);
            keys_pub.publish(new_key);
        }
        else
        {
            ROS_INFO("No keypress detected...");
        }

        ros::spinOnce(); // not necesary here: added to help remind that if we ever needed a subscription this calls the callback
        loop_rate.sleep(); // sleep the remaining time necesary until we meet the desired frequency
    }
}