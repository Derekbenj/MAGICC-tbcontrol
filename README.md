For controlling the turtlebot via ROS 1 (Noetic on my computer, Kinetic on the Turtlebots). The difference in ROS versions shouldn't matter since we're only passing
geometry_msg/Twist messages to the turtlebots currently

All that needs to be done to create a catkin workspace for further ROS development is to pull this code, and then move into the workspace titled "control_testing" and type "catkin_make". An extra folder exists alongside the catkin workspace folder only because all arduino code must be held in it's own folder. Because the arduino code relies on a custom message defined in our catkin project titled "keys", you must source the catkin workspace using "source /devel/setup.bash" prior to running the arduino code. If you forget to do this, you'll likely see an error from ROS telling you it's failing to communicate with the Arduino.

Current State:
    1) Arduino code works and can be uploaded for measurements. Doesn't measure lux, but scales so every arduino should be roughly equally sensitive to light.
    2) Code for experimenting with ROS interfacing written in C++ and Python (C++ code simply listens for simple keypresses and is unrelated to the controller project, while the python code interfaces with the arduino to change it's state in the state machine and listen for measurements).
    3) I haven't figured out how to write the arduino interface with ROS 2 yet...
    4) Controller code written for interfacing with the turtlebot node.

Useful Notes:
    1) Don't eat up too much of the dynamic memory in the Arduino code if you modify it. Any code using 75% or more starts warning me that the arduino may malfunction, and often it does.
    2) The control testing code should be run with the simulation.py module (which will open a ROS node) and will pause until the turtlesim_node is up and running if started first.

Beta Feature in the works: Connecting Optitrack Cameras and Connecting the Turtlebots
Include the following code in your .bashrc for a ros that's NOT the roscore (the backslash MATTERS)
export ROS_MASTER_URI=http://192.168.1.3:11311/
export ROS_HOSTNAME=192.168.1.6
export ROS_IP=192.168.1.6
Then, include the follwing in the .bashrc for a ros computer that IS the roscore (the backslash may not matter)
export ROS_MASTER_URI=http://localhost:11311/
export ROS_HOSTNAME=192.168.1.3
export ROS_IP=192.168.1.3

To start up the ROS node on the optitrack system you need to type exactly "rosrun optitrack_vrpn node _host:=192.168.1.3" and if you don't include the _host part it will complain indefinitely

On the ROS Master, I need to export the IP address so that other computers can not only see the topics but subscribe to the data too
When Brady wanted to ask the computer for information about it's public address he installed ldnsutils and then ran "drill optitrack" which was the hostname of the computer 
The node I'm going to want to subscribe to is a pose variable called /raph_ned or /raph_enu... these both use a geometry_msgs/PoseStamped message
*** The turtlebot simulator wants to read pose relative to the x and y horizon... so lets say east=x, north=y, and disregard "up".
Another point to mention: make sure you flip the switch to turn on the optitrack camera system before you attempt to open any nodes!!!!
