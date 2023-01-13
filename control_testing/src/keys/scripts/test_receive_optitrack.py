import rospy
import numpy as np
from geometry_msgs.msg import PoseStamped
from scipy.spatial.transform import Rotation as R

def callback(data):

    print("Incoming Data:")
    print("\tposition relative to east:       ", data.pose.position.x)
    print("\tposition relative to north:      ", data.pose.position.y)
    
    # calculate orientation:
    x = data.pose.orientation.x
    y = data.pose.orientation.y
    z = data.pose.orientation.z
    w = data.pose.orientation.w
    r = R.from_quat([x, y, z, w])
    theta = -r.as_euler('zyx', degrees=True)[1]
    print("\torientation (degrees) from east: ", theta, end='\n\n')

def listener():
    rospy.init_node('optitrack_msg_receiver', anonymous=True)
    rospy.Subscriber('raph_enu', PoseStamped, callback)
    rospy.spin()

if __name__ == "__main__":
    listener()