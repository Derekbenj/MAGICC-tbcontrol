import rospy
import math
import numpy as np
from geometry_msgs.msg import PoseStamped
from scipy.spatial.transform import Rotation as R

def euler_from_quaternion(x, y, z, w):
        """
        Convert a quaternion into euler angles (roll, pitch, yaw)
        roll is rotation around x in radians (counterclockwise)
        pitch is rotation around y in radians (counterclockwise)
        yaw is rotation around z in radians (counterclockwise)
        """
        t0 = +2.0 * (w * x + y * z)
        t1 = +1.0 - 2.0 * (x * x + y * y)
        roll_x = math.atan2(t0, t1)
     
        t2 = +2.0 * (w * y - z * x)
        t2 = +1.0 if t2 > +1.0 else t2
        t2 = -1.0 if t2 < -1.0 else t2
        pitch_y = math.asin(t2)
     
        t3 = +2.0 * (w * z + x * y)
        t4 = +1.0 - 2.0 * (y * y + z * z)
        yaw_z = math.atan2(t3, t4)
     
        return roll_x, pitch_y, yaw_z # in radians

def callback(data):

    print("Incoming Data:")
    print("\tposition relative to east:       ", data.pose.position.x)
    print("\tposition relative to north:      ", data.pose.position.y)
    
    # calculate orientation:
    x = data.pose.orientation.x
    y = data.pose.orientation.y
    z = data.pose.orientation.z
    w = data.pose.orientation.w
    print("\torientation (degrees) from east: ", euler_from_quaternion(x, y, z, w)[2]*180/math.pi, end='\n\n')

def listener():
    rospy.init_node('optitrack_msg_receiver', anonymous=True)
    rospy.Subscriber('raph_enu', PoseStamped, callback)
    rospy.spin()

if __name__ == "__main__":
    listener()