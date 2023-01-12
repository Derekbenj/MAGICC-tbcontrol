import rospy
from geometry_msgs.msg import PoseStamped

def callback(data):
    print("Incoming Data:")
    print("\tposition relative to east:  ", data.pose.position.x)
    print("\tposition relative to north: ", data.pose.position.y, end='\n\n')

def listener():
    rospy.init_node('optitrack_msg_receiver', anonymous=True)
    rospy.Subscriber('raph_enu', PoseStamped, callback)
    rospy.spin()

if __name__ == "__main__":
    listener()