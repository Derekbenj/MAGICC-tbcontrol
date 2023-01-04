import rospy
from keys.msg import LRM

def callback(data):
    print("Light Reading State: ", data.current_st)
    print("Light Reading Receive: ", data.light_reading)
    print("====================================")

def listener():
    rospy.init_node('python_readings_receiver', anonymous=True)
    rospy.Subscriber("light_readings", LRM, callback)
    rospy.spin()

if __name__ == "__main__":
    listener()