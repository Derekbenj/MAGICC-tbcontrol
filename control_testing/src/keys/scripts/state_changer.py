import rospy
from std_msgs.msg import Int8

def change_st():
    pub = rospy.Publisher('change_st', Int8, queue_size=10)
    rospy.init_node("python_state_changer", anonymous=True)
    
    while not rospy.is_shutdown():
        num = int(input("What State Should I Enter (5 to exit): "))
        if num == 1:
            pub.publish(1)
        elif num == 2:
            pub.publish(2)
        elif num == 3:
            pub.publish(3)
        elif num == 4:
            pub.publish(4)
        elif num == 5:
            raise rospy.ROSInterruptException
        else:
            print("NOT A STATE...")
            continue

if __name__ == "__main__":
    try:
        change_st()
    except rospy.ROSInterruptException:
        pass