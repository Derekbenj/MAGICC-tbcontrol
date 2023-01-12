import rospy
from geometry_msgs.msg import Twist

twist_msg = Twist()

def test_robot():
    rospy.init_node("test_control_python", anonymous=True)
    pub = rospy.Publisher('/mobile_base/commands/velocity', Twist, queue_size=10)
    
    while not rospy.is_shutdown():
        num = int(input("options:\n\tForward: 1\n\tBackward: 2\n\tLeft: 3\n\tRight: 4\n\tQuit: 5\nChoice: "))
        print('\n')
        if num == 1:
            twist_msg.linear.x = 1
            twist_msg.angular.z = 0
            pub.publish(twist_msg)
        elif num == 2:
            twist_msg.linear.x = -1
            twist_msg.angular.z = 0
            pub.publish(twist_msg)
        elif num == 3:
            twist_msg.linear.x = 0
            twist_msg.angular.z = 1
            pub.publish(twist_msg)
        elif num == 4:
            twist_msg.linear.x = 0
            twist_msg.angular.z = -1
            pub.publish(twist_msg)
        elif num == 5:
            raise rospy.ROSInterruptException
        else:
            print("NOT A STATE...")
            continue

if __name__ == "__main__":
    try:
        test_robot()
    except rospy.ROSInterruptException:
        pass