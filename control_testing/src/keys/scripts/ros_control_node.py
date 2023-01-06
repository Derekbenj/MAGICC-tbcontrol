import math
import rospy
from turtlesim.msg import Pose
from geometry_msgs.msg import Twist
from turtlesim.srv import Kill
from turtlesim.srv import TeleportAbsolute, TeleportRelative, Spawn
from std_srvs.srv import Empty

class ros_control_node():
    '''
    Class for interfacing with ROS nodes.
    Publishes to: cmd_vel
    Subscribes to: pose
    Services Called: teleport_absolute, teleport_relative
    '''

    def __init__(self):
        # initialize ros publisher and subscriber
        rospy.init_node("control_alg_node", anonymous=True)
        self.pose = Pose()
        self.twist = Twist()

        rospy.wait_for_service("/spawn")
        rospy.wait_for_service("/kill")

        self.teleport_absolute = rospy.ServiceProxy("/my_turtle/teleport_absolute", TeleportAbsolute)
        self.teleport_relative = rospy.ServiceProxy("/my_turtle/teleport_relative", TeleportRelative)
        self.spawn_turtle = rospy.ServiceProxy('/spawn', Spawn)
        self.clear_drawings = rospy.ServiceProxy('/clear', Empty)
        self.kill = rospy.ServiceProxy('/kill', Kill)

        print("attempting to kill a turtle")
        try:
            self.kill("turtle1")
        except rospy.service.ServiceException:
            print("can't kill a dead turtle...")

        self.cmd_vel_pub = rospy.Publisher('/my_turtle/cmd_vel', Twist, queue_size=10)
        self.pose_sub = rospy.Subscriber('my_turtle/pose', Pose, self.ros_pose_callback)


    def ros_pose_callback(self, data):
        self.pose = data

    def get_current_state(self):
        return [self.pose.x, self.pose.y, self.pose.theta]

    def publish_cmd_vel(self):
        self.cmd_vel_pub.publish(self.twist)

    # def generate_twist_msg(self, input):
        # self.twist.linear.x = input[1]*math.cos(self.pose.theta) / 100
        # self.twist.linear.y = input[1]*math.sin(self.pose.theta) / 100
        # self.twist.linear.z = 0

        # self.twist.angular.x = 0
        # self.twist.angular.y = 0
        # self.twist.angular.z = input[0] / 10# check this... angular data probs radians when read from controller.
        # ^^ is the angular velocity being created by the controller in the same form as the turtlebot requires? I think turtlebot agrees with the state...
        # print('geometry msg: ', [self.twist.linear.x, self.twist.linear.y, self.twist.linear.z], [self.twist.angular.x, self.twist.angular.y, self.twist.angular.z])

    def generate_twist_msg(self, components_dot):
        self.twist.linear.x = components_dot[0]
        self.twist.linear.y = components_dot[1]
        self.twist.linear.z = 0

        self.twist.angular.x = 0
        self.twist.angular.y = 0
        self.twist.angular.z = components_dot[2] # check this... angular data probs radians when read from controller.
        # ^^ is the angular velocity being created by the controller in the same form as the turtlebot requires? I think turtlebot agrees with the state...
        print('geometry msg: ', [self.twist.linear.x, self.twist.linear.y, self.twist.linear.z], [self.twist.angular.x, self.twist.angular.y, self.twist.angular.z])