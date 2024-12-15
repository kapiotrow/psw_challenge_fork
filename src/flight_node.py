#!/usr/bin/env python3
import rospy
from std_msgs.msg import Bool
from geometry_msgs.msg import PoseStamped, Twist
from std_msgs.msg import Int32MultiArray


class Control(object):
    def __init__(self) -> None:
        #self.target_pose = None
        self.challenge_started = False
        self.attitude_above_2 = False
        self.Kp = 0.1
        self.pos_pub_topic = "/mavros/setpoint_position/local"
        self.pos_sub_topic = "/mavros/local_position/pose"
        self.velocity_sub_topic = "/mavros/local_position/velocity_local"
        self.vel_pub_topic = "/iris_control/cmd_vel"
        self.vel_pub = rospy.Publisher(self.vel_pub_topic, Twist, queue_size=10)
        self.pos_pub = rospy.Publisher(self.pos_pub_topic, PoseStamped, queue_size=10)
        self.rate = rospy.Rate(20)
        

    def velocity_cb(self, msg):
        dx = msg.data[0] - 50
        dy = msg.data[1] - 50
        pub_msg = Twist()
        pub_msg.linear.z = -self.Kp * dy
        pub_msg.linear.y = 0
        pub_msg.linear.x = 2.0
        pub_msg.angular.x = 0
        pub_msg.angular.y = 0
        pub_msg.angular.z = -0.005 * dx
        self.vel_pub.publish(pub_msg)


    def start_challenge_cb(self, msg):
        if msg.data:
            self.challenge_started = True


    def drone_control(self):
        rospy.Subscriber("/iris_control/challenge_start", Bool, self.start_challenge_cb)
        while(not self.challenge_started):
            self.rate.sleep()
        rospy.Subscriber("/iris_control/mosse_output", Int32MultiArray, self.velocity_cb)
        rospy.spin()



if __name__ == "__main__":
    rospy.init_node("flight_node")
    flight = Control()
    try:
        flight.drone_control()
    except rospy.ROSInterruptException:
        pass
    flight.rate.sleep()
    while(not rospy.is_shutdown):
        print("aaa")