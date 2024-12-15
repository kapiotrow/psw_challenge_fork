#!/usr/bin/env python3
import rospy
from std_msgs.msg import Bool
from geometry_msgs.msg import PoseStamped



class Control(object):
    def __init__(self) -> None:
        self.target_pose = None
        self.challenge_started = False
        self.pos_pub_topic = "/mavros/setpoint_position/local"
        self.pos_sub_topic = "/mavros/local_position/pose"
        self.pub = rospy.Publisher(self.pos_pub_topic, PoseStamped, queue_size=10)
        

    def pose_cb(self, msg: PoseStamped):
        self.target_pose = (10, 10, 2)
        print(msg)
        if msg.pose.position.z >= 2:
            pub_msg = PoseStamped()

            pub_msg.pose.position.x = self.target_pose[0]
            pub_msg.pose.position.y = self.target_pose[1]
            pub_msg.pose.position.z = self.target_pose[2]
            self.pub.publish(pub_msg)


    def start_challenge_cb(self, msg: Bool):
        if msg.data:
            self.challenge_started = True


    def drone_control(self):
        rospy.Subscriber("/iris_control/challenge_start", Bool, self.start_challenge_cb)
        if self.challenge_started:
            rospy.Subscriber(self.pos_sub_topic, PoseStamped, self.pose_cb)
        
        rospy.spin()


# if __name__ == '__main__':
#     try:
#         drone_control()
#     except rospy.ROSInterruptException:
#         pass