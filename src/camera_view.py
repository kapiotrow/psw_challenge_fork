#!/usr/bin/env python3
import rospy
from sensor_msgs.msg import Image
import cv2
import numpy as np
from mosse_tracker import Mosse
from std_msgs.msg import Bool
from geometry_msgs.msg import PoseStamped


class Nodo(object):
    def __init__(self):
        self.frame = None
        self.tracker = None
        self.challenge_started = False
        rospy.Subscriber('/iris/usb_cam/image_raw', Image, self.camera_sub_callback)
        rospy.Subscriber("/iris_control/challenge_start", Bool, self.start_challenge_cb)
        print("Aaa")
        

    def camera_sub_callback(self, msg: Image):
        img = np.array(list(msg.data)).astype(np.uint8)
        self.frame = np.reshape(img, (msg.height, msg.width, 3))
        frame_gray = cv2.cvtColor(self.frame, cv2.COLOR_BGR2GRAY)
        if self.tracker==None:
            self.tracker = Mosse(frame_gray.shape)
            bbox = (565, 285, 150, 150)
            self.tracker.init(frame_gray, bbox)
        else:
            output_bbox = self.tracker.update(frame_gray)
            x_min, y_min, w, h = output_bbox
            cv2.rectangle(self.frame, (x_min, y_min), (x_min + w, y_min + h), (255, 0, 0), 2, 1)
            cv2.imshow("tracked result", self.frame)
            cv2.waitKey(1)

    def start_challenge_cb(self, msg: Bool):
        if msg.data:
            self.challenge_started = True


    def camera_sub(self):
        rospy.spin()




# if __name__ == '__main__':
#     rospy.init_node("camera_test", anonymous=True)
#     camera_node = Nodo()
#     camera_node.camera_sub()

#     cv2.destroyAllWindows()
