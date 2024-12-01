#!/usr/bin/env python3

from cv_bridge import CvBridge, CvBridgeError
from datetime import datetime
from sensor_msgs.msg import Image as ROSImage

import argparse
import cv2
import numpy as np
import os
import rospy
import time

class image_receiver:
    def __init__(self):
        self.bridge = CvBridge()
        self.image_sub = rospy.Subscriber("/camera/image_raw",ROSImage,self.receive_frames)

    def receive_frames(self, data):
        try:
            cv_image = self.bridge.imgmsg_to_cv2(data, "bgr8")
        except CvBridgeError as e:
            print(e)
    
        cv2.imshow("Image window", cv_image)
        cv2.waitKey(3)


def main():
    receiver = image_receiver()
    rospy.init_node('image_receiver', anonymous=True)
    print("Receiving images...")
    try:
        rospy.spin()
    except KeyboardInterrupt:
        print("shutting down")
        cv2.destroyAllWindows()

if __name__ == '__main__':
    main()