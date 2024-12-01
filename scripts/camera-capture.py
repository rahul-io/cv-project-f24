#!/usr/bin/env python3

from cv_bridge import CvBridge
from datetime import datetime
from picamera.array import PiRGBArray
from sensor_msgs.msg import Image as ROSImage

import argparse
import cv2
import numpy as np
import os
import picamera
import rospy
import time

def capture_image(camera):
    with PiRGBArray(camera) as output:
        camera.capture(output, format="bgr")
        return output.array

def capture_single(camera, save_dir):
    image_array = capture_image(camera)
    filename = datetime.now().strftime("%Y%m%d_%H%M%S") + ".jpg"
    frame_path = os.path.join(save_dir, filename)
    cv2.imwrite(frame_path, image_array)
    print(f"Saved single frame to {frame_path}")

def capture_continuous(camera):
    topic_name = '/camera/image_raw'
    rospy.init_node('camera_publisher', anonymous=True)
    pub = rospy.Publisher(topic_name, ROSImage, queue_size=10)
    bridge = CvBridge()
    rate = rospy.Rate(1)  # 1 Hz

    print(f"Publishing frames to {topic_name}. Press Ctrl+C to stop.")

    try:
        while not rospy.is_shutdown():
            image_array = capture_image(camera)
            ros_image = bridge.cv2_to_imgmsg(image_array, encoding="bgr8")
            pub.publish(ros_image)
            rate.sleep()
    except rospy.ROSInterruptException:
        print("\nStopped publishing.")

def main():
    parser = argparse.ArgumentParser(description="Capture and publish images from the PiCamera.")
    parser.add_argument("-c", "--continuous", action='store_true', help="Capture and publish continuous images.")
    parser.add_argument("-s", "--single", action='store_true', help="Capture and save a single image.")
    args = parser.parse_args()

    camera = picamera.PiCamera()
    camera.resolution = (640, 360)  # Set desired resolution
    time.sleep(2)  # Allow the camera to warm up

    if args.single:
        capture_single(camera, os.getcwd())
    elif args.continuous:
        capture_continuous(camera)
    else:
        print("Please specify --single or --continuous mode.")

    camera.close()

if __name__ == "__main__":
    main()
