#!/usr/bin/env python3
import os
import argparse
from datetime import datetime
import picamera
from PIL import Image
import rospy
from sensor_msgs.msg import Image as ROSImage
from cv_bridge import CvBridge 


# Initialize the camera
picam = picamera()
picam.configure(picam.create_still_configuration())

# Initialize ROS components
rospy.init_node('camera_publisher', anonymous=True)
pub = rospy.Publisher('/camera/raw', ROSImage, queue_size=10)
bridge = CvBridge()


def capture_single(save_dir):
    # Capture an image
    image = picam.capture_array()
    image_pil = Image.fromarray(image)

    # Save the image with its own timestamped filename
    filename = datetime.now().strftime("%Y%m%d_%H%M%S") + ".jpg"  # Format: YYYYMMDD_HHMMSS.jpg
    frame_path = os.path.join(save_dir, filename)
    image_pil.save(frame_path)

def capture_continuous():
    """
    Continuously capture images and publish them to the /camera/raw topic.
    """
    rate = rospy.Rate(1)
    print("Publishing frames to /camera/raw. Press Ctrl+C to stop.")

    try:
        while not rospy.is_shutdown():
            image_array = picam.capture_array()
            ros_image = bridge.cv2_to_imgmsg(image_array, encoding="bgr8")
            pub.publish(ros_image)
            rate.sleep()
    except rospy.ROSInterruptException:
        print("\nStopped publishing.")
    finally:
        picam.stop()


def main():
    parser = argparse.ArgumentParser(description="Capture and publish images from the PiCamera.")
    parser.add_argument("-c", "--continuous", action='store_true', help="Capture and publish continuous images.")
    parser.add_argument("-s", "--single", action='store_true', help="Capture and publish a single image.")
    args = parser.parse_args()

    # Start the camera
    picam.start()

    if args.single:
        print("Capturing a single frame...")
        capture_single(os.getcwd())
        print(f"Saved single frame to {os.getcwd()}")
    elif args.continuous:
        capture_continuous()
    else:
        print("Please specify --single or --continuous mode.")

    picam.stop()


if __name__ == "__main__":
    main()
