import rospy
from sensor_msgs.msg import Image
from cv_bridge import CvBridge
import cv2
import os

def image_publisher():
    rospy.init_node('image_publisher', anonymous=True)
    pub = rospy.Publisher('/left/image_raw', Image, queue_size=10)
    rate = rospy.Rate(5)  # Publish at 10 Hz
    bridge = CvBridge()

    # Path to your image folder
    image_folder = "/home/rahul/sources/map1"
    images = sorted([f for f in os.listdir(image_folder) if f.endswith(('.jpg', '.png'))])

    for img_name in images:
        img_path = os.path.join(image_folder, img_name)
        cv_image = cv2.imread(img_path)
        if cv_image is None:
            continue

        # Convert OpenCV image to ROS Image message
        ros_image = bridge.cv2_to_imgmsg(cv_image, encoding="bgr8")
        ros_image.header.stamp = rospy.Time.now()

        pub.publish(ros_image)
        rospy.loginfo(f"Published {img_name}")
        rate.sleep()

    rospy.loginfo("All images published.")

if __name__ == "__main__":
    try:
        image_publisher()
    except rospy.ROSInterruptException:
        pass