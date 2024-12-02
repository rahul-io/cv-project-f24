from cv_bridge import CvBridge, CvBridgeError
from sensor_msgs.msg import Image as ROSImage
from datetime import datetime
import rospy
import cv2
import torch
from models.experimental import attempt_load
from utils.general import check_img_size, non_max_suppression, scale_coords
from utils.plots import plot_one_box
from utils.torch_utils import select_device

class ROSYOLODetector:
    def __init__(self, model_path, device_type, img_size):
        # ROS-related
        self.bridge = CvBridge()
        self.image_sub = rospy.Subscriber("/camera/image_raw", ROSImage, self.receive_frames)
        
        # YOLO model-related
        self.device = select_device(device_type)
        self.model = attempt_load(model_path, map_location=self.device)  # Load model
        self.stride = int(self.model.stride.max())  # Model stride
        self.img_size = check_img_size(img_size, s=self.stride)  # Verify img_size
        self.names = self.model.module.names if hasattr(self.model, 'module') else self.model.names

    def detect(self, frame):
        # Preprocess frame
        img = cv2.resize(frame, (self.img_size, self.img_size))  # Resize to model size
        img = torch.from_numpy(img).to(self.device)
        img = img.permute(2, 0, 1).float()  # HWC to CHW
        img /= 255.0  # Normalize to 0-1
        if img.ndimension() == 3:
            img = img.unsqueeze(0)

        # Inference
        with torch.no_grad():
            pred = self.model(img)[0]
        pred = non_max_suppression(pred, 0.25, 0.45, agnostic=False)  # NMS

        # Process detections
        for det in pred:
            if len(det):
                det[:, :4] = scale_coords(img.shape[2:], det[:, :4], frame.shape).round()
                for *xyxy, conf, cls in reversed(det):
                    label = f'{self.names[int(cls)]} {conf:.2f}'
                    plot_one_box(xyxy, frame, label=label, color=(0, 255, 0), line_thickness=2)

        return frame

    def receive_frames(self, data):
        try:
            cv_image = self.bridge.imgmsg_to_cv2(data, "bgr8")
            result_image = self.detect(cv_image)  # Perform detection
            cv2.imshow("YOLO Detection", result_image)
            cv2.waitKey(1)  # Display the frame
        except CvBridgeError as e:
            rospy.logerr(f"CV Bridge Error: {e}")

def main():
    rospy.init_node('ros_yolo_detector', anonymous=True)
    model_path = 'yolov7.pt'  # Path to your YOLO model
    detector = ROSYOLODetector(model_path=model_path, device_type='cuda', img_size=640)
    print("YOLO Detector Initialized. Waiting for images...")
    try:
        rospy.spin()
    except KeyboardInterrupt:
        print("Shutting down YOLO Detector...")
        cv2.destroyAllWindows()

if __name__ == '__main__':
    main()


