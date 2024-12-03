#!/usr/bin/env python3

import argparse
import time
from pathlib import Path

import cv2
import rospy
import torch
import torch.backends.cudnn as cudnn
from cv_bridge import CvBridge, CvBridgeError
from sensor_msgs.msg import Image as ROSImage
from numpy import random
from models.experimental import attempt_load
from utils.general import check_img_size, non_max_suppression, scale_coords, set_logging
from utils.plots import plot_one_box
from utils.torch_utils import select_device, TracedModel


class ROSYOLODetector:
    def __init__(self, weights, device, img_size, conf_thres, iou_thres, classes, agnostic_nms):
        # YOLO model-related
        self.device = select_device(device)
        self.model = attempt_load(weights, map_location=self.device)  # Load model
        self.stride = int(self.model.stride.max())  # Model stride
        self.img_size = check_img_size(img_size, s=self.stride)  # Verify img_size
        self.conf_thres = conf_thres
        self.iou_thres = iou_thres
        self.classes = classes
        self.agnostic_nms = agnostic_nms

        # Names and colors for visualization
        self.names = self.model.module.names if hasattr(self.model, 'module') else self.model.names
        self.colors = [[random.randint(0, 255) for _ in range(3)] for _ in self.names]

        # ROS-related
        self.bridge = CvBridge()
        self.image_sub = rospy.Subscriber("/left/image_raw", ROSImage, self.receive_frames)

    def receive_frames(self, data):
        try:
            # Convert ROS Image to OpenCV format
            cv_image = self.bridge.imgmsg_to_cv2(data, "bgr8")
            result_image = self.detect(cv_image)  # Perform detection
            cv2.imshow("YOLO Detection", result_image)
            cv2.waitKey(1)  # Refresh the display window
        except CvBridgeError as e:
            rospy.logerr(f"CV Bridge Error: {e}")

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
            pred = self.model(img, augment=False)[0]
        pred = non_max_suppression(pred, self.conf_thres, self.iou_thres, classes=self.classes, agnostic=self.agnostic_nms)

        # Process detections
        for det in pred:
            if len(det):
                det[:, :4] = scale_coords(img.shape[2:], det[:, :4], frame.shape).round()
                for *xyxy, conf, cls in reversed(det):
                    label = f'{self.names[int(cls)]} {conf:.2f}'
                    plot_one_box(xyxy, frame, label=label, color=self.colors[int(cls)], line_thickness=2)

        return frame


def main():
    # Parse arguments
    parser = argparse.ArgumentParser()
    parser.add_argument('--weights', type=str, required=True, help='Path to model weights')
    parser.add_argument('--device', default='', help='Device: cuda or cpu')
    parser.add_argument('--img-size', type=int, default=640, help='Inference size (pixels)')
    parser.add_argument('--conf-thres', type=float, default=0.25, help='Object confidence threshold')
    parser.add_argument('--iou-thres', type=float, default=0.45, help='IOU threshold for NMS')
    parser.add_argument('--classes', nargs='+', type=int, help='Filter by class')
    parser.add_argument('--agnostic-nms', action='store_true', help='Class-agnostic NMS')
    args = parser.parse_args()

    rospy.init_node('ros_yolo_detector', anonymous=True)
    detector = ROSYOLODetector(
        weights=args.weights,
        device=args.device,
        img_size=args.img_size,
        conf_thres=args.conf_thres,
        iou_thres=args.iou_thres,
        classes=args.classes,
        agnostic_nms=args.agnostic_nms
    )
    print("YOLO ROS Detector is running...")
    try:
        rospy.spin()
    except KeyboardInterrupt:
        print("Shutting down YOLO ROS Detector...")
        cv2.destroyAllWindows()


if __name__ == '__main__':
    main()
