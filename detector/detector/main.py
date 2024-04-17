import argparse
import os
import sys
from pathlib import Path
import traceback

import cv2
import numpy as np
from detector.rknn import yolov8

import rclpy
from rclpy.node import Node
from sensor_msgs.msg import Image
from cv_bridge import CvBridge



class detector_ros(Node):
    def __init__(self):
        super().__init__('yolo_detector')

        self.bridge = CvBridge()
        self.sub_image = self.create_subscription(Image, 'image_raw', self.image_callback, 10)
        self.pub_image = self.create_publisher(Image, '/yolo_detector/image_raw', 10)
        # parameter
        FILE = Path(__file__).resolve()
        ROOT = FILE.parents[0]
        if str(ROOT) not in sys.path:
            sys.path.append(str(ROOT))  # add ROOT to PATH
        ROOT = Path(os.path.relpath(ROOT, Path.cwd()))
        self.model_path = str(ROOT) + '/rknn/new/yolov8n_1280_736_b16_e500_relu_rockchip_rknnopt_cut_RK3588_i8.rknn'
        self.yolo = yolov8(self.model_path)

        # self.video_writer = cv2.VideoWriter('/home/orangepi/ros2_ws/src/YOLOv5-ROS/yolov5_ros/yolov5_ros/video.mp4', -1, 1, (640, 640))

    def image_callback(self, image:Image):
        image_raw = self.bridge.imgmsg_to_cv2(image, "bgr8")
 
        classes, scores, boxes = self.yolo.run(image_raw)
        try:
            if boxes is None:
                imshow = image_raw
            else:
                imshow = self.yolo.draw(image_raw, boxes, scores, classes)

            imshow_raw = self.bridge.cv2_to_imgmsg(imshow, "bgr8")
            self.pub_image.publish(imshow_raw)
            #cv2.imshow('test', imshow)
            #cv2.waitKey(1)


        except Exception:
            self.get_logger().error(traceback.format_exc())
            # self.video_writer.release()


    # def __del__(self):
    #     self.video_writer.release()

def ros_main(args=None):
    rclpy.init(args=args)
    detector_node = detector_ros()
    rclpy.spin(detector_node)
    detector_node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    ros_main()
