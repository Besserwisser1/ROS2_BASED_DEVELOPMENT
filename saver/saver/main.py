import cv2
import rclpy 
from rclpy.node import Node
from sensor_msgs.msg import Image
from std_msgs.msg import Float32MultiArray
from std_msgs.msg import String
from cv_bridge import CvBridge

from pathlib import Path

import time
import message_filters
import json

home = str(Path.home())

class saver(Node):
    def __init__(self):
        super().__init__('saver')
        self.time_local = time.localtime()
        self.strtime = str(time.strftime('%d.%m.%Y_%H.%M.%S', self.time_local))
        self.frameSize = (1280,720)
        self.bridge = CvBridge()
        self.sub_image = self.create_subscription(Image, '/yolo_detector/image_raw', self.image_callback,10)   
        self.fourcc = cv2.VideoWriter_fourcc('M', 'J', 'P', 'G')
        self.videoWriter = cv2.VideoWriter(home+'/'+'video_'+self.strtime+'.avi', self.fourcc, 20.0, self.frameSize)


    def image_callback(self, image:Image):
        image_raw = self.bridge.imgmsg_to_cv2(image, "bgr8")
        self.videoWriter.write(image_raw)
        cv2.imshow('video', image_raw)
        cv2.waitKey(1)


def main(args=None):
    rclpy.init(args=args)
    saver_node = saver()
    rclpy.spin(saver_node)
    saver_node.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()

