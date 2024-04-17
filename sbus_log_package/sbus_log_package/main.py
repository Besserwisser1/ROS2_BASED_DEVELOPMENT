import rclpy 
from rclpy.node import Node
from sensor_msgs.msg import Image
from std_msgs.msg import Float32MultiArray
from std_msgs.msg import String
from std_msgs.msg import Bool
from cv_bridge import CvBridge

from sbus_log.receiver.sbus_receiver import SBUSReceiver

from pathlib import Path

import time

import logging




home = str(Path.home())
strtime = str(time.strftime('%d.%m.%Y_%H.%M.%S', time.localtime()))



class sbus_log(Node):
    def __init__(self):
        super().__init__('sbus_log')
        self.sbus = SBUSReceiver('/dev/ttyS4')
        self.do_save = False
        self.timer_ = self.create_timer(0.005, self.sbus_log_callback)
        #logging.basicConfig(level=logging.INFO, filename= home+'/'+'SBUS_LOG_'+strtime+'.log' ,filemode="w",
        #            format="%(asctime)s %(levelname)s %(message)s")


    def sbus_log_callback(self):
        self.sbus.update()
        if self.sbus.sbusChannels[0] != 0:
            self.get_logger().info(str(self.sbus.get_rx_channels()))
            #logging.info(str(self.sbus.get_rx_channels()))



def main(args=None):
    rclpy.init(args=args)
    sbus_log_node = sbus_log()
    rclpy.spin(sbus_log_node)
    rclpy.shutdown()


if __name__ == '__main__':
    main()

