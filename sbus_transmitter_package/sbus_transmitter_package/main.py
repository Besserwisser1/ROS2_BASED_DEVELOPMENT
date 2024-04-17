import rclpy
from rclpy.node import Node
from std_msgs.msg import Bool, UInt8MultiArray
import serial
import logging
import time
from pathlib import Path
home = str(Path.home())

class TransmitterNode(Node):
    def __init__(self):
        super().__init__('com_node')
        self.com_port = serial.Serial('/dev/ttyS0', 100000, timeout=0, parity=serial.PARITY_EVEN, stopbits=serial.STOPBITS_TWO, bytesize=serial.EIGHTBITS)
        strtime = str(time.strftime('%d.%m.%Y_%H.%M.%S', time.localtime()))

        self.subscription = self.create_subscription(UInt8MultiArray, '/translator/bytes', self.sbus_callback, 10)
        self.subscription_sbus2 = self.create_subscription(UInt8MultiArray, 'sbus_commands', self.sbus2_callback, 10)
        self.subscription_bool = self.create_subscription(Bool, '/switch/option', self.bool_callback, 10)

        self.use_first_source = True

        #logging.basicConfig(level=logging.INFO, filename= home+'/'+'SBUS_LOG_'+strtime+'.log' ,filemode="w",
        #    format="%(asctime)s %(levelname)s %(message)s")

    def sbus_callback(self, msg):
        #self.get_logger().info("1st source callback")
        if self.use_first_source:
            #logging.info("use 1st source")
            self.get_logger().info("use 1st source")
            self.send_to_com(msg.data)

    def sbus2_callback(self, msg):
        if not self.use_first_source:
            #logging.info("use 2nd source")
            self.get_logger().info("use 2nd source")
            self.send_to_com(msg.data)

    def bool_callback(self, msg):
        #logging.info("switched sources")
        self.use_first_source = not msg.data

    def send_to_com(self, data):
        self.get_logger().info(str(data))
        if self.com_port.is_open:
            byte_data = bytes(data)
            self.com_port.write(byte_data)
            self.get_logger().info(str(byte_data))
            #logging.info(str(data))

def main(args=None):
    rclpy.init(args=args)
    node = TransmitterNode()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()
