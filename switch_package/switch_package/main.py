import rclpy 
from rclpy.node import Node
from std_msgs.msg import UInt8MultiArray
from std_msgs.msg import Bool
from switch_package.receiver.sbus_decoder import SBUSDecoder
import serial

class sbus_decode(Node):
    def __init__(self):
        super().__init__('translate_sbus')
        self.sbus = SBUSDecoder()
        self.sub_bytes = self.create_subscription(UInt8MultiArray, "/translator/bytes", self.byte_callback, 10)
        self.pub_bool = self.create_publisher(Bool, "/switch/option", 10)

    def byte_callback(self, frame):
        if len(frame.data) != 0:
            data_=False
            channels = self.sbus.decode_frame(bytearray(frame.data))
            if channels[7] == 1810:
                data_=True
            self.pub_bool.publish(Bool(data=data_))
            self.get_logger().info(str(data_))
            self.get_logger().info(str(channels))
                
def main(args=None):
    rclpy.init(args=args)
    sbus_decode_node = sbus_decode()
    rclpy.spin(sbus_decode_node)
    rclpy.shutdown()


if __name__ == '__main__':
    main()
