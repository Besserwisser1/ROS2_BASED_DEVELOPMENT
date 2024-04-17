import rclpy 
from rclpy.node import Node
from std_msgs.msg import UInt8MultiArray
import serial


class translate_sbus(Node):
    def __init__(self):
        super().__init__('translate_sbus')
        self.sbus_ser = serial.Serial(
			port='/dev/ttyS4',
			baudrate = 100000,
			parity=serial.PARITY_EVEN,
			stopbits=serial.STOPBITS_TWO,
			bytesize=serial.EIGHTBITS,
			timeout = 1,
		)

        self.if_serial_read = True
        self.timer_0 = self.create_timer(0.005, self.main_callback3)
        self.publish_to_switch = self.create_publisher(UInt8MultiArray, "/translator/bytes", 10)

    def main_callback3(self):
        while True:
            byte = self.sbus_ser.read(1)

            if byte == b'\x0F':
                break

        remaining_bytes = self.sbus_ser.read(24)

        full_packet = byte + remaining_bytes

        msg = UInt8MultiArray()
        msg.data = list(full_packet)
        self.publish_to_switch.publish(msg)
        self.get_logger().info(str(msg.data))
        
    def main_callback2(self):
        if self.if_serial_read:
            self.if_serial_read = False
            
            read_ser = self.sbus_ser.read(25)
            self.get_logger().info(str(len(read_ser)))
            list_ser = list(read_ser)
            msg = UInt8MultiArray()
            msg.data = list_ser
            self.get_logger().info(str(msg.data))
            self.publish_to_switch.publish(msg)
            
            self.if_serial_read = True
        else:
            self.get_logger().info("already reading from serial")

    def main_callback(self):
        read_ser = self.sbus_ser.read(25)
        self.get_logger().info(str(len(read_ser)))
        msg = UInt8MultiArray()
        msg.data = list(read_ser)
        self.get_logger().info(str(msg.data))
        # self.bytes_.clear()
        self.publish_to_switch.publish(msg)



    def main_callback1(self):
        read_ser = self.sbus_ser.read(25)
        self.get_logger().info(str(len(read_ser)))
        list_ser = list(read_ser)
        if len(list_ser) == 25 and list_ser[0] == 15:
            msg = UInt8MultiArray()
            msg.data = list(read_ser)
            self.get_logger().info(str(msg.data))
        # self.bytes_.clear()
            self.publish_to_switch.publish(msg)
        

def main(args=None):
    rclpy.init(args=args)
    translate_sbus_node = translate_sbus()
    rclpy.spin(translate_sbus_node)
    rclpy.shutdown()


if __name__ == '__main__':
    main()

