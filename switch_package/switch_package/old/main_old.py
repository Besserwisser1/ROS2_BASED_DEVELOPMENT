import rclpy
from rclpy.node import Node
from std_msgs.msg import Bool
import os
import time

NUM = 14  # Номер пина для Orange Pi 4, измените на нужный

class PinStatePublisher(Node):
    def __init__(self):
        super().__init__('pin_state_publisher')
        self.publisher_ = self.create_publisher(Bool, 'pin_state', 10)
        
        self.timer = self.create_timer(0.5, self.timer_callback)

    def timer_callback(self):
        pin_state = False
        try:
            input_value = int(os.popen('gpio read 14').read())
            self.get_logger().info('Input_value: ' + str(input_value))
            if input_value == 1:
                pin_state = True
        except:
            pass
        finally:
            self.get_logger().info('pin_state: ' + str(pin_state))
        msg = Bool()
        msg.data = bool(pin_state)
        self.publisher_.publish(msg)
        self.get_logger().info(f'Publishing: {msg.data}')

def main(args=None):
    rclpy.init(args=args)
    pin_state_publisher = PinStatePublisher()
    rclpy.spin(pin_state_publisher)
    pin_state_publisher.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()
