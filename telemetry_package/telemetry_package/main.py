import rclpy
from rclpy.node import Node
from pymavlink import mavutil
from geometry_msgs.msg import Quaternion

import ros2_numpy as rnp
import numpy as np

class OurNode(Node):

    def __init__(self):
        super().__init__('sender')
        self.arr = np.array([0.0,0.0,0.0,0.0])
        self.sender_ = self.create_publisher(Quaternion, '/quaternion_topic', 10)
        self.connection = mavutil.mavlink_connection('/dev/ttyS1', baud=115200, dialect='common')
        

        self.timer = self.create_timer(0.005, self.sender)
        
        

    def sender(self):
        self.param = self.connection.recv_match(type=['ATTITUDE', 'VFR_HUD'], blocking=True)
        if (self.param.get_type() == 'VFR_HUD' and  self.param.alt):
            self.arr[0] = self.param.alt
            self.get_logger().info('alt: '+str(self.param.alt))
        if (self.param.get_type() == 'ATTITUDE'):
            self.arr[1] = self.param.yaw
            self.arr[2] = self.param.pitch
            self.arr[3] = self.param.roll
            self.get_logger().info('yaw: '+ str(self.param.yaw) + ' pitch: ' + str(self.param.pitch) + ' roll: ' + str(self.param.roll))
        msg = rnp.msgify(Quaternion, self.arr)
        self.sender_.publish(msg)



def main(args=None):
    rclpy.init(args=args)
    node = OurNode()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()
