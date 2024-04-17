import rclpy
from rclpy.node import Node
from geometry_msgs.msg import Quaternion
from std_msgs.msg import UInt8MultiArray
#from sensor_msgs.msg import Image
import math
import datetime
import transformations
import ros2_numpy as rnp
import numpy as np

class BB:
    '''Класс bounding box с центром объекта и длиной, шириной ограничивающей рамки'''
    def __init__(self, x, y, w, h, class_index, confidence):
        self.x = x
        self.y = y
        self.w = w
        self.h = h
        self.class_index = class_index
        self.confidence = confidence

    @staticmethod
    def from_x1x2y1y2_array(arr):
        return BB((arr[0][0]+arr[0][2])/2, (arr[0][1]+arr[0][3])/2, abs(arr[0][0]-arr[0][2]), abs(arr[0][1]-arr[0][3]), arr[1], arr[2])

    @staticmethod
    def from_x1x2y1y2_array_of_arrays(arr):
        res = []
        for inner_arr in arr:
            res.append(BB.from_x1x2y1y2_array(inner_arr))
        return res

class MavLink:
    '''Класс пакетов MavLink'''
    def __init__(self, roll, pitch, yaw, h):
        #print(f"Initializing MavLink: roll={roll}, pitch={pitch}, yaw={yaw}, h={h}")
        self.roll = roll
        self.pitch = pitch
        self.yaw = yaw
        self.h = h

    @staticmethod
    def from_array(arr):
        #print(f"Creating MavLink from array: {arr}")
        if len(arr) >=4:
            return MavLink(arr[0], arr[1], arr[2], arr[3])
        else:
            return MavLink(arr[0], arr[1], arr[2], 0)

class SBUS:
    def __init__(self):
        self.throttle = 0
        self.roll = 0
        self.pitch = 0
        self.yaw = 0
        self.sbus_packet = bytearray(25)

    def update_values(self, throttle, roll, pitch, yaw):
        #print(f"Updating SBUS values: throttle={throttle}, roll={roll}, pitch={pitch}, yaw={yaw}")
        self.throttle = int(np.clip(np.round(throttle), 173, 1810))
        self.roll = int(np.clip(np.round(roll), 173, 1810))
        self.pitch = int(np.clip(np.round(pitch), 173, 1810))
        self.yaw = int(np.clip(np.round(yaw), 173, 1810))

        self.form_packet()
        
    def form_packet(self):
        #print(f"Forming packet with: throttle={self.throttle}, roll={self.roll}, pitch={self.pitch}, yaw={self.yaw}")
        values = [self.roll, self.pitch, self.throttle, self.yaw] + [0] * 12
        self.sbus_packet = bytearray(25)
        self.sbus_packet[0] = 0x0F
        for i in range(16):
            value = values[i]
            byte_index = 1 + (i * 11 // 8)
            bit_index = (i * 11) % 8
            self.sbus_packet[byte_index] |= int(((value & 0x07FF) << bit_index) & 0xFF)
            self.sbus_packet[byte_index + 1] |= int(((value & 0x07FF) << bit_index) >> 8 & 0xFF)
            if bit_index >= 6 and i < 15:
                self.sbus_packet[byte_index + 2] |= int(((value & 0x07FF) << bit_index) >> 16 & 0xFF)
        self.sbus_packet[24] = 0x00
        return self.sbus_packet
        
class StaticAutopilot:
  
    """Первый параметр (flight_altitude) отвечает за высоту полета (в единицах измерения телеметрии. должен быть в метрах)
     SBUS_min, SBUS_mid, SBUS_max, max_angular_velocity для всех каналов одинаковый
     изменение throttle происходит равномерно с одним шагом
    """
    def __init__(self, zero_throttle = 700, h_error = 0.07, leveling_throttle = 1000, leveling_roll = 1, leveling_pitch = 1, SBUS_min = 173, SBUS_mid = 993, SBUS_max = 1810, max_angular_velocity = 360):
      self.zero_throttle = zero_throttle
      self.h_error = abs(h_error)
      self.SBUS_min = SBUS_min
      self.SBUS_mid = SBUS_mid
      self.SBUS_max = SBUS_max
      self.SBUS_to_max = self.SBUS_max - self.SBUS_mid
      self.SBUS_to_min = self.SBUS_mid - self.SBUS_min
      self.max_angular_velocity = max_angular_velocity
      self.first_iter = True
      self.leveling_throttle = leveling_throttle
      self.leveling_roll = leveling_roll
      self.leveling_pitch = leveling_pitch

    def to_SBUS_range(self, x):
      if x < self.SBUS_min:
        return self.SBUS_min
      if x > self.SBUS_max:
        return self.SBUS_max
      return x

    def fly(self, telemetry):
      print(f"Пришедшие данные на fly: h={telemetry.h}, roll={telemetry.roll}, pitch={telemetry.pitch}, yaw={telemetry.yaw}")
      
      if self.first_iter:
        throttle = self.zero_throttle
        self.first_iter = False
        self.flight_altitude = telemetry.h
        print(f"zero_throttle: {throttle}")
      else:
        if telemetry.h < self.flight_altitude + self.h_error and telemetry.h > self.flight_altitude - self.h_error:
          throttle = self.zero_throttle
        else:
          throttle = self.zero_throttle - (telemetry.h - self.flight_altitude)*self.leveling_throttle

      if telemetry.roll > 0:
        roll = self.SBUS_mid - telemetry.roll/self.max_angular_velocity*self.SBUS_to_min*self.leveling_roll/math.pi*180
      else:
        roll = self.SBUS_mid - telemetry.roll/self.max_angular_velocity*self.SBUS_to_max*self.leveling_roll/math.pi*180

      if telemetry.pitch > 0:
        pitch = self.SBUS_mid + telemetry.pitch/self.max_angular_velocity*self.SBUS_to_min*self.leveling_pitch/math.pi*180
      else:
        pitch = self.SBUS_mid + telemetry.pitch/self.max_angular_velocity*self.SBUS_to_max*self.leveling_pitch/math.pi*180
      yaw = self.SBUS_mid

      print(f"Данные для SBUS из fly: throttle={throttle}, roll={roll}, pitch={pitch}, yaw={yaw}")

      sbus = SBUS()
      sbus.update_values(throttle, roll, pitch, yaw)
      return sbus

class AutopilotNode(Node):
    def __init__(self):
        super().__init__('autopilot_node')
        self.sbus = SBUS()
        self.autopilot = StaticAutopilot()

        self.telemetry_subscription = self.create_subscription(Quaternion, '/quaternion_topic', self.telemetry_callback, 10)
        self.sbus_publisher = self.create_publisher(UInt8MultiArray, 'sbus_commands', 10)

    def telemetry_callback(self, msg):
        print(f"Полученные данные телеметрии: h={msg.x}, yaw={msg.y}, pitch={msg.z}, roll={msg.w}")

        h = msg.x
        yaw = msg.y
        pitch = msg.z
        roll = msg.w

        telemetry = MavLink(roll, pitch, yaw, h)

        #throttle, roll, pitch, yaw = self.autopilot.fly(telemetry)
        sbus_command = self.autopilot.fly(telemetry)

        #self.sbus.update_values(throttle, roll, pitch, yaw)
        self.sbus.update_values(sbus_command.throttle, sbus_command.roll, sbus_command.pitch, sbus_command.yaw)
        sbus_packet = self.sbus.form_packet()

        message = UInt8MultiArray()
        message.data = list(sbus_packet)
        self.sbus_publisher.publish(message)
        self.get_logger().info('Published SBUS packet: ' + ' '.join(format(byte, '08b') for byte in sbus_packet))
        #self.get_logger().info(f'Published SBUS packet: {list(sbus_packet)}')

def main(args=None):
    rclpy.init(args=args)
    node = AutopilotNode()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()
