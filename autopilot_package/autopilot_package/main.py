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
    def __init__(self, h, yaw, pitch, roll):
    #def __init__(self, roll, pitch, yaw, h):
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
        self.sbusChannels = [0] * 16
        self.sbus_packet = bytearray(25)

    def decode_frame(self, frame):
        print("len(frame): " + str(len(frame)))
        print("frame[0]: " + str(frame[0]))
        if len(frame) != 25 or frame[0] != 15:
            print("Invalid SBUS data")
            return False
        
        #self.sbusChannels[0]  = (((frame[1])    	|(frame[2])<<8)									& 0x07FF);
        #self.sbusChannels[1]  = (((frame[2])>>3 	|(frame[3])<<5)									& 0x07FF);
        #self.sbusChannels[2]  = (((frame[3])>>6 	|(frame[4])<<2 |(frame[5])<<10)		& 0x07FF);
        #self.sbusChannels[3]  = (((frame[5])>>1 	|(frame[6])<<7)									& 0x07FF);
        self.sbusChannels[4]  = (((frame[6])>>4 	|(frame[7])<<4)									& 0x07FF);
        self.sbusChannels[5]  = (((frame[7])>>7 	|(frame[8])<<1 |(frame[9])<<9)   	& 0x07FF);
        self.sbusChannels[6]  = (((frame[9])>>2 	|(frame[10])<<6)									& 0x07FF);
        self.sbusChannels[7]  = (((frame[10])>>5	|(frame[11])<<3)									& 0x07FF);
        self.sbusChannels[8]  = (((frame[12])   	|(frame[13])<<8)									& 0x07FF);
        self.sbusChannels[9]  = (((frame[13])>>3	|(frame[14])<<5)									& 0x07FF);
        self.sbusChannels[10] = (((frame[14])>>6	|(frame[15])<<2|(frame[16])<<10)	& 0x07FF);
        self.sbusChannels[11] = (((frame[16])>>1	|(frame[17])<<7)									& 0x07FF);
        self.sbusChannels[12] = (((frame[17])>>4	|(frame[18])<<4)									& 0x07FF);
        self.sbusChannels[13] = (((frame[18])>>7	|(frame[19])<<1|(frame[20])<<9)		& 0x07FF);
        self.sbusChannels[14] = (((frame[20])>>2	|(frame[21])<<6)									& 0x07FF);
        self.sbusChannels[15] = (((frame[21])>>5	|(frame[22])<<3)									& 0x07FF);
    
    def update_with_translator_data(self, autopilot_data, translator_data):
        print("translator_data: " + str(translator_data))
        #print("translator_data: " + str(autopilot_data))
        print("decode_frame: " + str(self.decode_frame(translator_data)))
        #if self.decode_frame(translator_data) != False:
        self.sbusChannels[0] = int(autopilot_data[0])
        self.sbusChannels[1] = int(autopilot_data[1])
        self.sbusChannels[2] = int(autopilot_data[2])
        self.sbusChannels[3] = int(autopilot_data[3])
        print("-------------sbusChannels: " + str(self.sbusChannels))
        
    def form_packet(self):	
        #print(f"Forming packet with: throttle={self.throttle}, roll={self.roll}, pitch={self.pitch}, yaw={self.yaw}")
        self.sbus_packet = bytearray(25)
        self.sbus_packet[0] = 0x0F
        for i in range(16):
            value = self.sbusChannels[i]
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
    def __init__(self, zero_throttle = 700, h_error = 0.1, leveling_throttle = 500, leveling_roll = 1, leveling_pitch = 1, SBUS_min = 173, SBUS_mid = 993, SBUS_max = 1810, max_angular_velocity = 360):
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

      return {
          'throttle': throttle,
          'roll': roll,
          'pitch': pitch,
          'yaw': yaw
      }

class AutopilotNode(Node):
    def __init__(self):
        super().__init__('autopilot_node')
        self.autopilot = StaticAutopilot()
        self.sbus = SBUS()

        self.telemetry_subscription = self.create_subscription(Quaternion, '/quaternion_topic', self.telemetry_callback, 10)
        self.translator_subscription = self.create_subscription(UInt8MultiArray, "/translator/bytes", self.translator_callback, 10)
        self.sbus_publisher = self.create_publisher(UInt8MultiArray, 'sbus_commands', 10)
        self.last_translator_channels = [0] * 16

    def telemetry_callback(self, msg):
        print(f"Полученные данные телеметрии: h={msg.x}, yaw={msg.y}, pitch={msg.z}, roll={msg.w}")

        h = msg.x
        yaw = msg.y
        pitch = msg.z
        roll = msg.w

        #telemetry = MavLink(roll, pitch, yaw, h)
        telemetry = MavLink(roll=roll, pitch=pitch, yaw=yaw, h=h)

        control_data = self.autopilot.fly(telemetry)

        autopilot_data = [control_data['roll'], control_data['pitch'], control_data['throttle'], control_data['yaw']]
        self.get_logger().info("Autopilot_data" + str(autopilot_data))

        self.sbus.update_with_translator_data(autopilot_data, self.last_translator_channels)

        sbus_packet = self.sbus.form_packet()
        message = UInt8MultiArray()
        message.data = list(sbus_packet)
        self.sbus_publisher.publish(message)

        self.get_logger().info('Published SBUS packet: ' + ' '.join(format(byte, '08b') for byte in sbus_packet))
        #self.get_logger().info(f'Published SBUS packet: {list(sbus_packet)}')

    def translator_callback(self, msg):
        if len(msg.data) == 25:
            self.sbus.decode_frame(msg.data)
            self.last_translator_channels = self.sbus.sbusChannels
        #self.get_logger().info(f'Received translator data: {self.last_translator_channels}')

def main(args=None):
    rclpy.init(args=args)
    node = AutopilotNode()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()
