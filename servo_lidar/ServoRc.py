#!/usr/bin/env python3

import rclpy
from rclpy.node import Node
from std_msgs.msg import Bool
from px4_msgs.msg import InputRc
import Jetson.GPIO as GPIO

# 물리 핀 번호 설정 (아두이노 입력 핀과 연결)
GRIPPER_PIN = 32
GIMBAL_PIN = 33

class DigitalServoNode(Node):
    def __init__(self):
        super().__init__('digital_servo_node')

        # GPIO 설정
        GPIO.setmode(GPIO.BOARD)
        # 초기값 LOW 설정
        GPIO.setup(GRIPPER_PIN, GPIO.OUT, initial=GPIO.LOW)
        GPIO.setup(GIMBAL_PIN, GPIO.OUT, initial=GPIO.LOW)

        self.input_rc = self.create_subscription(InputRc, '/fmu/out/input_rc', self.rc_callback, qos_profile)

        self.get_logger().info('Digital Signal Node Started (To Arduino)')

    def rc_callback(self, msg)
        delivery_channel = msg.values[7]
        gimbal_channel = msg.values[8]

        # delivery servo
        if delivery_channel > 1300:
            delivery_level = GPIO.HIGH
        else
            delivery_level = GPIO.LOW
          
        GPIO.output(GRIPPER_PIN, level)
        state = "HIGH (Open)" if level else "LOW (Close)"
        self.get_logger().info(f'Gripper: {state}')

        # gimbal servo
        if gimbal_channel > 1300:
            gimbal_level = GPIO.HIGH
        else
            gimbal_level = GPIO.LOW   
          
        GPIO.output(GIMBAL_PIN, level)
        state = "HIGH (Down)" if level else "LOW (Forward)"
        self.get_logger().info(f'Gimbal: {state}')
      
    def destroy_node(self):
        GPIO.cleanup()
        super().destroy_node()

def main():
    rclpy.init()
    node = DigitalServoNode()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()

if __name__ == '__main__':
    main()
