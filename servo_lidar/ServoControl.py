#!/usr/bin/env python3

import rclpy
from rclpy.node import Node
from std_msgs.msg import Bool
import Jetson.GPIO as GPIO

GPIO.setwarnings(False)

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

        self.create_subscription(
            Bool, 'delivery_open_flag', self.delivery_callback, 10
        )
        self.create_subscription(
            Bool, 'gimbal_look_down_flag', self.gimbal_callback, 10
        )

        self.get_logger().info('Digital Signal Node Started (To Arduino)')

    def delivery_callback(self, msg: Bool):
        # True(Open) -> HIGH, False(Init) -> LOW
        level = GPIO.HIGH if msg.data else GPIO.LOW
        GPIO.output(GRIPPER_PIN, level)
        
        state = "HIGH (Open)" if level else "LOW (Close)"
        self.get_logger().info(f'Gripper: {state}')

    def gimbal_callback(self, msg: Bool):
        # True(Down) -> HIGH, False(Forward) -> LOW
        level = GPIO.HIGH if msg.data else GPIO.LOW
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