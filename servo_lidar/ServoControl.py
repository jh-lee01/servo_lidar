#!/usr/bin/env python3

import rclpy
from rclpy.node import Node
from std_msgs.msg import Bool
import Jetson.GPIO as GPIO
import time

PWM_PIN = 33        # 물리 핀 번호 33
PWM_FREQ = 50       # 서보모터 주파수

INIT_ANGLE = 0
TARGET_ANGLE = 180


def angle_to_duty(angle):
    return 2.5 + (angle / 180.0) * 10.0


class ServoNode(Node):
    def __init__(self):
        super().__init__('servo_node')

        self.subscription = self.create_subscription(
            Bool,
            'servo_flag',
            self.callback,
            10
        )

        GPIO.setmode(GPIO.BOARD)
        GPIO.setup(PWM_PIN, GPIO.OUT)

        self.pwm = GPIO.PWM(PWM_PIN, PWM_FREQ)
        self.pwm.start(angle_to_duty(INIT_ANGLE))

        self.current_state = False
        self.get_logger().info('Servo node started')

    def callback(self, msg: Bool):
        if msg.data == self.current_state:
            return

        self.current_state = msg.data

        if msg.data:
            angle = TARGET_ANGLE
            self.get_logger().info('Move servo to TARGET position')
        else:
            angle = INIT_ANGLE
            self.get_logger().info('Move servo to INIT position')

        duty = angle_to_duty(angle)
        self.pwm.ChangeDutyCycle(duty)

    def destroy_node(self):
        self.pwm.stop()
        GPIO.cleanup()
        super().destroy_node()


def main():
    rclpy.init()
    node = ServoNode()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
