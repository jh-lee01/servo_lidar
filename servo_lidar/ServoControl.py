#!/usr/bin/env python3

import rclpy
from rclpy.node import Node
from std_msgs.msg import Bool
import Jetson.GPIO as GPIO
import threading
import time


class ServoControl(Node):
    def __init__(self):
        super().__init__('servo_control')

        # ===== GPIO 설정 =====
        self.servo_pin = 33           # BOARD 기준
        GPIO.setmode(GPIO.BOARD)
        GPIO.setup(self.servo_pin, GPIO.OUT)

        # ===== Servo 설정 =====
        self.pwm_hz = 50.0
        self.period = 1.0 / self.pwm_hz
        self.initial_angle = 0.0
        self.target_angle = 90.0

        self.current_angle = self.initial_angle
        self.running = True

        # ===== ROS2 Subscriber =====
        self.sub = self.create_subscription(
            Bool,
            'servo_flag',
            self.flag_callback,
            10
        )

        # ===== PWM Thread =====
        self.pwm_thread = threading.Thread(target=self.pwm_loop)
        self.pwm_thread.daemon = True
        self.pwm_thread.start()

        self.get_logger().info('Software PWM servo node started')

    def angle_to_pulse(self, angle):
        """
        각도(deg) → 펄스폭(sec)
        0deg = 1.0ms, 180deg = 2.0ms
        """
        return 0.001 + (angle / 180.0) * 0.001

    def pwm_loop(self):
        while self.running:
            pulse_width = self.angle_to_pulse(self.current_angle)

            GPIO.output(self.servo_pin, GPIO.HIGH)
            time.sleep(pulse_width)

            GPIO.output(self.servo_pin, GPIO.LOW)
            time.sleep(self.period - pulse_width)

    def flag_callback(self, msg):
        if msg.data:
            self.current_angle = self.target_angle
        else:
            self.current_angle = self.initial_angle

    def destroy_node(self):
        self.running = False
        self.pwm_thread.join()
        GPIO.cleanup()
        super().destroy_node()


def main(args=None):
    rclpy.init(args=args)
    node = ServoControl()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
