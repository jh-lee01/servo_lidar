import rclpy
from rclpy.node import Node
from std_msgs.msg import Float32

from smbus2 import SMBus
import time
from collections import deque


class LidarLiteNode(Node):
    def __init__(self):
        super().__init__('filtered_lidar_node')

        # I2C 설정
        self.i2c_bus_num = 7
        self.i2c_addr = 0x62
        self.bus = SMBus(self.i2c_bus_num)

        # Publisher
        self.publisher_ = self.create_publisher(Float32, 'lidar_height', 10)

        # Timer (40 Hz)
        self.timer = self.create_timer(0.25, self.timer_callback)

        # Median filter 설정
        self.window_size = 3
        self.buffer = deque(maxlen=self.window_size)

        self.get_logger().info('LiDAR-Lite v3 node started (median filter enabled)')


    def timer_callback(self):
        try:
            distance = self.read_distance()   # raw distance (m)

            # buffer에 저장
            self.buffer.append(distance)

            # window가 찼을 때만 median 계산
            if len(self.buffer) == self.window_size:
                filtered_distance = self.median(self.buffer)

                msg = Float32()
                msg.data = filtered_distance
                self.publisher_.publish(msg)

        except Exception as e:
            self.get_logger().error(f'I2C read failed: {e}')


    def read_distance(self):
        # 거리 측정 트리거
        self.bus.write_byte_data(self.i2c_addr, 0x00, 0x04)
        time.sleep(0.02)  # 측정 대기 (20 ms)

        # 거리 읽기 (2바이트)
        high = self.bus.read_byte_data(self.i2c_addr, 0x0f)
        low  = self.bus.read_byte_data(self.i2c_addr, 0x10)

        distance_cm = (high << 8) | low
        return distance_cm / 100.0  # meter


    def median(self, data):
        sorted_data = sorted(data)
        return sorted_data[len(sorted_data) // 2]


def main(args=None):
    rclpy.init(args=args)
    node = LidarLiteNode()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()
