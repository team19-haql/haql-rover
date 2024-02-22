from typing import Optional

import rclpy
import numpy as np

# Node, State and Publisher are aliases for LifecycleNode, LifecycleState and LifecyclePublisher
# respectively.
# In case of ambiguity, the more explicit names can be imported.

from rclpy.node import Node

from sensor_msgs.msg import Imu, BatteryState, Temperature
from std_msgs.msg import Float32

class I2CBus(Node):
    def __init__(self):
        super().__init__('i2c_bus')

        self.imu_pub = self.create_publisher(Imu, 'i2c_bus/imu', 10)
        self.battery_pub = self.create_publisher(BatteryState, 'i2c_bus/battery', 10)
        self.temperature_pub = self.create_publisher(Temperature, 'i2c_bus/temperature', 10)
        self.current_pub = self.create_publisher(Float32, 'i2c_bus/motor_current', 10)

        self.timer_ = self.create_timer(1 / 50, self.timer_callback)
        self.last_time = self.get_clock().now()
        self.logs = np.zeros(100, dtype=np.float64)
        self.i = 0

    def timer_callback(self):
        elapsed = self.get_clock().now() - self.last_time
        self.last_time = self.get_clock().now()
        us = elapsed.nanoseconds / 1e3
        # self.get_logger().info(f'Timer callback ({int(us):>5} us)')
        self.logs[self.i] = us
        self.i = (self.i + 1) % 100

        if (self.i == 0):
            self.get_logger().info(f'Mean: {int(np.mean(self.logs)):>7} us, Std dev: {int(np.std(self.logs)):>7} us')
    
def main():
    rclpy.init()

    executor = rclpy.executors.SingleThreadedExecutor()
    node = I2CBus()
    executor.add_node(node)
    try:
        executor.spin()
    except (KeyboardInterrupt, rclpy.executors.ExternalShutdownException):
        node.destroy_node()


if __name__ == '__main__':
    main()