import rclpy
from rclpy.node import Node
from sensor_msgs.msg import Imu
import numpy as np

class ImuNoiseNode(Node):
    def __init__(self):
        super().__init__('imu_noise_node')
        self.subscription = self.create_subscription(
            Imu,
            'imu_no_noise',
            self.imu_noise_callback,
            10)
        self.publisher = self.create_publisher(Imu, 'imu', 10)

    def imu_noise_callback(self, msg):
        # Add noise to the IMU data here
        msg.orientation_covariance = np.diag(np.repeat(0.05, 3)).reshape(-1)
        msg.angular_velocity_covariance = np.diag(np.repeat(0.001, 3)).reshape(-1)
        msg.linear_acceleration_covariance = np.diag(np.repeat(0.05, 3)).reshape(-1)

        # Publish the IMU data
        self.publisher.publish(msg)

def main(args=None):
    rclpy.init(args=args)
    imu_noise_node = ImuNoiseNode()
    rclpy.spin(imu_noise_node)
    imu_noise_node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()
