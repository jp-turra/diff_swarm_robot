import rclpy
import math
import numpy as np

from rclpy.node import Node
from geometry_msgs.msg import Vector3
from sensor_msgs.msg import Imu

ACCEL_TOPIC = "/sim/imu/accel"
GYRO_TOPIC = "/sim/imu/gyro"

DEFAULT_QOS = 10

class CopSimImuNode(Node):
    def __init__(self):
        super().__init__("cop_sim_imu")
        self.get_logger().info("CopSimImuNode started")

        self.create_subscription(
            msg_type=Vector3,
            topic=ACCEL_TOPIC,
            callback=self.accel_callback,
            qos_profile=DEFAULT_QOS
        )

        self.create_subscription(
            msg_type=Vector3,
            topic=GYRO_TOPIC,
            callback=self.gyro_callback,
            qos_profile=DEFAULT_QOS
        )

        self.imu_pub = self.create_publisher(Imu, "/sim/imu", DEFAULT_QOS)
        self.imu_msg: Imu = Imu()

        self.imu_msg.header.frame_id = "base_link"

        self.imu_timer = self.create_timer(
            0.01,
            lambda: self.imu_pub.publish(self.imu_msg)
        )

        self.accel_history = [0, 0, 0]
        self.accel_history_index = 0
        self.gyro_history = [0, 0, 0]
        self.gyro_history_index = 0
        self.quat_history = [0, 0, 0]
        self.quat_history_index = 0

    def accel_callback(self, msg: Vector3):
        self.imu_msg.header.stamp = self.get_clock().now().to_msg()
        self.imu_msg.linear_acceleration.x = float(msg.x)
        self.imu_msg.linear_acceleration.y = float(msg.y)
        self.imu_msg.linear_acceleration.z = float(msg.z)

        self.calculate_accel_quaternion(msg)

        self.accel_history[self.accel_history_index] = [msg.x, msg.y, msg.z]

        self.accel_history_index += 1
        if self.accel_history_index >= 3:
            self.accel_history_index = 0

        self.imu_msg.linear_acceleration_covariance = self.calculate_covariance_matrix(self.accel_history)

    def gyro_callback(self, msg: Vector3):
        self.imu_msg.header.stamp = self.get_clock().now().to_msg()
        self.imu_msg.angular_velocity.x = float(msg.x)
        self.imu_msg.angular_velocity.y = float(msg.y)
        self.imu_msg.angular_velocity.z = float(msg.z)

        self.gyro_history[self.gyro_history_index] = [msg.x, msg.y, msg.z]
        self.gyro_history_index += 1
        if self.gyro_history_index >= 3:
            self.gyro_history_index = 0

        self.imu_msg.angular_velocity_covariance = self.calculate_covariance_matrix(self.gyro_history)


    def calculate_accel_quaternion(self, accel: Vector3):
        az = accel.z
        ay = accel.y

        if az >= 0:
            q0 = math.sqrt((az+ 1)/2)
            q1 = -ay/(math.sqrt(2*(az+1)))
            q2 = az/(math.sqrt(2*(az+1)))
            q3 = 0.0
        else:
            q0 = -ay/(math.sqrt(2*(1-az)))
            q1 = math.sqrt((1-az)/2)
            q2 = 0.0
            q3 = az/(math.sqrt(2*(1-az)))

        self.imu_msg.orientation.w = q0
        self.imu_msg.orientation.x = q1
        self.imu_msg.orientation.y = q2
        self.imu_msg.orientation.z = q3

        self.quat_history[self.quat_history_index] = [q0, q1, q2, q3]
        self.quat_history_index += 1
        if self.quat_history_index >= 3:
            self.quat_history_index = 0

        self.imu_msg.orientation_covariance = self.calculate_covariance_matrix(self.quat_history)

    def calculate_covariance_matrix(self, history: list[float]):
        if not np.iterable(history[0]) or not np.iterable(history[1]) or not np.iterable(history[2]):
            return [0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0]
        
        history = np.array(history, dtype=np.float16)
        
        n = history.shape[0]
        x = history[:, 0]
        y = history[:, 1]
        z = history[:, 2]

        x_var = np.sum((x - np.mean(x))**2, axis=0) / (n-1)
        y_var = np.sum((y - np.mean(y))**2, axis=0) / (n-1)
        z_var = np.sum((z - np.mean(z))**2, axis=0) / (n-1)

        xy_cov = np.sum((x - np.mean(x)) * (y - np.mean(y)), axis=0) / n
        xz_cov = np.sum((x - np.mean(x)) * (z - np.mean(z)), axis=0) / n
        yz_cov = np.sum((y - np.mean(y)) * (z - np.mean(z)), axis=0) / n

        return [
            x_var, xy_cov, xz_cov,
            xy_cov, y_var, yz_cov,
            xz_cov, yz_cov, z_var
        ]
        

def main(args=None):
    rclpy.init(args=args)
    cop_sim_imu = CopSimImuNode()
    rclpy.spin(cop_sim_imu)
    cop_sim_imu.destroy_node()
    rclpy.shutdown()

if __name__ == "__main__":
    main()