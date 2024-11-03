import numpy as np
import rclpy

from functools import partial
from rclpy.qos import qos_profile_sensor_data
from std_msgs.msg import Float32, Bool
from geometry_msgs.msg import Twist, Pose

from ros2swarm.hardware_protection_layer import HardwareProtectionLayer
from ros2swarm.utils.scan_calculation_functions import ScanCalculationFunctions
from communication_interfaces.msg import RangeData
from typing import Tuple

LEFT = 0
FRONT = 1
RIGHT = 2

# Position type
PosType = Tuple[float, float]

# Memory type
MemType = Tuple[PosType, float]

class HardwareProtectionLayerUS(HardwareProtectionLayer):
    def __init__(self):
        super().__init__()

        self.us_front_sub = self.create_subscription(
            Float32,
            self.get_namespace() + "/us_front",
            self.swarm_command_controlled(partial(self.ultrasonic_to_range, index=FRONT)),
            qos_profile=qos_profile_sensor_data,
        )

        self.us_left_sub = self.create_subscription(
            Float32,
            self.get_namespace() + "/us_left",
            self.swarm_command_controlled(partial(self.ultrasonic_to_range, index=LEFT)),
            qos_profile=qos_profile_sensor_data,
        )

        self.us_right_sub = self.create_subscription(
            Float32,
            self.get_namespace() + "/us_right",
            self.swarm_command_controlled(partial(self.ultrasonic_to_range, index=RIGHT)),
            qos_profile=qos_profile_sensor_data,
        )

        self.range_pub = self.create_publisher(
            RangeData,
            self.get_namespace() + "/range_data",
            qos_profile_sensor_data
        )

        self.has_obstacle_pub = self.create_publisher(
            Bool,
            self.get_namespace() + "/has_obstacle",
            qos_profile_sensor_data
        )

        self.ranges = [0.0, 0.0, 0.0]
        self.us_angles = [0.0, 1.5708, 3.14159]

        self.requested_drive_command = Twist()

        self.update_timer = self.create_timer(0.1, self.swarm_command_controlled_timer(self.update))

    # PSO Adaptative functions
    def initialize_aor_variables(self, k: float = 0.01, p: float = 0.9, trr: float = 0.1) -> None:
        # theta -> Angle of rotation (AoR)
        self.theta = 0 
        # s -> Dynamic change step for AoR
        self.s = 1 if np.random.rand() > 0.5 else -1
        # dc -> Probability of changing s each interaction
        self.dc = 0
        # temperature -> Prevent theta decreasing too quickly
        self.temperature = 0
        # Trr -> Temperature reduction rate
        self.Trr = trr

        # Constants
        self.k = k
        self.p = p

    def initialize_mem_tool(self, mms: int = 10, mpr: float = 0.1, md: float = 0.1, mpp1: float = 0.1, mpp2: float = 0.1, FFthr: float = 0.0) -> None:
        self.mem: list[MemType] = []
        self.mms = mms # Maximum Memory Size
        self.mpr = mpr # Memory Point Radius
        self.md = md # Minimum Displacement
        self.mpp1 = mpp1 # Memory Point Penalty 1
        self.mpp2 = mpp2 # Memory Point Penalty 2
        self.FFthr = FFthr # Threshold

    def update_theta(self, direction: Twist) -> np.ndarray:
        V = [direction.linear.x, direction.angular.z]
        
        # Update velocity using AoR concept [7]
        RV = [[np.cos(self.theta), -np.sin(self.theta)], [np.sin(self.theta), np.cos(self.theta)]]
        RV = np.matmul(RV, V)

        # Update theta with temperature [8]
        self.theta = self.temperature * self.theta

        # Update temperature [9]
        self.temperature = self.Trr * self.temperature

        return RV

    # HPL functions
    def ultrasonic_to_range(self, msg: Float32, index: int):
        self.ranges[index] = msg.data

        ranges: RangeData = RangeData()
        ranges.header.stamp = self.get_clock().now().to_msg()
        ranges.header.frame_id = self.get_namespace()

        ranges.angles = self.us_angles
        ranges.ranges = self.ranges

        self.range_pub.publish(ranges)

    def vector_calc(self):
        if self.current_ranges is None:
            print("[DEBUG] No range data received")
            return [False, None]

        avoid_distance = self.param_max_range
        direction, obstacle_free = ScanCalculationFunctions.potential_field(self.param_front_attraction,
                                                     avoid_distance,
                                                     self.param_max_rotational_velocity,
                                                     self.param_max_translational_velocity,
                                                     self.param_min_range,
                                                     self.param_threshold,
                                                     self.current_ranges,
                                                     self.angles)


        avoid_needed = not obstacle_free
        direction = self.wrap_direction(direction)

        return [avoid_needed, direction]
    
    def wrap_direction(self, direction: Twist):
        direction.linear.x = np.clip(direction.linear.x, -self.param_max_translational_velocity, self.param_max_translational_velocity)
        direction.angular.z = np.clip(direction.angular.z, -self.param_max_rotational_velocity, self.param_max_rotational_velocity)
        return direction

    def command_callback(self, msg):
        self.get_logger().debug('heard: "%s"' % msg)

        self.requested_drive_command = msg

    def range_data_callback(self, msg):
        self.get_logger().debug('heard: "%s"' % msg)
        self.current_ranges = msg.ranges
        self.angles = msg.angles      

    def update(self):
        avoid_distance = self.param_max_range
        direction = self.requested_drive_command

        ranges = ScanCalculationFunctions.adjust_ranges(self.current_ranges, self.param_min_range, avoid_distance)
        obstacle_free = ScanCalculationFunctions.is_obstacle_free(avoid_distance, ranges, self.param_threshold)

        if not obstacle_free:
            self.get_logger().info('Obstacle detected')
        
        self.has_obstacle_pub.publish(Bool(data=(not obstacle_free)))

        direction = self.wrap_direction(direction)
        self.publisher_cmd_vel.publish(direction)

def main(args=None):
    rclpy.init(args=args)

    node = HardwareProtectionLayerUS()
    rclpy.spin(node)
    node.destroy_node()

    rclpy.shutdown()


if __name__ == '__main__':
    main()
