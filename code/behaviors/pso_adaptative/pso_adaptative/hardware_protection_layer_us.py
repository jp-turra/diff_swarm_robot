import numpy as np
import rclpy
import time

from functools import partial
from rclpy.qos import qos_profile_sensor_data
from std_msgs.msg import Float32, Bool
from geometry_msgs.msg import Twist

from ros2swarm.hardware_protection_layer import HardwareProtectionLayer
from ros2swarm.utils.scan_calculation_functions import ScanCalculationFunctions
from communication_interfaces.msg import RangeData

LEFT = 0
FRONT = 1
RIGHT = 2


class HardwareProtectionLayerUS(HardwareProtectionLayer):
    def __init__(self):
        super().__init__()
        self.get_logger().info("HardwareProtectionLayerUS initializing...")

        self.us_front_sub = self.create_subscription(
            Float32,
            self.get_namespace() + "/us_front",
            self.swarm_command_controlled(
                partial(self.ultrasonic_to_range, index=FRONT)
            ),
            qos_profile=qos_profile_sensor_data,
        )

        self.us_left_sub = self.create_subscription(
            Float32,
            self.get_namespace() + "/us_left",
            self.swarm_command_controlled(
                partial(self.ultrasonic_to_range, index=LEFT)
            ),
            qos_profile=qos_profile_sensor_data,
        )

        self.us_right_sub = self.create_subscription(
            Float32,
            self.get_namespace() + "/us_right",
            self.swarm_command_controlled(
                partial(self.ultrasonic_to_range, index=RIGHT)
            ),
            qos_profile=qos_profile_sensor_data,
        )

        self.has_obstacle_pub = self.create_publisher(
            Bool, self.get_namespace() + "/has_obstacle", qos_profile_sensor_data
        )

        self.current_ranges = [0.0, 0.0, 0.0]
        self.us_angles = [0.0, 1.5708, 3.14159]
        self.rotate_init_time = None
        self.rotate_left = True

        self.requested_drive_command = Twist()

        self.update_timer = self.create_timer(
            0.1, self.swarm_command_controlled_timer(self.update)
        )

        self.get_logger().info("HardwareProtectionLayerUS initialized")

    # HPL functions
    def ultrasonic_to_range(self, msg: Float32, index: int):
        self.current_ranges[index] = msg.data

    def vector_calc(self):
        if self.current_ranges is None:
            print("[DEBUG] No range data received")
            return [False, None]

        avoid_distance = self.param_max_range
        direction, obstacle_free = ScanCalculationFunctions.potential_field(
            self.param_front_attraction,
            avoid_distance,
            self.param_max_rotational_velocity,
            self.param_max_translational_velocity,
            self.param_min_range,
            self.param_threshold,
            self.current_ranges,
            self.angles,
        )

        avoid_needed = not obstacle_free
        direction = self.wrap_direction(direction)

        return [avoid_needed, direction]

    def wrap_direction(self, direction: Twist):
        direction.linear.x = np.clip(
            direction.linear.x,
            -self.param_max_translational_velocity,
            self.param_max_translational_velocity,
        )
        direction.angular.z = np.clip(
            direction.angular.z,
            -self.param_max_rotational_velocity,
            self.param_max_rotational_velocity,
        )
        return direction

    def command_callback(self, msg):
        self.get_logger().debug('heard: "%s"' % msg)

        self.requested_drive_command = msg

    def range_data_callback(self, msg):
        self.get_logger().debug('heard: "%s"' % msg)
        self.current_ranges = msg.ranges
        self.angles = msg.angles

    def wait_and_rotate(self):
        command = Twist()
        turn_time = np.random.rand() * 3 + 2
        if time.time() - self.rotate_init_time > turn_time:
            self.rotate_init_time = None
        elif time.time() - self.rotate_init_time > 2:
            rotation_sign = 1 if self.rotate_left else -1
            command.angular.z = self.param_max_rotational_velocity * rotation_sign

        return command

    def is_obstacle_free(max_range, ranges, threshold):
        """Return true if no obstacle is detected within the max_range, false otherwise."""
        total = 0
        if 0 < ranges[FRONT] < max_range:
            total += 1
        elif 0 < ranges[LEFT] < max_range / 2:
            total += 1
        elif 0 < ranges[RIGHT] < max_range / 2:
            total += 1

        return total <= threshold

    def update(self):
        avoid_distance = self.param_max_range
        command = self.requested_drive_command

        ranges = ScanCalculationFunctions.adjust_ranges(
            self.current_ranges, self.param_min_range, avoid_distance
        )
        obstacle_free = ScanCalculationFunctions.is_obstacle_free(
            avoid_distance, ranges, self.param_threshold
        )

        if not obstacle_free:
            self.get_logger().debug("Obstacle detected")

        self.has_obstacle_pub.publish(Bool(data=(not obstacle_free)))

        if not obstacle_free and self.rotate_init_time is None:
            # Add a behavior that stops the robot, wait random x seconds and rotates randomly and start again
            self.rotate_init_time = time.time()
            self.rotate_left = (
                self.current_ranges[LEFT] > self.current_ranges[RIGHT]
                or self.current_ranges[LEFT] == 0
            )

            command = Twist()
        elif self.rotate_init_time is not None:
            command = self.wait_and_rotate()

        # if np.abs(command.linear.x) > self.param_max_translational_velocity:
        #     command.linear.x = (
        #         np.sign(command.linear.x) * self.param_max_translational_velocity
        #     )

        # if np.abs(command.angular.z) > self.param_max_rotational_velocity:
        #     command.angular.z = (
        #         np.sign(command.angular.z) * self.param_max_rotational_velocity
        #     )

        self.publisher_cmd_vel.publish(command)


def main(args=None):
    rclpy.init(args=args)

    node = HardwareProtectionLayerUS()
    rclpy.spin(node)
    node.destroy_node()

    rclpy.shutdown()


if __name__ == "__main__":
    main()
