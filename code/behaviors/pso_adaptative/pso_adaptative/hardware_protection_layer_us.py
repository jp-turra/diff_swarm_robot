import numpy as np
import rclpy

from functools import partial
from rclpy.qos import qos_profile_sensor_data
from std_msgs.msg import Float32
from geometry_msgs.msg import Twist

from ros2swarm.hardware_protection_layer import HardwareProtectionLayer
from communication_interfaces.msg import RangeData


FRONT = 0
LEFT = 1
RIGHT = 2


class HardwareProtectionLayerUS(HardwareProtectionLayer):
    def __init__(self):
        super().__init__()

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

        self.current_angles = [0.0, -np.pi, np.pi]
        self.current_ranges = [0.0, 0.0, 0.0]
        self.angles = self.current_angles

    def ultrasonic_to_range(self, msg: Float32, index: int):
        self.current_ranges[index] = msg.data

        ranges: RangeData = RangeData()
        ranges.ranges = self.current_ranges
        ranges.angles = self.current_angles

        self.range_data_callback(ranges)

    def command_callback(self, msg: Twist):
        self.get_logger().debug('heard: "%s"' % msg)

        [adjust, direction] = self.check_collision(msg)

        if adjust:
            msg = direction
            self.get_logger().debug('Adjusting to"%s"' % direction)

        msg = self.apply_limits_to_command(msg)

        self.publisher_cmd_vel.publish(msg)

    def apply_limits_to_command(self, msg: Twist):
        if np.abs(msg.linear.x) > self.param_max_translational_velocity:
            msg.linear.x = np.sign(msg.linear.x) * self.param_max_translational_velocity
        if np.abs(msg.angular.z) > self.param_max_rotational_velocity:
            msg.angular.z = np.sign(msg.angular.z) * self.param_max_rotational_velocity

        return msg

    def check_collision(self, msg: Twist):
        if self.current_ranges is None:
            return [False, None]

        avoid_needed = False
        direction = msg

        stop_distance = self.param_min_range
        avoid_distance = self.param_max_range

        # Frontal collision
        if (
            self.current_ranges[FRONT] < stop_distance
            and self.current_ranges[FRONT] > 0
        ):
            self.get_logger().warning("Front Collision detected")
            direction.linear.x = 0.0
            avoid_needed = True
        # Left collision
        if self.current_ranges[LEFT] < stop_distance and self.current_ranges[LEFT] > 0:
            self.get_logger().warning("Left Collision detected")
            # I need to turn slowly to right ( negative )
            direction.angular.z -= 0.01 * self.param_max_rotational_velocity
            avoid_needed = True
        # Right collision
        if (
            self.current_ranges[RIGHT] < stop_distance
            and self.current_ranges[RIGHT] > 0
        ):
            self.get_logger().warning("Right Collision detected")
            # I need to turn slowly to left ( positive )
            direction.angular.z += 0.01 * self.param_max_rotational_velocity
            avoid_needed = True

        self.get_logger().info(f"Distances: {self.current_ranges}")

        return [avoid_needed, direction]

        # avoid_distance = self.param_max_range
        # direction, obstacle_free = ScanCalculationFunctions.potential_field(self.param_front_attraction,
        #                                              avoid_distance,
        #                                              self.param_max_rotational_velocity,
        #                                              self.param_max_translational_velocity,
        #                                              self.param_min_range,
        #                                              self.param_threshold,
        #                                              self.current_ranges,
        #                                              self.angles)


def main(args=None):
    rclpy.init(args=args)
    node = HardwareProtectionLayerUS()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()


if __name__ == "__main__":
    main()
