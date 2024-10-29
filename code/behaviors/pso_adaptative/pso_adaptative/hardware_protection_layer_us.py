import numpy as np
import rclpy

from functools import partial
from rclpy.qos import qos_profile_sensor_data
from std_msgs.msg import Float32, Twist

from ros2swarm.hardware_protection_layer import HardwareProtectionLayer
from communication_interfaces.msg import RangeData


FRONT=0
LEFT=1
RIGHT=2

class HardwareProtectionLayerUS(HardwareProtectionLayer):
    def __init__(self):
        super().__init__()

        self.create_subscription(
            Float32,
            self.get_namespace() + '/us_front',
            self.swarm_command_controlled(
                partial(self.ultrasonic_to_range, index=FRONT)
            ),
            qos_profile=qos_profile_sensor_data
        )

        self.create_subscription(
            Float32,
            self.get_namespace() + '/us_left',
            self.swarm_command_controlled(
                partial(self.ultrasonic_to_range, index=LEFT)
            ),
            qos_profile=qos_profile_sensor_data
        )

        self.create_subscription(
            Float32,
            self.get_namespace() + '/us_right',
            self.swarm_command_controlled(
                partial(self.ultrasonic_to_range, index=RIGHT)
            ),
            qos_profile=qos_profile_sensor_data
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

        [adjust, direction] = self.vector_calc(msg)

        if adjust:
            msg = direction
            self.get_logger().debug('Adjusting to"%s"' % direction)

        self.publisher_cmd_vel.publish(msg)

    def vector_calc(self, msg: Twist):
        """
        Calculate an avoidance vector and if it is needed to avoid.

        Returns
        -------
        [avoid_needed{boolean}, direction{Twist}]

        """
        avoid_needed = False

        if self.current_ranges is None:
            return [False, None]

        direction = Twist()
        avoid_distance = self.param_max_range
        stop_distance = self.param_min_range

        if self.current_ranges[FRONT] < avoid_distance:
            direction.linear.x = 0.2

        # if self.current_ranges is None:
        #     return [False, None]

        # avoid_distance = self.param_max_range
        # direction, obstacle_free = ScanCalculationFunctions.potential_field(self.param_front_attraction,
        #                                              avoid_distance,
        #                                              self.param_max_rotational_velocity,
        #                                              self.param_max_translational_velocity,
        #                                              self.param_min_range,
        #                                              self.param_threshold,
        #                                              self.current_ranges,
        #                                              self.angles)
        # avoid_needed = not obstacle_free

        return [avoid_needed, direction]

def main(args=None):
    rclpy.init(args=args)
    node = HardwareProtectionLayerUS()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()