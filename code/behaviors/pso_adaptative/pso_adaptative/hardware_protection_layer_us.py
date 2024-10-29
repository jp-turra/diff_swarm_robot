import rclpy

from rclpy.qos import qos_profile_sensor_data

from std_msgs.msg import Float32

from ros2swarm.hardware_protection_layer import HardwareProtectionLayer

from communication_interfaces.msg import RangeData

from functools import partial

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

        self.current_angles = [0.0, -90.0, 90.0]
        self.current_ranges = [0.0, 0.0, 0.0]

    def ultrasonic_to_range(self, msg: Float32, index: int):
        self.current_ranges[index] = msg.data

        ranges: RangeData = RangeData()
        ranges.ranges = self.current_ranges
        ranges.angles = self.current_angles

        self.range_data_callback(ranges)    

def main(args=None):
    rclpy.init(args=args)
    node = HardwareProtectionLayerUS()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()