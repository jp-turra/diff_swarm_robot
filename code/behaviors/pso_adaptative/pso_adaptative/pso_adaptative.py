import rclpy
import numpy as np

from rclpy.qos import qos_profile_sensor_data, QoSProfile, QoSHistoryPolicy
from geometry_msgs.msg import Twist, Pose, Vector3

from ros2swarm.movement_pattern.movement_pattern import MovementPattern
from ros2swarm.utils import setup_node

class PSOAdaptative(MovementPattern):
    def __init__(self, node_name):
        super().__init__(node_name)

        self.declare_parameters(
            namespace='',
            parameters=[
                ('pso_w', 0.9),
                ('pso_c1', 0.5),
                ('pso_c2', 0.3),
            ]
        )

        self.pso_timer = self.create_timer(
            0.1,
            self.swarm_command_controlled_timer(self.pso_interation)
        )

        self.pso_w = float(self.get_parameter("pso_w").get_parameter_value().double_value)
        self.pso_c1 = float(self.get_parameter("pso_c1").get_parameter_value().double_value)
        self.pso_c2 = float(self.get_parameter("pso_c2").get_parameter_value().double_value)

        self.pose_sub = self.create_subscription(
            Pose,
            self.get_namespace() + '/pose',
            self.pose_callback,
            qos_profile_sensor_data
        )

        gbest_qos = QoSProfile(
            history=QoSHistoryPolicy.KEEP_LAST,
            depth=1,
            avoid_ros_namespace_conventions=True
        )
        self.gbest_pub = self.create_publisher(Vector3, '/gbest', gbest_qos)
        self.gbest_sub = self.create_subscription(
            Vector3,
            '/gbest',
            self.gbest_callback,
            gbest_qos
        )

        self.target = np.array([1, 1])

        self.X = np.array([0, 0])
        self.V = np.random.rand(2)

        self.pbest = self.X.copy()
        self.gbest = self.X.copy()

    def gbest_callback(self, msg: Vector3):
        self.gbest = np.array([msg.x, msg.y])

    def pose_callback(self, msg: Pose):
        self.X = np.array([msg.position.x, msg.position.y])

    def distance_to_target(self, pos: np.ndarray) -> float:
        return np.sqrt((pos[0] - self.target[0])**2 + (pos[1] - self.target[1])**2)

    def pso_evalute_bests(self):
        pbest_cost = self.distance_to_target(self.pbest)
        gbest_cost = self.distance_to_target(self.gbest)
        local_cost = self.distance_to_target(self.X)

        if local_cost < pbest_cost:
            self.pbest = self.X

        if pbest_cost < gbest_cost:
            self.gbest = self.pbest
            self.gbest_pub.publish(Vector3(x=self.gbest[0], y=self.gbest[1], z=0.0))

    def pso_update_veloticy(self):
        r1, r2 = np.random.rand(2)

        self.V = self.pso_w * self.V + r1 * self.pso_c1 * (self.pbest - self.X) + r2 * self.pso_c2 * (self.gbest - self.X)
        
    def pso_interation(self):
        self.pso_update_veloticy()
        self.pso_evalute_bests()

        self.command_publisher.publish(
            Twist(
                linear=Vector3(x=self.V[0], y=0.0, z=0.0),
                angular=Vector3(x=0.0, y=0.0, z=self.V[1])
            )
        )

        if self.distance_to_target(self.gbest) < 0.2:
            self.get_logger().info(f"Target reached. Cost: {self.distance_to_target(self.gbest):.2f} Pos: {self.gbest}")
            self.pso_timer.cancel()
            self.destroy_node()

    def destroy_node(self):
        self.command_publisher.publish(Twist())
        super().destroy_node()

def main(args=None):
    rclpy.init(args=args)
    pattern_node = PSOAdaptative('pso_adaptative')
    rclpy.spin(pattern_node)
    pattern_node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()