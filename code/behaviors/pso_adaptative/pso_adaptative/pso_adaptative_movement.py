import rclpy
import numpy as np

from rclpy.qos import qos_profile_sensor_data, QoSProfile, QoSHistoryPolicy
from geometry_msgs.msg import Twist, Pose, Vector3, Vector3Stamped
from std_msgs.msg import Bool, Header
from communication_interfaces.msg import Int8Message

from ros2swarm.movement_pattern.movement_pattern import MovementPattern

np.printoptions(precision=2)


class PSOAdaptative(MovementPattern):
    def __init__(self, node_name):
        super().__init__(node_name)
        self.get_logger().info("PSOAdaptative initializing...")

        self.declare_parameters(
            namespace="",
            parameters=[
                ("pso_w", 1.0),  # Velocity smoother
                ("pso_c1", 0.5),
                ("pso_c2", 0.3),
                ("target", [1.0, 1.0]),
            ],
        )

        self.pso_w = float(
            self.get_parameter("pso_w").get_parameter_value().double_value
        )
        self.pso_c1 = float(
            self.get_parameter("pso_c1").get_parameter_value().double_value
        )
        self.pso_c2 = float(
            self.get_parameter("pso_c2").get_parameter_value().double_value
        )

        self.target = np.array(
            self.get_parameter("target").get_parameter_value().double_array_value
        )

        self.swarm_command_pub = self.create_publisher(
            Int8Message,
            "/swarm_command",
            1,
        )

        self.X: np.ndarray = None
        self.last_X = np.array([0, 0])
        self.V = np.random.rand(2) * 0.2
        self.dt = 1.0

        self.pbest: np.ndarray = None
        self.gbest: np.ndarray = None

        self.has_obstacle = False

        self.gbest_pub = self.create_publisher(Vector3Stamped, "/gbest", 2)
        self.gbest_sub = self.create_subscription(
            Vector3Stamped,
            "/gbest",
            self.swarm_command_controlled(self.gbest_callback),
            1,
        )
        self.pose_sub = self.create_subscription(
            Pose,
            self.get_namespace() + "/pose",
            self.swarm_command_controlled(self.pose_callback),
            qos_profile_sensor_data,
        )

        self.has_obstacle_sub = self.create_subscription(
            Bool,
            self.get_namespace() + "/has_obstacle",
            self.swarm_command_controlled(self.has_obstacle_callback),
            qos_profile_sensor_data,
        )

        self.pso_timer = self.create_timer(
            self.dt, self.swarm_command_controlled_timer(self.pso_interation)
        )
        self.pso_timer.cancel()

        self.log_timer = self.create_timer(
            1.0,
            lambda: self.get_logger().info(
                f"P: {self.X} | V: {self.V} | PBest: {self.pbest} | GBest: {self.gbest}"
            ),
        )
        self.log_timer.cancel()

        self.pose_lock = False

        self.get_logger().info("PSOAdaptative initialized...")

    def has_obstacle_callback(self, msg: Bool):
        self.has_obstacle = msg.data

    def gbest_callback(self, msg: Vector3Stamped):
        if msg.header.frame_id != self.get_namespace():
            self.gbest = np.array([msg.vector.x, msg.vector.y])

    def pose_callback(self, msg: Pose):
        if self.X is None:
            self.X = np.array([msg.position.x, msg.position.y])
        else:
            self.X[0] = msg.position.x
            self.X[1] = msg.position.y

        if self.pso_timer.is_canceled():
            self.get_logger().info("PSOAdaptative TIMER started...")
            self.pbest = np.array([msg.position.x, msg.position.y])
            self.gbest = np.array([msg.position.x, msg.position.y])
            self.pso_timer.reset()
            # self.log_timer.reset()

    def distance_to_target(self, pos: np.ndarray) -> float:
        return np.linalg.norm(pos - self.target)

    def evalute_bests(self):
        pbest_cost = self.distance_to_target(self.pbest)
        gbest_cost = self.distance_to_target(self.gbest)
        local_cost = self.distance_to_target(self.X)

        if local_cost < pbest_cost:
            self.get_logger().info(
                f"PBest improved. Cost: {self.distance_to_target(self.pbest):.2f} Pos: {self.pbest}"
            )
            self.pbest = self.X.copy()

            if pbest_cost < gbest_cost:
                self.gbest = self.pbest.copy()
                self.get_logger().info(
                    f"GBest improved. Cost: {self.distance_to_target(self.gbest):.2f} Pos: {self.gbest}"
                )
                msg = Vector3Stamped(
                    header=Header(
                        frame_id=self.get_namespace(),
                        stamp=self.get_clock().now().to_msg(),
                    ),
                    vector=Vector3(x=self.gbest[0], y=self.gbest[1], z=0.0),
                )
                self.gbest_pub.publish(msg)

    def evaluate_velocity(self):
        self.last_X = self.X

        # If pbest or gbest is not set yet. Do not change speed.
        if self.gbest is not None and self.pbest is not None:
            # r1, r2 = np.random.rand(2)
            r1 = 1
            r2 = 1
            self.V = (
                self.pso_w * self.V
                + r1 * self.pso_c1 * (self.pbest - self.last_X)
                + r2 * self.pso_c2 * (self.gbest - self.last_X)
            )

            # Normalize velocity
            self.V = self.V / np.linalg.norm(self.V) * 0.5

    def move_to_new_position(self):
        self.command_publisher.publish(
            Twist(
                linear=Vector3(x=self.V[0], y=0.0, z=0.0),
                angular=Vector3(x=0.0, y=0.0, z=self.V[1]),
            )
        )

    def termination_condition(self) -> bool:
        return self.distance_to_target(self.X) < 0.25

    def pso_interation(self):
        if self.termination_condition():
            self.get_logger().info(
                f"Target reached. Cost: {self.distance_to_target(self.gbest):.2f} Pos: {self.gbest}"
            )
            self.pso_timer.cancel()
            self.swarm_command_pub.publish(Int8Message(data=0))
            self.swarm_command_pub.publish(Int8Message(data=0))

        self.evaluate_velocity()

        self.move_to_new_position()

        self.evalute_bests()

        self.get_logger().info(
            f"P: {self.X} | V: {self.V} | PBest: {self.pbest} | GBest: {self.gbest}"
        )

    def destroy_node(self):
        self.command_publisher.publish(Twist())
        super().destroy_node()


def main(args=None):
    rclpy.init(args=args)
    pattern_node = PSOAdaptative("pso_adaptative")
    rclpy.spin(pattern_node)
    pattern_node.destroy_node()
    rclpy.shutdown()


if __name__ == "__main__":
    main()
