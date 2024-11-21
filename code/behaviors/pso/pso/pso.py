import rclpy
import numpy as np

from rclpy.node import Node
from rclpy.qos import qos_profile_sensor_data
from std_msgs.msg import Header
from geometry_msgs.msg import Vector3Stamped, Pose, Twist, Vector3


class PSO(Node):
    def __init__(self, name):
        super().__init__(name)
        self.setup_parameters()

        self.X: np.ndarray = None
        self.V = np.random.rand(2) * 0.2
        self.dt = 0.5
        self.pbest: np.ndarray = None
        self.gbest: np.ndarray = None
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

        self.setup_pub_suv()

        self.pso_timer = self.create_timer(self.dt, self.pso_interation)
        self.pso_timer.cancel()

    def setup_parameters(self):
        self.declare_parameters(
            namespace="",
            parameters=[
                ("pso_w", 1.0),  # Velocity smoother
                ("pso_c1", 0.5),
                ("pso_c2", 0.3),
                ("target", [1.0, 1.0]),
            ],
        )

    def setup_pub_suv(self):
        self.gbest_pub = self.create_publisher(Vector3Stamped, "/gbest", 2)
        self.gbest_sub = self.create_subscription(
            Vector3Stamped,
            "/gbest",
            self.gbest_callback,
            1,
        )
        self.pose_sub = self.create_subscription(
            Pose,
            self.get_namespace() + "/pose",
            self.pose_callback,
            qos_profile_sensor_data,
        )

        self.velocity_pub = self.create_publisher(
            Twist, self.get_namespace() + "/cmd_vel", 1
        )

    def gbest_callback(self, msg: Vector3Stamped):
        new_gbest = np.array([msg.vector.x, msg.vector.y])
        if msg.header.frame_id != self.get_namespace() and self.distance_to_target(
            self.gbest
        ) > self.distance_to_target(new_gbest):
            self.gbest = new_gbest.copy()
            self.get_logger().info(
                f"Global GBest improved. Cost: {self.distance_to_target(self.gbest):.2f} Pos: {self.gbest}"
            )

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

    def distance_to_target(self, pos: np.ndarray) -> float:
        return np.linalg.norm(pos - self.target)

    def termination_condition(self):
        return self.distance_to_target(self.X) < 0.3

    def evaluate_velocity(self):
        # If pbest or gbest is not set yet. Do not change speed.
        if self.gbest is not None and self.pbest is not None:
            r1, r2 = np.random.rand(2)
            self.V = (
                self.pso_w * self.V
                + r1 * self.pso_c1 * (self.pbest - self.X)
                + r2 * self.pso_c2 * (self.gbest - self.X)
            )

    def publish_velocity(self):
        x = self.V[0]  # / np.linalg.norm(self.V) * 1
        z = np.arctan2(self.V[1], self.V[0])  # / np.pi * 1

        self.velocity_pub.publish(
            Twist(
                linear=Vector3(x=x, y=0.0, z=0.0),
                angular=Vector3(x=0.0, y=0.0, z=z),
            )
        )

        self.get_logger().info(
            f"P: {self.X[0]:.2f}, {self.X[1]:.2f} | V: {x:.2f}, {z:.2f} | PBest: {self.pbest[0]:.2f}, {self.pbest[1]:.2f} | GBest: {self.gbest[0]:.2f}, {self.gbest[1]:.2f}"
        )

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

    def pso_interation(self):
        if self.termination_condition():
            self.get_logger().info(
                f"Target reached. Cost: {self.distance_to_target(self.gbest):.2f} Pos: {self.gbest}"
            )
            self.velocity_pub.publish(Twist())
            self.pso_timer.cancel()

        self.evaluate_velocity()

        self.publish_velocity()

        self.evalute_bests()


def main(args=None):
    rclpy.init(args=args)
    pattern_node = PSO("pso")
    rclpy.spin(pattern_node)
    pattern_node.destroy_node()
    rclpy.shutdown()


if __name__ == "__main__":
    main()
