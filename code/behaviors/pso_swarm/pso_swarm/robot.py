import rclpy
import functools
import numpy as np
import math

from rclpy.node import Node
from rclpy.qos import QoSProfile
from geometry_msgs.msg import Pose, Twist, Vector3
from std_msgs.msg import Float32
from typing import List, Callable

Vector2 = List[float]

class PSO():
    block_pose = True
    def __init__(self, cost_function: Callable, notify_gbest: Callable=None, c1 = 0.5, c2 = 0.5, w=0.9, X = [0, 0]) -> None:
        self.c1 = c1
        self.c2 = c2
        self.w = w
        self.X = np.array(X)
        self.pbest = np.array(X)
        self.local_gbest = np.array(X)
        self.cost_function = cost_function
        self.notify_gbest = notify_gbest
        self.V = np.zeros(2)

    def update_pbest(self) -> None:
        self.pbest = np.where(self.cost_function(self.X) < self.cost_function(self.pbest), self.X, self.pbest)
        if self.cost_function(self.pbest) < self.cost_function(self.local_gbest):
            if self.notify_gbest is not None:
                self.notify_gbest(self.pbest)
            self.local_gbest = self.pbest

    def update_gbest(self, incoming_gbest) -> None:
        print("Should not be here")
        self.local_gbest = np.where(self.cost_function(incoming_gbest) < self.cost_function(self.local_gbest), incoming_gbest, self.local_gbest)

    def update_position(self, listen_pose: Vector2 = None) -> None:
        if listen_pose is not None:
            self.X = np.array(listen_pose)
        else:
            self.X = self.X + self.V

    def update_velocity(self) -> None:
        r1, r2 = np.random.rand(2)
        self.V = self.w * self.V + r1 * self.c1 * (self.pbest - self.X) + r2 * self.c2 * (self.local_gbest - self.X)

    def interact(self) -> None:
        self.update_velocity()
        self.update_position()
        self.update_pbest()

class Robot(Node):
    target_position: Vector2 = (0, 2)

    def __init__(self):
        super().__init__('robot')
        self.pso: PSO = None
        self.declare_parameter("robot_id", 15)

        self.robot_id = self.get_parameter("robot_id").get_parameter_value().integer_value
        self.speed_limit = 0.3
        
        self.create_subscription(
            Pose,
            f'/r{self.robot_id}/pose',
            self.pose_callback,
            1
        )

        qos = rclpy.qos.QoSProfile(
            history=rclpy.qos.QoSHistoryPolicy.KEEP_LAST,
            depth=1,
            avoid_ros_namespace_conventions=True
        )
        self.create_subscription(
            Vector3, 
            '/gbest', 
            self.gbest_callback, 
            qos
        )

        self.cmd_pub = self.create_publisher(Twist, f'/r{self.robot_id}/cmd_vel', 1)
        self.gbest_pub = self.create_publisher(Vector3, f'/gbest', qos)

        self.timer = self.create_timer(0.5, self.run)
    
    def pose_callback(self, msg: Pose):
        self.X = (msg.position.x, msg.position.y)
        if not self.pso:
            self.get_logger().info(f"Robot {self.robot_id} initialized")
            self.pso = PSO(
                notify_gbest=self.notify_local_gbest, 
                cost_function=self.cost_function, 
                X=self.X,
                w=0.9
            )
        else:
            self.pso.update_position(listen_pose=self.X)

    def gbest_callback(self, msg: Vector3):
        pass
        # self.pso.update_gbest(incoming_gbest=(msg.x, msg.y))

    def notify_local_gbest(self, gbest: Vector2):
        self.gbest_pub.publish(Vector3(x=gbest[0], y=gbest[1], z=0.0))

    def publish_velocity(self, velocity: Vector2):
        cmd_vel = Twist()

        cmd_vel.linear.x = float(velocity[0])
        cmd_vel.angular.z = float(velocity[1])

        self.cmd_pub.publish(cmd_vel)
    
    def cost_function(self, X: Vector2) -> float:
        return math.sqrt((X[0] - self.target_position[0])**2 + (X[1] - self.target_position[1])**2)

    def run(self):
        if self.pso:
            if self.cost_function(self.pso.X) < 0.6:
                self.get_logger().info(f"Robot {self.robot_id} reached target")
                self.publish_velocity([0.0, 0.0])
                return

            self.pso.interact()
            self.publish_velocity(self.pso.V)
            self.get_logger().info(f"Robot {self.robot_id} position: {self.X} | Velocity: {self.pso.V}")
        else:
            self.get_logger().info(f"Robot {self.robot_id} not initialized")


def main():
    rclpy.init()
    node = Robot()
    rclpy.spin(node)
    rclpy.shutdown()

if __name__ == '__main__':
    main()