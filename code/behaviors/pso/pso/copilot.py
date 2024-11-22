import rclpy
import random
import numpy as np

from rclpy.node import Node
from geometry_msgs.msg import Twist, Pose, PoseStamped
from sensor_msgs.msg import Range
from collections import deque


class PSOAlgorithm(Node):
    def __init__(self):
        super().__init__("pso_algorithm")

        # Parâmetros do PSO
        self.declare_parameter("c1", 1.5)
        self.declare_parameter("c2", 1.5)
        self.declare_parameter("w", 0.5)
        self.declare_parameter("max_speed", 0.5)

        self.c1 = self.get_parameter("c1").get_parameter_value().double_value
        self.c2 = self.get_parameter("c2").get_parameter_value().double_value
        self.w = self.get_parameter("w").get_parameter_value().double_value
        self.max_speed = (
            self.get_parameter("max_speed").get_parameter_value().double_value
        )

        self.namespace = self.get_namespace()

        self.publisher_ = self.create_publisher(Twist, f"{self.namespace}/cmd_vel", 10)
        self.subscription_pose = self.create_subscription(
            Pose, f"{self.namespace}/pose", self.pose_callback, 10
        )
        self.subscription_front = self.create_subscription(
            Range, f"{self.namespace}/us_front", self.front_sensor_callback, 10
        )
        self.subscription_left = self.create_subscription(
            Range, f"{self.namespace}/us_left", self.left_sensor_callback, 10
        )
        self.subscription_right = self.create_subscription(
            Range, f"{self.namespace}/us_right", self.right_sensor_callback, 10
        )

        # Publisher e Subscriber para gbest
        self.gbest_publisher = self.create_publisher(PoseStamped, "/global_best", 10)
        self.gbest_subscription = self.create_subscription(
            PoseStamped, "/global_best", self.gbest_callback, 10
        )

        self.cmd = Twist()
        self.pbest = (
            np.random.rand(2) * 2 - 1
        )  # Exemplo inicial da melhor posição da partícula
        self.gbest = (
            np.random.rand(2) * 2 - 1
        )  # Exemplo inicial da melhor posição global
        self.current_position = np.zeros(2)  # Posição atual do robô
        self.velocity = np.random.rand(2)  # Velocidade inicial aleatória

        # self.position_queue = deque(maxlen=5)
        self.velocity_queue = deque(maxlen=5)

        self.target_position = np.array([0.0, 1.0])
        self.termination_distance = 0.2

        # Flag para detectar obstáculos
        self.obstacle_detected = False

        # Flags para os sensores laterais
        self.left_obstacle_detected = False
        self.right_obstacle_detected = False

        self.timer = self.create_timer(0.1, self.update_position)
        self.log_timer = self.create_timer(0.5, self.log_position)

    def log_position(self):
        pos_str = f"P: {self.current_position[0]:.2f}, {self.current_position[1]:.2f}, {self.evaluate_fitness(self.current_position):.2f}"
        pbest_str = f"pbest: {self.pbest[0]:.2f}, {self.pbest[1]:.2f}, {self.evaluate_fitness(self.pbest):.2f}"
        gbest_str = f"gbest: {self.gbest[0]:.2f}, {self.gbest[1]:.2f}, {self.evaluate_fitness(self.gbest):.2f}"
        self.get_logger().info(f"{pos_str} | {pbest_str} | {gbest_str}")

    def is_target_reached(self, position=None):
        if position is None:
            position = self.current_position
        return self.evaluate_fitness(position) < self.termination_distance

    def pose_callback(self, msg: Pose):
        # Atualizar a posição atual do robô com os dados do simulador
        # new_position = np.array([msg.position.x, msg.position.y])
        # self.position_queue.append(new_position)

        # # Calcular a posição suavidada
        # self.current_position: np.ndarray = np.mean(self.position_queue, axis=0)

        self.current_position[0] = msg.position.x
        self.current_position[1] = msg.position.y

    def evaluate_bests(self):
        # Avaliação da Aptidão
        fitness = self.evaluate_fitness(self.current_position)

        # Atualizar pbest
        if fitness < self.evaluate_fitness(self.pbest):
            self.pbest = self.current_position.copy()

        # Atualizar gbest e publicar
        if fitness < self.evaluate_fitness(self.gbest):
            self.gbest = self.current_position.copy()
            self.publish_gbest()

    def evaluate_fitness(self, position):
        # Função de avaliação de aptidão
        return np.linalg.norm(position - self.target_position)

    def front_sensor_callback(self, msg: Range):
        # Lógica do sensor frontal
        self.obstacle_detected = msg.range < 0.5 and msg.range > 0
        if self.obstacle_detected:
            self.cmd.angular.z = 1.0 * np.sign(self.gbest[1] - self.current_position[1])
            self.cmd.linear.x = 0.0

        self.publisher_.publish(self.cmd)

    def left_sensor_callback(self, msg):
        # Lógica do sensor lateral esquerdo
        self.left_obstacle_detected = msg.range < 0.4 and msg.range > 0
        if self.obstacle_detected:
            self.cmd.angular.z = -0.5

    def right_sensor_callback(self, msg):
        # Lógica do sensor lateral direito
        self.right_obstacle_detected = msg.range < 0.4 and msg.range > 0
        if self.obstacle_detected:
            self.cmd.angular.z = 0.5

    def gbest_callback(self, msg: PoseStamped):
        # Atualizar gbest apenas se o frame_id não for do próprio robô
        if msg.header.frame_id != self.namespace:
            self.gbest[0] = msg.pose.position.x
            self.gbest[1] = msg.pose.position.y

    def publish_gbest(self):
        gbest_msg = PoseStamped()
        gbest_msg.header.frame_id = self.namespace
        gbest_msg.pose.position.x = self.gbest[0]
        gbest_msg.pose.position.y = self.gbest[1]
        self.gbest_publisher.publish(gbest_msg)

    def update_position(self):
        # Checar condição de térmno
        if self.is_target_reached():
            self.get_logger().info(f"Robô {self.namespace} achou o alvo!")
            self.publisher_.publish(Twist())
            rclpy.shutdown()

        if (
            not self.obstacle_detected
            and not self.left_obstacle_detected
            and not self.right_obstacle_detected
        ):
            dt = 0.1
            # Lógica de atualização da posição com base no PSO
            r1, r2 = np.random.rand(2)
            new_velocity = (
                self.w * self.velocity
                + self.c1 * r1 * (self.pbest - self.current_position)
                + self.c2 * r2 * (self.gbest - self.current_position)
            )
            self.velocity_queue.append(new_velocity)
            self.velocity = np.mean(self.velocity_queue, axis=0)

            self.current_position += self.velocity * dt

            # Limitar a velocidade ao valor máximo
            speed = np.linalg.norm(self.velocity)
            if speed > self.max_speed:
                self.velocity = self.velocity / speed * self.max_speed

            # Atualizar a velocidade do robô
            self.cmd.linear.x = self.velocity[0]
            self.cmd.angular.z = self.velocity[1]

            self.evaluate_bests()

        self.publisher_.publish(self.cmd)


def main(args=None):
    rclpy.init(args=args)
    pso_algorithm = PSOAlgorithm()
    rclpy.spin(pso_algorithm)
    pso_algorithm.destroy_node()
    rclpy.shutdown()


if __name__ == "__main__":
    main()
