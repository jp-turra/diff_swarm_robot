import rclpy
import random
import numpy as np
import math

from rclpy.node import Node
from geometry_msgs.msg import Twist, Pose, PoseStamped, Quaternion
from sensor_msgs.msg import Range

np.set_printoptions(precision=2)


class Controller:
    def __init__(
        self,
        kP=1.0,
        kI=0.01,
        kD=0.01,
        dT=0.1,
        v=1.0,
    ):
        self.E = 0  # Cummulative error
        self.old_e = 0  # Previous error

        self.Kp = kP
        self.Ki = kI
        self.Kd = kD

        self.desiredV = v
        self.dt = dT  # in second
        return

    def iteratePID(self, current, goal, orientation, logger):
        # Difference in x and y
        d_x = goal[0] - current[0]
        d_y = goal[1] - current[1]

        # Angle from robot to goal
        g_theta = np.arctan2(d_y, d_x)

        # Error between the goal angle and robot angle
        siny_cosp = 2.0 * (
            orientation.w * orientation.z + orientation.x * orientation.y
        )
        cosy_cosp = 1.0 - 2.0 * (
            orientation.y * orientation.y + orientation.z * orientation.z
        )

        yaw = math.atan2(siny_cosp, cosy_cosp)
        alpha = g_theta - yaw

        # alpha = g_theta - np.pi / 2
        e = np.arctan2(np.sin(alpha), np.cos(alpha))

        e_P = e
        e_I = self.E + e
        e_D = e - self.old_e
        # Anti-Windup
        if e_I > 2 * np.pi:
            e_I = 2 * np.pi
        elif e_I < -2 * np.pi:
            e_I = -2 * np.pi

        # This PID controller only calculates the angular
        # velocity with constant speed of v
        w = self.Kp * e_P + self.Ki * e_I + self.Kd * e_D

        w = np.arctan2(np.sin(w), np.cos(w))

        self.E = self.E + e
        self.old_e = e
        v = self.desiredV

        logger.info(
            f"alpha: {alpha:.2f} | e_P: {e_P:.2f} | e_I: {e_I:.2f} | e_D: {e_D:.2f} | w: {w:.2f} | v: {v:.2f}"
        )

        return v, w


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

        self.orientation = Quaternion()
        self.cmd = Twist()
        self.pbest = (
            np.random.rand(2) * 2
        )  # Exemplo inicial da melhor posição da partícula
        self.gbest = np.random.rand(2) * 2  # Exemplo inicial da melhor posição global
        self.current_position = np.zeros(2)  # Posição atual do robô
        self.velocity = np.zeros(2)  # Velocidade inicial aleatória

        # self.velocity_queue = deque(maxlen=5)

        self.target_position = np.array([5.0, 6.0])
        self.termination_distance = 0.4

        # Flag para detectar obstáculos
        self.obstacle_detected = False

        # Flags para os sensores laterais
        self.left_obstacle_detected = False
        self.right_obstacle_detected = False

        # Variáveis para AoR
        self.theta = 0.0  # Angulo de rotação
        self.s = random.choice([-1, 1])  # Direção de rotação dinâmica
        self.Dc = 0.0  # Probabilidade de mudança de s
        self.T = 0.0  # Temperatura inicial
        self.TRR = 0.9  # Taxa de redução da temperatura
        self.k = 0.01
        self.p = 0.9

        self.timer = self.create_timer(0.1, self.update_position)
        # self.log_timer = self.create_timer(0.5, self.log_position)

        self.controller = Controller(
            kP=100.0, kI=0.01, kD=0.0001, dT=0.1, v=self.max_speed
        )

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
        self.current_position[0] = msg.position.x + 5
        self.current_position[1] = msg.position.y + 5

        self.orientation = msg.orientation

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

    def left_sensor_callback(self, msg: Range):
        # Lógica do sensor lateral esquerdo
        self.left_obstacle_detected = msg.range < 0.4 and msg.range > 0
        if self.obstacle_detected:
            self.velocity[1] += -0.01

    def right_sensor_callback(self, msg: Range):
        # Lógica do sensor lateral direito
        self.right_obstacle_detected = msg.range < 0.4 and msg.range > 0
        if self.obstacle_detected:
            self.velocity[0] -= 0.01

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
            self.get_logger().info(
                f"P: {self.current_position[0]:.2f}, {self.current_position[1]:.2f} | PBest: {self.pbest[0]:.2f}, {self.pbest[1]:.2f} | GBest: {self.gbest[0]:.2f}, {self.gbest[1]:.2f}"
            )
            self.get_logger().info(
                f"Custo p: {self.evaluate_fitness(self.current_position):.2f} | Custo pbest: {self.evaluate_fitness(self.pbest):.2f} | Custo gbest: {self.evaluate_fitness(self.gbest):.2f}"
            )
            self.publisher_.publish(Twist())
            rclpy.shutdown()

        # Atualizar velocidade
        r1, r2 = np.random.rand(2)
        self.velocity = (
            self.w * self.velocity
            + self.c1 * r1 * (self.pbest - self.current_position)
            + self.c2 * r2 * (self.gbest - self.current_position)
        )

        # Limitar a velocidade ao valor máximo
        speed = np.linalg.norm(self.velocity)
        if speed > self.max_speed:
            self.velocity = self.velocity * self.max_speed / speed

        # atualizar AoR
        RV = [
            [np.cos(self.theta), -np.sin(self.theta)],
            [np.sin(self.theta), np.cos(self.theta)],
        ]
        RV = np.matmul(RV, self.velocity)
        # self.velocity_queue.append(RV)
        # RV = np.mean(self.velocity_queue, axis=0)

        self.theta = self.T * self.theta
        self.T = self.TRR * self.T

        # Atualizar a posição e velocidade da simulação
        desired_position = self.current_position + RV

        # Evaluate best
        self.evaluate_bests()

        # Checar obstáculos
        if (
            self.obstacle_detected
            # or self.left_obstacle_detected
            # or self.right_obstacle_detected
        ):
            self.theta = self.theta + self.s
            self.T = 1
            self.Dc = self.Dc + self.k
            if self.Dc > np.random.rand():
                self.s *= -1
                self.Dc = 0
        else:
            self.Dc = self.Dc * self.p

        linear_speed, angular_speed = self.controller.iteratePID(
            self.current_position,
            desired_position,
            # self.target_position,
            self.orientation,
            self.get_logger(),
        )
        # self.get_logger().info(
        #     f"dx: {linear_speed:.2f} | dy: {angular_speed:.2f} | dp: {self.target_position} | cp: {self.current_position} | object: {self.obstacle_detected}"
        # )

        if self.obstacle_detected:
            self.cmd.linear.x -= 0.1
            self.cmd.angular.z += np.sign(angular_speed) * 0.1
        else:
            self.cmd.linear.x = linear_speed
            self.cmd.angular.z = angular_speed / np.pi * 0.5

        self.publisher_.publish(self.cmd)


def main(args=None):
    rclpy.init(args=args)
    pso_algorithm = PSOAlgorithm()
    rclpy.spin(pso_algorithm)
    pso_algorithm.destroy_node()
    rclpy.shutdown()


if __name__ == "__main__":
    main()
