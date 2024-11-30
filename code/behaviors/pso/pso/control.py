import rclpy
import random
import numpy as np
import math

from rclpy.node import Node
from std_msgs.msg import Bool
from geometry_msgs.msg import Twist, Pose, PoseStamped, Quaternion
from sensor_msgs.msg import Range

np.set_printoptions(precision=2)


class LoggerCSV:
    def __init__(self, name):
        self.name = name
        with open(self.name, "w") as f:
            f.write("yaw,pose_x,pose_y,target_x,target_y,kP,kI,kD,e,e_P,e_I,e_D,w,v\n")

    def log_pid(self, yaw, pose, target, kP, kI, kD, e, e_P, e_I, e_D, w, v):
        with open(self.name, "a") as f:
            f.write(
                f"{yaw:.2f},{pose[0]:.2f},{pose[1]:.2f},{target[0]:.2f},{target[1]:.2f},{kP:.2f},{kI:.2f},{kD:.2f},{e:.2f},{e_P:.2f},{e_I:.2f},{e_D:.2f},{w:.2f},{v:.2f}\n"
            )


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
        self.csv = LoggerCSV("log.csv")

    def setPID(self, kP, kI, kD, dT=None, v=None):
        self.Kp = kP
        self.Ki = kI
        self.Kd = kD
        if dT is not None:
            self.dt = dT

        if v is not None:
            self.desiredV = v

    def iteratePID(self, current, goal, orientation):
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

        e = math.atan2(math.cos(alpha), math.sin(alpha))

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

        w = -np.arctan2(np.sin(w), np.cos(w))

        self.E = self.E + e
        self.old_e = e
        v = self.desiredV * np.cos(abs(e))
        if v < 0.01:
            v = 0.01

        self.csv.log_pid(
            yaw, current, goal, self.Kp, self.Ki, self.Kd, e, e_P, e_I, e_D, w, v
        )

        return v, w


class PSOAlgorithm(Node):
    def __init__(self):
        super().__init__("pso_algorithm")
        self.dt = 0.05

        self.setup_parameters()

        self.namespace = self.get_namespace()

        self.setup_pub_sub()

        # Variáveis de controle
        self.orientation = Quaternion()
        self.cmd = Twist()
        self.pbest = np.random.rand(2) * 2
        self.gbest = np.random.rand(2) * 2
        self.current_position = np.zeros(2)
        self.velocity = np.zeros(2)  # TODO: Check to be random

        self.map_offset = 5
        self.termination_distance = 0.1

        # Flag para detectar obstáculos
        self.obstacle_detected = False

        # Flags para os sensores laterais
        self.left_obstacle_detected = False
        self.right_obstacle_detected = False

        # Variáveis Memory Tool

        self.controller = Controller(dT=self.dt)
        self.update_params()

        self.start_sim = self.create_publisher(Bool, "/startSimulation", 1)
        self.pause_sim = self.create_publisher(Bool, "/pauseSimulation", 1)

        self.timer = self.create_timer(self.dt, self.update_position)
        self.timer.cancel()
        self.params_timer = self.create_timer(self.dt * 3, self.update_params)

    def setup_parameters(self):
        self.declare_parameters(
            namespace="",
            parameters=[
                ("c1", 1.5),
                ("c2", 1.5),
                ("w", 0.5),
                ("k", 0.01),
                ("p", 0.9),
                ("TRR", 0.9),
                ("max_speed", 0.2),
                ("target", [1.0, 1.0]),
                ("kP", 1.0),
                ("kI", 0.01),
                ("kD", 0.01),
            ],
        )

    def setup_pub_sub(self):
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

    def setup_pso_variables(self):
        # Classic PSO variables
        self.c1 = self.get_parameter("c1").get_parameter_value().double_value
        self.c2 = self.get_parameter("c2").get_parameter_value().double_value
        self.w = self.get_parameter("w").get_parameter_value().double_value

        # Angle of Rotation (AoR) variables
        self.k = self.get_parameter_or("k", 0.01).get_parameter_value().double_value
        self.p = self.get_parameter_or("p", 0.9).get_parameter_value().double_value
        self.TRR = (
            self.get_parameter_or("TRR", 0.9).get_parameter_value().double_value
        )  # Taxa de redução da temperatura
        self.theta = 0.0  # Angulo de rotação
        s_value = 0.5
        self.s = random.choice([-s_value, s_value])  # Direção de rotação dinâmica
        self.Dc = 0.0  # Probabilidade de mudança de s
        self.T = 0.0  # Temperatura inicial

        # Memory Tool (Mem Tool) variables
        self.mem = []
        self.mms = 10  # Maximum Memory Size
        self.mpr = 0.1  # Memory Point Radius
        self.md = 0.1  # Minimum Displacement
        self.mpp1 = 0.1  # Memory Point Penalty 1
        self.mpp2 = 0.1  # Memory Point Penalty 2
        self.FFthr = 0.0  # Threshold

    def update_params(self):
        # Parâmetros do PSO
        self.setup_pso_variables()

        # Parâmetros do PID
        self.max_speed = (
            self.get_parameter("max_speed").get_parameter_value().double_value
        )
        kP = self.get_parameter("kP").get_parameter_value().double_value
        kI = self.get_parameter("kI").get_parameter_value().double_value
        kD = self.get_parameter("kD").get_parameter_value().double_value
        self.controller.setPID(kP, kI, kD, v=self.max_speed)

        # Emulação de fonte de sinal
        self.target_position = (
            np.array(
                self.get_parameter("target").get_parameter_value().double_array_value
            )
            + self.map_offset
        )

    def is_target_reached(self, position=None):
        if position is None:
            position = self.current_position
        return self.evaluate_fitness(position) < self.termination_distance

    def pose_callback(self, msg: Pose):
        # Atualizar a posição atual do robô com os dados do simulador
        self.current_position[0] = msg.position.x + self.map_offset
        self.current_position[1] = msg.position.y + self.map_offset

        self.orientation = msg.orientation

        if self.timer.is_canceled():
            self.get_logger().info("PSOAdaptative TIMER started...")
            self.timer.reset()

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
        self.obstacle_detected = msg.range < 0.3 and msg.range > 0

    def left_sensor_callback(self, msg: Range):
        # Lógica do sensor lateral esquerdo
        self.left_obstacle_detected = msg.range < 0.2 and msg.range > 0

    def right_sensor_callback(self, msg: Range):
        # Lógica do sensor lateral direito
        self.right_obstacle_detected = msg.range < 0.2 and msg.range > 0

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
        # self.pause_sim.publish(Bool(data=True))
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

        self.theta = self.T * self.theta
        self.T = self.TRR * self.T

        # Atualizar a posição e velocidade da simulação
        desired_position = self.current_position + RV * self.dt

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
                self.get_logger().info("Change turn direction!")
                self.s *= -1
                self.Dc = 0
        else:
            self.Dc = self.Dc * self.p

        linear_speed, angular_speed = self.controller.iteratePID(
            self.current_position,
            self.target_position,
            # desired_position,
            self.orientation,
        )

        # Linear speed fixed according to the Temperature, which means that the robot found a obstacle.
        fixed_linear_speed = linear_speed - self.max_speed * self.T

        # Normalized angular speed to [-max_speed, max_speed]
        norm_angular_speed = angular_speed

        # Sides obstacle avoidance
        if self.left_obstacle_detected:
            norm_angular_speed += -0.001
        if self.right_obstacle_detected:
            norm_angular_speed += 0.001

        dx = self.target_position[0] - self.current_position[0]
        dy = self.target_position[1] - self.current_position[1]
        angle = np.arctan2(dy, dx)

        self.get_logger().info(
            f"fix_v: {fixed_linear_speed:.2f}, norm_w: {norm_angular_speed:.2f} | pos: {self.current_position} | target: {self.target_position} | dx: {dx:.2f}, dy: {dy:.2f}, angle: {angle:.2f}"
        )

        self.cmd.linear.x = fixed_linear_speed
        self.cmd.angular.z = norm_angular_speed

        self.publisher_.publish(self.cmd)
        # self.start_sim.publish(Bool(data=True))


def main(args=None):
    rclpy.init(args=args)
    pso_algorithm = PSOAlgorithm()
    rclpy.spin(pso_algorithm)
    pso_algorithm.destroy_node()
    rclpy.shutdown()


if __name__ == "__main__":
    main()
