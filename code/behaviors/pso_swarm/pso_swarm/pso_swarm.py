import rclpy
import functools
import numpy as np
import math

from rclpy.node import Node
from geometry_msgs.msg import Pose, Twist, Vector3
import rclpy.qos
from std_msgs.msg import Float32
from enum import Enum
from typing import Tuple

SEED = 50
np.random.seed(SEED)

# Position type
PosType = Tuple[float, float]

# Velocity type
VelType = Tuple[float, float]

# Memory type
MemType = Tuple[PosType, float]

class MemoryIndex(Enum):
    TAL_INDEX = 0
    COST_INDEX = 1

class UltrasonicIndex(Enum):
    FRONT = 0
    LEFT = 1
    RIGHT = 2

class Robot(Node):
    X = np.array([0, 0])
    pbest = np.array([0, 0])
    gbest = np.array([0, 0])

    ultrasonics = [0, 0, 0]
    target_pos = [1, 0]

    def __init__(self) -> None:
        super().__init__('pso_swarm')
        self.get_logger().info('Robot started')
        self.namespace = "/sim" # self.get_namespace()

        self.setup_parameters()

        self.initialize_classic_pso_variables()

        self.initialize_aor_variables()

        self.initialize_mem_tool_variables()

        self.setup_pub_subs()
        
        self.verbose = self.get_parameter('verbose').get_parameter_value().bool_value
        self.speed_limit = self.get_parameter('speed_limit').get_parameter_value().double_value

        self.timer = self.create_timer(0.1, self.run)

    # ROS FUNCTIONS
    def setup_pub_subs(self):
        self.create_subscription(
            Float32,
            f'{self.namespace}/ultrasonic/left',
            functools.partial(self.ultrasonic_callback, index=UltrasonicIndex.LEFT),
            2
        )

        self.create_subscription(
            Float32,
            f'{self.namespace}/ultrasonic/right',
            functools.partial(self.ultrasonic_callback, index=UltrasonicIndex.RIGHT),
            2
        )

        self.create_subscription(
            Float32,
            f'{self.namespace}/ultrasonic/front',
            functools.partial(self.ultrasonic_callback, index=UltrasonicIndex.FRONT),
            2
        )

        self.create_subscription(
            Pose,
            f'{self.namespace}/pose',
            self.pose_callback,
            10
        )

        self.cmd_vel_pub = self.create_publisher(
            Twist,
            f'{self.namespace}/cmd_vel',
            2
        )

        qos = rclpy.qos.QoSProfile(
            history=rclpy.qos.QoSHistoryPolicy.KEEP_LAST,
            depth=1,
            avoid_ros_namespace_conventions=True
        )
        
        self.gbest_pub = self.create_publisher(
            Vector3,
            'gbest',
            qos
        )
        
        self.create_subscription(
            Vector3,
            'gbest',
            self.gbest_callback,
            qos
        )

    def pose_callback(self, msg: Pose):
        self.X = (msg.position.x, msg.position.y)

    def ultrasonic_callback(self, msg: Float32, index: UltrasonicIndex):
        self.ultrasonics[index.value] = msg.data

    def setup_parameters(self):
        # Classic PSO parameters
        self.declare_parameter('c1', 0.5)
        self.declare_parameter('c2', 0.5)

        # Alternative PSO parameters (AoR)
        self.declare_parameter('k', 0.01)
        self.declare_parameter('p', 0.9)
        self.declare_parameter('trr', 0.1)

        # Alternative PSO parameters (Memory)
        self.declare_parameter('mms', 10)
        self.declare_parameter('mpr', 0.1)
        self.declare_parameter('md', 0.1)
        self.declare_parameter('mpp1', 0.1)
        self.declare_parameter('mpp2', 0.1)
        self.declare_parameter('FFthr', 0.0)

        # Runtime parameters
        self.declare_parameter('verbose', False)
        self.declare_parameter('speed_limit', 0.2)

    # TODO: Make a PID here
    def calculate_publish_velocity(self, velocity: VelType) -> None:
        # polar = self.euclidean_to_polar(velocity[0], velocity[1])

        if np.linalg.norm(velocity) > self.speed_limit:
            velocity = velocity / np.linalg.norm(velocity) * self.speed_limit

        cmd_vel = Twist()

        cmd_vel.linear.x = float(velocity[0])
        cmd_vel.angular.z = float(velocity[1])

        self.cmd_vel_pub.publish(cmd_vel)

    def gbest_callback(self, msg: Vector3):
        self.gbest = np.array([msg.x, msg.y])

    # ALTERNATIVE PSO FUNCTIONS
    def initialize_classic_pso_variables(self):
        self.c1 = self.get_parameter('c1').get_parameter_value().double_value
        self.c2 = self.get_parameter('c2').get_parameter_value().double_value
        # TODO: Make cost function configurable somehow

        # TODO: Think about how to initialize position X randomly and sync with CoppeliaSim
        self.V = np.random.rand(2) * 0.2

    def initialize_aor_variables(self):
        # Constants
        self.k = self.get_parameter('k').get_parameter_value().double_value
        self.p = self.get_parameter('p').get_parameter_value().double_value
        # Trr -> Temperature reduction rate
        self.Trr = self.get_parameter('trr').get_parameter_value().double_value

        # theta -> Angle of rotation (AoR)
        self.theta = 0.0 
        # s -> Dynamic change step for AoR
        self.s = 1 if np.random.rand() > 0.5 else -1   
        # dc -> Probability of changing s each interaction
        self.dc = 0.0
        # temperature -> Prevent theta decreasing too quickly
        self.temperature = 0.0

    def initialize_mem_tool_variables(self):
        self.mem: list[MemType] = []
        # Maximum Memory Size
        self.mms = self.get_parameter('mms').get_parameter_value().integer_value
        # Memory Point Radius
        self.mpr = self.get_parameter('mpr').get_parameter_value().double_value
        # Minimum Displacement
        self.md = self.get_parameter('md').get_parameter_value().double_value
        # Memory Point Penalty 1
        self.mpp1 = self.get_parameter('mpp1').get_parameter_value().double_value
        # Memory Point Penalty 2
        self.mpp2 = self.get_parameter('mpp2').get_parameter_value().double_value
        # Threshold
        self.FFthr = self.get_parameter('FFthr').get_parameter_value().double_value

    def update_theta(self):
        # Update velocity using AoR concept [7]
        RV = [[np.cos(self.theta), -np.sin(self.theta)], [np.sin(self.theta), np.cos(self.theta)]]
        RV = np.matmul(RV, self.V)

        # Update theta with temperature [8]
        self.theta = self.temperature * self.theta

        # Update temperature [9]
        self.temperature = self.Trr * self.temperature

        return RV
    
    def evaluate_position(self) -> None:
        # Reduce ν for τ near to Pit1 inside the Memoryit by MPP2  
        if len(self.mem) > 0 and self.md > self.euclidean_distance(self.X, self.mem[-1][MemoryIndex.TAL_INDEX.value]):
            self.mem[-1][MemoryIndex.COST_INDEX.value] -= self.mpp2
    
    def update_memory(self) -> None:
        current_position = self.pbest
        
        # If a particle memory is empty, then start it
        if 0 == len(self.mem):
            self.mem.append([current_position, self.cost_function(current_position)])
            return
        
        # For each memory registered in the memory of a particle
        for i in range(len(self.mem)):

            # Take the τ and ν from the memory
            tal, val = self.mem[i]
            
            # if Pbest distance to any τ in the memory is less than MPR
            distance = self.euclidean_distance(current_position, tal)
            if distance < self.mpr:
                # Reduce corresponding ν inside the Memory by MPP1
                self.mem[i][1] = val - self.mpp1

                # Select the pair with maximun ν from the memory and set its τ as Pit
                # TODO: Test with minimum value.
                best_cost_index = np.argmax(self.mem[i][:][1])
                self.mem[best_cost_index][MemoryIndex.TAL_INDEX.value] = current_position
            else:
                # if size of Memory i > MMS then 
                if len(self.mem) > self.mms:
                    # Remove the oldest Memory i member
                    self.mem.pop(0)

                # Insert new xi into Memory i
                self.mem.append([current_position, self.cost_function(current_position)])

    def evaluate_bests(self) -> None:
        pbest_cost = self.cost_function(self.pbest)
        local_cost = self.cost_function(self.X)

        # Current x has better fitness than pbest
        if local_cost < pbest_cost:
            # Update the set of Personal-Bests it with P it
            self.pbest = self.X.copy()
            # print(f"Personal best improved. Cost: {self.cost_function(self.pbest):.2f} Pos: {self.pbest}")

            self.update_memory()
         
            # Select the best element of Personal-Bests it as G i
            global_cost = self.cost_function(self.gbest)
            if pbest_cost < global_cost:
                # if self.verbose:
                self.get_logger().info(f"Global best improved. Cost: {self.cost_function(self.gbest):.2f} Pos: {self.gbest}")
                self.gbest = self.pbest
                self.gbest_pub.publish(Vector3(x=self.gbest[0], y=self.gbest[1], z=0.0))

    def update_velocity(self, n1=1.0, n2=1.0) -> None:
        # Randomly set r1 and r2
        r1 = np.random.rand()
        r2 = np.random.rand()

        # Update velocity as classic PSO [2]
        self.V = self.V + self.c1 * n1 * r1 * (self.pbest - self.X) + self.c2 * n2* r2 * (self.gbest - self.X)

        # Limit velocity between arbitrary limits
        if np.abs(self.V[0]) > self.speed_limit:
            self.V[0] = np.sign(self.V[0])
        if np.abs(self.V[1]) > self.speed_limit:
            self.V[1] = np.sign(self.V[1])

    # TODO: Make new colision function
    def check_for_collision(self) -> None:
        if self.ultrasonics[UltrasonicIndex.FRONT.value] < 0.5:
            return True

    def update_movement(self, RV=(0.0, 0.0)) -> None:
        # next_pos = self.X + RV

        if self.check_for_collision():

            # Change theta [4]
            self.theta += self.s

            # Set temperature to maximun
            self.temperature = 1.0

            # Increase Dc [5]
            self.dc += self.k

            # Randomly modify s and dc (Maybe add a scaller here)
            if np.random.rand() < self.dc:
                self.s *= -1
                self.dc = 0.0

            RV = (0.0, 0.0) # --> next_pos = self.X
        else:
            # Decrease dc [6]
            self.dc *= self.p

        self.calculate_publish_velocity(RV) # --> self.X = next_pos

    def euclidean_distance(self, p1: PosType, p2: PosType) -> float:
        return np.sqrt((p1[0] - p2[0])**2 + (p1[1] - p2[1])**2)
    
    def euclidean_to_polar(self, x, y):
        r = math.sqrt(x**2 + y**2)
        theta = math.atan2(y, x)
        return [r, theta]

    def cost_function(self, position: PosType) -> float:
        return self.euclidean_distance(self.target_pos, position)

    def log(self, RV = [0, 0]):
        self.get_logger().info(
            f"Pos: {self.X} Vel: {self.V} | {RV} \n\tPbest: {self.pbest} Gbest: {self.gbest} \n\tCost: {self.cost_function(self.X):.2f}"
        )

    def run(self):
        if self.cost_function(self.X) < 0.5:
            self.get_logger().info(f"Goal reached. Cost: {self.cost_function(self.X):.2f} Pos: {self.X}")
            self.calculate_publish_velocity((0.0, 0.0))
            self.timer.cancel()
            self.destroy_node()
            return
        
        RV = self.update_theta()
        self.evaluate_position()
        
        n1 = 1 if self.cost_function(self.pbest) > self.FFthr else 0
        n2 = 1 if self.cost_function(self.gbest) > self.FFthr else 0

        if n1 == 0 and n2 == 0:
            self.mem = []
        else:
            self.update_velocity(n1, n2)

        self.log(RV)

        self.update_movement(RV)
        self.calculate_publish_velocity(self.V)

def main():
    rclpy.init()
    node = Robot()
    rclpy.spin(node)
    rclpy.shutdown()

if __name__ == '__main__':
    main()
