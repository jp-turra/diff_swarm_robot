import numpy as np
import matplotlib.pyplot as plt
import matplotlib.animation as animation
from typing import Callable, Tuple

SEED = 50
np.random.seed(SEED)

# Position type
PosType = Tuple[float, float]

# Velocity type
VelType = Tuple[float, float]

# Memory type
MemType = Tuple[PosType, float]

# History type
HistType = Tuple[PosType, PosType, PosType, VelType]

MEM_TAL_INDEX = 0
MEM_COST_INDEX = 1

class PSO:
    def __init__(self, num_particles: int = 10, c1 = 0.5, c2 = 0.5, k: float = 0.01, p: float = 0.9, trr: float = 0.1,
                 mms: int = 10, mpr: float = 0.1, md: float = 0.1, mpp1: float = 0.1, mpp2: float = 0.1, FFthr: float = 0.0,
                 speed_limit: int = 1, speed_scaller: int = 1, spawn_point: PosType = (0, 0), spawn_radius: int = 1, 
                 cost_function: Callable = None, bubble_radius: int = 1, swarm_radius: int = 0.25, verbose=False) -> None:
        
        # Classic PSO parameters
        self.num_particles = num_particles
        self.cost_function = cost_function
        self.c1 = c1
        self.c2 = c2

        # Initializations
        self.initialize_positions(spawn_point=spawn_point, spawn_radius=spawn_radius)
        self.initialize_velocities(scaller=speed_scaller)

        # Alternative PSO parameters
        self.initialize_aor_variables(k, p, trr)
        self.initialize_mem_tool(mms, mpr, md, mpp1, mpp2, FFthr)

        # Global control variables
        cost = [self.cost_function(x) for x in self.X]
        self.gbest = self.pbest.copy()[np.argmin(cost)]

        # History for plotting
        self.initialize_history()

        # Simulation control variables
        self.speed_limit = speed_limit
        self.walls = []
        self.bubble_radius = bubble_radius
        self.swarm_radius = swarm_radius
        self.verbose = verbose

    def euclidean_distance(self, a: np.ndarray, b: np.ndarray) -> float:
        return np.sqrt((a[0] - b[0])**2 + (a[1] - b[1])**2)
    
    def get_history(self) -> HistType:
        return np.array(self.X_history), np.array(self.pbest_history), np.array(self.gbest_history), np.array(self.V_history)

    def initialize_history(self) -> None:
        self.X_history = []
        self.V_history = []
        self.pbest_history = []
        self.gbest_history = []

    def initialize_positions(self, spawn_point: PosType = (0, 0), spawn_radius: int = 1,) -> None:
        self.X = np.random.rand(self.num_particles, 2) * spawn_radius + spawn_point
        self.pbest = self.X.copy()

    def initialize_velocities(self, scaller: int = 1) -> None:
        self.V = np.random.rand(self.num_particles, 2) * scaller 

    def initialize_aor_variables(self, k: float = 0.01, p: float = 0.9, trr: float = 0.1) -> None:
        # theta -> Angle of rotation (AoR)
        self.theta = np.zeros(self.num_particles, dtype=np.float16) 
        # s -> Dynamic change step for AoR
        self.s = np.array([1 if np.random.rand() > 0.5 else -1 for _ in range(self.num_particles)], dtype=np.float16)   
        # dc -> Probability of changing s each interaction
        self.dc = np.zeros(self.num_particles, dtype=np.float16)
        # temperature -> Prevent theta decreasing too quickly
        self.temperature = np.zeros(self.num_particles, dtype=np.float16)
        # Trr -> Temperature reduction rate
        self.Trr = trr

        # Constants
        self.k = k
        self.p = p

    def initialize_mem_tool(self, mms: int = 10, mpr: float = 0.1, md: float = 0.1, mpp1: float = 0.1, mpp2: float = 0.1, FFthr: float = 0.0) -> None:
        self.mem: list[list[MemType]] = [[] for _ in range(self.num_particles)]
        self.mms = mms # Maximum Memory Size
        self.mpr = mpr # Memory Point Radius
        self.md = md # Minimum Displacement
        self.mpp1 = mpp1 # Memory Point Penalty 1
        self.mpp2 = mpp2 # Memory Point Penalty 2
        self.FFthr = FFthr # Threshold

    def has_collision_with_particle(self, pos: tuple[int, int]) -> bool:
        for _, agent in enumerate(self.X):
            if np.linalg.norm(agent - pos) < self.swarm_radius:
                return True
        return False
    
    def has_collision_with_wall(self, pos: tuple[int, int]) -> bool:
        bubble = (pos[0], pos[1], self.bubble_radius, self.bubble_radius)
        for wall in self.walls:
            if bubble[0] < wall[0] + wall[2] and bubble[0] + bubble[2] > wall[0] \
                and bubble[1] < wall[1] + wall[3] and bubble[1] + bubble[3] > wall[1]:
                return True
        return False
    
    def update_theta(self, index) -> np.ndarray:
        # Update velocity using AoR concept [7]
        RV = [[np.cos(self.theta[index]), -np.sin(self.theta[index])], [np.sin(self.theta[index]), np.cos(self.theta[index])]]
        RV = np.matmul(RV, self.V[index])

        # Update theta with temperature [8]
        self.theta[index] = self.temperature[index] * self.theta[index]

        # Update temperature [9]
        self.temperature[index] = self.Trr * self.temperature[index]

        return RV

    def evaluate_position(self, index: int = None) -> None:
        # Reduce ν for τ near to Pit1 inside the Memoryit by MPP2  
        if len(self.mem[index]) > 0 and self.md > self.euclidean_distance(self.X[index], self.mem[index][-1][MEM_TAL_INDEX]):
            self.mem[index][-1][MEM_COST_INDEX] -= self.mpp2

    def update_memory(self, index: int = None) -> None:
        if index is None:
            return
        
        current_position = self.pbest[index]
        
        # If a particle memory is empty, then start it
        if 0 == len(self.mem[index]):
            self.mem[index].append([current_position, self.cost_function(current_position)])
            return
        
        # For each memory registered in the memory of a particle
        for i in range(len(self.mem[index])):

            # Take the τ and ν from the memory
            tal, val = self.mem[index][i]
            
            # if Pbest distance to any τ in the memory is less than MPR
            distance = self.euclidean_distance(current_position, tal)
            if distance < self.mpr:
                # Reduce corresponding ν inside the Memory by MPP1
                self.mem[index][i][1] = val - self.mpp1

                # Select the pair with maximun ν from the memory and set its τ as Pit
                # TODO: Test with minimum value.
                best_cost_index = np.argmax(self.mem[index][i][:][1])
                self.mem[index][best_cost_index][MEM_TAL_INDEX] = current_position
            else:
                # if size of Memory i > MMS then 
                if len(self.mem[index]) > self.mms:
                    # Remove the oldest Memory i member
                    self.mem[index].pop(0)

                # Insert new xi into Memory i
                self.mem[index].append([current_position, self.cost_function(current_position)])

    def evaluate_bests(self, index: int = None, epoch: int = 0) -> None:
        if index is None:
            return
        
        pbest_cost = self.cost_function(self.pbest[index])
        local_cost = self.cost_function(self.X[index])

        # Current x has better fitness than pbest
        if local_cost < pbest_cost:
            # Update the set of Personal-Bests it with P it
            self.pbest[index] = self.X.copy()[index]
            # print(f"Epoch {epoch}. Personal best improved. Cost: {self.cost_function(self.pbest[index]):.2f} Pos: {self.pbest[index]}")

            self.update_memory(index)
         
            # Select the best element of Personal-Bests it as G i
            global_cost = self.cost_function(self.gbest)
            if pbest_cost < global_cost:
                if self.verbose:
                    print(f"Epoch {epoch}. Global best improved. Cost: {self.cost_function(self.gbest):.2f} Pos: {self.gbest}")
                self.gbest = self.pbest[index]

    def update_velocity(self, index, n1=1.0, n2=1.0) -> None:
        # Randomly set r1 and r2
        r1 = np.random.rand()
        r2 = np.random.rand()

        # Update velocity as classic PSO [2]
        self.V[index] = self.V[index] + self.c1 * n1 * r1 * (self.pbest[index] - self.X[index]) + self.c2 * n2* r2 * (self.gbest - self.X[index])

        # Limit velocity between arbitrary limits
        if np.abs(self.V[index][0]) > self.speed_limit:
            self.V[index][0] = np.sign(self.V[index][0])
        if np.abs(self.V[index][1]) > self.speed_limit:
            self.V[index][1] = np.sign(self.V[index][1])

    def update_movement(self, index, RV=(0, 0)) -> None:
        next_pos = self.X[index] + RV

        if self.has_collision_with_wall(next_pos) or self.has_collision_with_particle(next_pos):
            # Change theta [4]
            self.theta[index] += self.s[index]

            # Set temperature to maximun
            self.temperature[index] = 1.0

            # Increase Dc [5]
            self.dc[index] += self.k

            # Randomly modify s and dc (Maybe add a scaller here)
            if np.random.rand() < self.dc[index]:
                self.s[index] *= -1
                self.dc[index] = 0.0

            next_pos = self.X[index]
        else:
            # Decrease dc [6]
            self.dc[index] *= self.p

        self.X[index] = next_pos

    def register_history(self) -> None:
        self.X_history.append(self.X.copy())
        self.V_history.append(self.V.copy())
        self.pbest_history.append(self.pbest.copy())
        self.gbest_history.append(self.gbest)

    def run(self, epochs: int = 300, end_condition_checker: Callable = None, timeout_epochs: int = 1000) -> None:      
        try:
            MAX_EPOCHS = timeout_epochs
            while epochs or end_condition_checker is not None:
                counter = epochs if end_condition_checker is None else MAX_EPOCHS-timeout_epochs

                for i in range(self.num_particles):
                    # Update only if the particle is not near the goal
                    if self.cost_function(self.X[i]) < 1.0:
                        continue
                    
                    RV = self.update_theta(i)
                    self.evaluate_position(i)
                    self.evaluate_bests(i, counter)
                    
                    n1 = 1 if self.cost_function(self.pbest[i]) > self.FFthr else 0
                    n2 = 1 if self.cost_function(self.gbest) > self.FFthr else 0

                    if n1 == 0 and n2 == 0:
                        self.mem: list[list[MemType]] = [[] for _ in range(self.num_particles)]
                    else:
                        self.update_velocity(i, n1, n2)

                    self.update_movement(i, RV)

                self.register_history()

                if end_condition_checker is not None and end_condition_checker(self.gbest, self.pbest):
                    print(f"End condition reached in {counter} epochs. Best cost: {self.cost_function(self.gbest)}")
                    break
                elif end_condition_checker is None:
                    epochs -= 1
                elif end_condition_checker is not None and timeout_epochs > 0:
                    timeout_epochs -= 1
                elif timeout_epochs == 0:
                    print(f"Timeout reached in {counter} epochs. Best cost: {self.cost_function(self.gbest)}")
                    break
                
        except KeyboardInterrupt:
            print("Stopped by user")
    
    def add_virtual_wall(self, x, y, width, height) -> None:
        self.walls.append((x, y, width, height))