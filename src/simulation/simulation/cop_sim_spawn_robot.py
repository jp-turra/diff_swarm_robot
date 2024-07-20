import rclpy
import os

from rclpy.node import Node, ParameterDescriptor

from coppeliasim_zmqremoteapi_client import RemoteAPIClient

import rclpy.subscription

class CopSimSpawnRobotNode(Node):
    def __init__(self):
        super().__init__('cop_sim_spawn_robot')
        self.get_logger().info("CopSimSpawnRobotNode started")
        self.declare_parameter(
            name='robot_description_path',
            value="",
            descriptor=ParameterDescriptor(
                description='Path to the robot description file.'
            )
        )
        
        self.connect_to_coppeliasim()
        self.spawn_robot()

        self.get_logger().info("CopSimSpawnRobotNode stopped")
        self.destroy_node()

    def connect_to_coppeliasim(self):
        self.client = RemoteAPIClient(
            host='localhost', 
            port=23000,
            verbose=None,
            # WARN: 'timeout' is a custom modification that is not published in the package
            timeout=60 
        )

    def load_model(self, sim, filepath):
        assert os.path.exists(filepath)
        assert os.path.isfile(filepath)
        assert sim is not None
        
        with open(filepath, 'rb') as file:
            sim.loadModel(file.read())
        
        return

    def get_robot_description_param(self):

        robot_description_path_param = self.get_parameter('robot_description_path')
        if robot_description_path_param is None:
            self.get_logger().error("Parameter 'robot_description_path' not set!")
            raise ValueError
        
        path = robot_description_path_param.get_parameter_value().string_value
        if path is None:
            self.get_logger().error("Value of parameter 'robot_description_path' not set!")
            raise ValueError
        
        if not os.path.exists(path):
            self.get_logger().error(f"File {path} not found!")
            raise FileNotFoundError
        
        return path

    def spawn_robot(self):
        if self.client is None:
            self.get_logger().error("Client not initialized!")
            raise ValueError
        
        sim = self.client.require('sim')
        robot_desc_path = self.get_robot_description_param()
        self.load_model(sim, robot_desc_path)

def main():
    rclpy.init()
    rclpy.spin_once(CopSimSpawnRobotNode(), timeout_sec=1)
    rclpy.shutdown()

if __name__ == '__main__':
    main()