import rclpy

from rclpy.node import Node
from geometry_msgs.msg import Twist

from coppeliasim_zmqremoteapi_client import RemoteAPIClient

class RobotInterface(Node):

    def __init__(self):
        super().__init__('robot_interface')
        self.get_logger().info("RobotInterface started")

        # TODO: Add parameters
        self.robot_name = "Rover"
        self.wheel_radius = 0.013
        self.wheel_base = 0.12


        self.client = RemoteAPIClient(
            host='localhost', 
            port=23000,
            verbose=None,
            # WARN: 'timeout' is a custom modification that is not published in the package
            timeout=60 
        )

        if self.client is None:
            self.get_logger().error("RemoteAPIClient not created!")
            raise ValueError
        
        self.cmd_vel_sub = self.create_subscription(
            Twist,
            '/cmd_vel',
            self.cmd_vel_callback,
            10
        )

        sim = self.client.require('sim')
        simState = sim.getSimulationState()
        if sim.simulation_stopped == simState:
            sim.startSimulation()

        self.left_joint = "/" + self.robot_name + '/LeftWheelJoint'
        self.right_joint = "/" +  self.robot_name + '/RightWheelJoint'

    def cmd_vel_callback(self, msg: Twist):
        throttle = msg.linear.x
        yaw = msg.angular.z

        self.send_speed_to_coppeliasim(throttle, yaw)

    def get_speed_from_robot_kinematics(self, throttle, yaw):
        # Diff drive kinematics
        left_angular_speed = (throttle - yaw * self.wheel_base / 2) / self.wheel_radius
        right_angular_speed = (throttle + yaw * self.wheel_base / 2) / self.wheel_radius

        left_speed = left_angular_speed * self.wheel_radius
        right_speed = right_angular_speed * self.wheel_radius

        return left_speed, right_speed


    def send_speed_to_coppeliasim(self, throttle, yaw):

        sim = self.client.require('sim')
        left_joint_obj = sim.getObject(self.left_joint)
        right_joint_obj = sim.getObject(self.right_joint)

        left_speed, right_speed = self.get_speed_from_robot_kinematics(throttle, yaw)

        print(left_speed, right_speed)

        sim.setJointTargetVelocity(left_joint_obj, left_speed)
        sim.setJointTargetVelocity(right_joint_obj, right_speed)

def main(args=None):
    rclpy.init(args=args)
    rclpy.spin(RobotInterface())
    rclpy.shutdown()

if __name__ == '__main__':
    main()