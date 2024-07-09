import os
import xacro

from ament_index_python import get_package_share_path

from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription
from launch.substitutions import LaunchConfiguration
from launch.launch_description_sources import PythonLaunchDescriptionSource

from launch_ros.actions import Node

project_name="robot"

def generate_launch_description():
    # Get <project_name> shared folder
    project_name_path = get_package_share_path(project_name)

    # Get robot URDF path
    project_robot_path = os.path.join(
        project_name_path, 
        'urdf', 
        'robot_model.xacro'
    )

    world_path = os.path.join(project_name_path, 'worlds', 'empty_world.world')

    # Run Xacro on the URDF file
    robot_description = xacro.process_file(project_robot_path).toxml()

    # Gazebo ROS launch
    gazebo_ros_launch = PythonLaunchDescriptionSource(
        os.path.join(get_package_share_path('gazebo_ros'), 'launch', 'gazebo.launch.py')
    )

    # Gazebo launch description
    gazebo_ros_launch_desc = IncludeLaunchDescription(
        gazebo_ros_launch,
        launch_arguments={
            'world': world_path,
        }.items()
    )

    # Node to spawn the robot
    spawn_robot = Node(
        package='gazebo_ros',
        executable='spawn_entity.py',
        arguments=[
            '-topic', 'robot_description',
            '-entity', 'model'
        ],
        output='screen'
    )

    # Robot State Publisher Node
    robot_state_publisher_node = Node(
        package='robot_state_publisher',
        executable='robot_state_publisher',
        name='robot_state_publisher',
        output='screen',
        parameters=[{
            'robot_description': robot_description,
            'use_sim_time': True
        }]
    )

    return LaunchDescription([
        gazebo_ros_launch_desc,
        spawn_robot,
        robot_state_publisher_node
    ])