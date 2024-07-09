import os
import xacro

from ament_index_python import get_package_share_path

from launch import LaunchDescription

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

    # Run Xacro on the URDF file
    robot_description = xacro.process_file(project_robot_path).toxml()

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

    joint_state_publisher_gui = Node(
        package='joint_state_publisher_gui',
        executable='joint_state_publisher_gui',
        name='joint_state_publisher_gui',
        output='screen'
    )

    joint_state_publisher_node = Node(
        package='joint_state_publisher',
        executable='joint_state_publisher',
        name='joint_state_publisher',
        output='screen'
    )

    rviz_config_path = os.path.join(
        project_name_path, 
        'rviz', 
        'robot.rviz'
    )

    rviz_node = Node(
        package='rviz2',
        executable='rviz2',
        name='rviz2',
        output='screen',
        arguments=['-d', rviz_config_path]
    )

    return LaunchDescription([
        robot_state_publisher_node,
        joint_state_publisher_gui,
        joint_state_publisher_node,
        rviz_node
    ])