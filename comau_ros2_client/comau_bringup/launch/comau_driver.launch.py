# Copyright 2024 Comau Robotics S.p.A.

import os
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import Command, FindExecutable, PathJoinSubstitution, LaunchConfiguration
from launch_ros.actions import Node
from launch_ros.substitutions import FindPackageShare

def generate_launch_description():
    declared_arguments = [
        DeclareLaunchArgument(
            "robot_ip",
            default_value="127.0.0.1",
            description="IP of the Comau robot controller",
        )
    ]
    robot_ip = LaunchConfiguration("robot_ip")
    
    robot_description_content = Command([
        PathJoinSubstitution([FindExecutable(name="xacro")]),
        " ",
        PathJoinSubstitution([
            FindPackageShare("racer5-0-80_description"),
            "robots",
            "racer5-0-80_robot.urdf.xacro",
        ]),
    ])
    robot_description = {"robot_description": robot_description_content}
    
    config_param = os.path.join(
        get_package_share_directory('comau_bringup'),
        'config',
        'general_config.yaml',
    )
    
    robot_state_publisher_node = Node(
        package="robot_state_publisher",
        executable="robot_state_publisher",
        name="robot_state_publisher",
        output="screen",
        parameters=[robot_description],
    )
    
    driver_node = Node(
        package="comau_handlers",
        executable="execute_trajectory_handler_node",
        name="execute_trajectory_handler_node",
        parameters=[
            robot_description,
            config_param,
            {"robot_ip": robot_ip}
        ],
        output="screen"
    )
    
    return LaunchDescription(declared_arguments + [robot_state_publisher_node, driver_node])
