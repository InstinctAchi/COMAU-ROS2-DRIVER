# Copyright 2024 Comau Robotics S.p.A.

from launch_ros.substitutions import FindPackageShare
from launch import LaunchDescription
from launch.actions import IncludeLaunchDescription
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import PathJoinSubstitution

def generate_launch_description():
    return LaunchDescription([
        IncludeLaunchDescription(
            PythonLaunchDescriptionSource([
                PathJoinSubstitution([
                    FindPackageShare('racer5-0-80_description'),
                    'launch',
                    'view_racer5-0-80.launch.py'
                ])
            ])
        )
    ])
