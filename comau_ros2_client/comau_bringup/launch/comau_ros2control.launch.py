"""
Launch file for Comau robot using ROS 2 Control stack.

Replaces the legacy execute_trajectory_handler_node approach with:
  - ros2_control_node + comau_hardware_interface (real/mirror modes)
  - joint_trajectory_controller spawner
  - gripper_controller spawner

Arguments:
  robot_ip           : IP address of the real Comau robot (default: 192.168.2.1)
  config_package     : ROS 2 package containing robot config files
  controllers_yaml   : path to the controllers yaml (default: auto-detected from config_package)
  control_urdf_xacro : path to the ros2_control URDF xacro (default: auto-detected from config_package)
"""

import os
import subprocess
import tempfile

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, OpaqueFunction
from launch_ros.actions import Node


def launch_setup(context, *args, **kwargs):
    robot_ip = context.launch_configurations.get('robot_ip', '192.168.2.1')
    config_package = context.launch_configurations.get('config_package', '')

    # Resolve config_package share directory
    if config_package:
        try:
            pkg_share = get_package_share_directory(config_package)
        except Exception:
            pkg_share = ''
    else:
        pkg_share = ''

    # Resolve ros2_control URDF xacro
    control_urdf_xacro = context.launch_configurations.get('control_urdf_xacro', '')
    if not control_urdf_xacro or not os.path.exists(control_urdf_xacro):
        if pkg_share:
            candidate = os.path.join(pkg_share, 'config', 'robot_control_real.urdf.xacro')
            if os.path.exists(candidate):
                control_urdf_xacro = candidate

    if not control_urdf_xacro or not os.path.exists(control_urdf_xacro):
        raise RuntimeError(
            f"ros2_control URDF xacro not found. "
            f"Expected: config/robot_control_real.urdf.xacro in package '{config_package}'. "
            f"Path tried: {control_urdf_xacro}"
        )

    # Resolve controllers yaml
    controllers_yaml = context.launch_configurations.get('controllers_yaml', '')
    if not controllers_yaml or not os.path.exists(controllers_yaml):
        if pkg_share:
            candidate = os.path.join(pkg_share, 'config', 'ros2_controllers_real.yaml')
            if os.path.exists(candidate):
                controllers_yaml = candidate

    if not controllers_yaml or not os.path.exists(controllers_yaml):
        raise RuntimeError(
            f"Controllers yaml not found. "
            f"Expected: config/ros2_controllers_real.yaml in package '{config_package}'. "
            f"Path tried: {controllers_yaml}"
        )

    # Inject robot_ip into the xacro file by replacing the placeholder, then compile
    with open(control_urdf_xacro, 'r', encoding='utf-8') as f:
        xacro_content = f.read()

    xacro_content_with_ip = xacro_content.replace('__ROBOT_IP__', robot_ip)

    # Write patched xacro to temp file, then compile with xacro to get URDF string
    with tempfile.NamedTemporaryFile(
        mode='w', suffix='.urdf.xacro', delete=False, encoding='utf-8'
    ) as tmp_xacro:
        tmp_xacro.write(xacro_content_with_ip)
        tmp_xacro_path = tmp_xacro.name

    try:
        xacro_bin = '/opt/ros/jazzy/bin/xacro'
        if not os.path.exists(xacro_bin):
            import shutil
            xacro_bin = shutil.which('xacro') or 'xacro'

        result = subprocess.run(
            [xacro_bin, tmp_xacro_path],
            capture_output=True,
            text=True,
            timeout=30.0,
        )
        if result.returncode != 0:
            raise RuntimeError(f"xacro compilation failed: {result.stderr}")
        robot_description_content = result.stdout
    finally:
        try:
            os.unlink(tmp_xacro_path)
        except Exception:
            pass

    robot_description_param = {'robot_description': robot_description_content}

    # ros2_control_node — manages hardware interfaces
    ros2_control_node = Node(
        package='controller_manager',
        executable='ros2_control_node',
        parameters=[
            robot_description_param,
            controllers_yaml,
        ],
        output='screen',
    )

    # robot_state_publisher — publishes TF from joint states
    robot_state_publisher = Node(
        package='robot_state_publisher',
        executable='robot_state_publisher',
        name='robot_state_publisher_hw',
        output='both',
        parameters=[
            robot_description_param,
            {'use_sim_time': False},
        ],
    )

    # Spawner for joint state broadcaster
    jsb_spawner = Node(
        package='controller_manager',
        executable='spawner',
        arguments=[
            'joint_state_broadcaster',
            '--controller-manager', '/controller_manager',
        ],
        output='screen',
    )

    # Spawner for arm trajectory controller
    arm_spawner = Node(
        package='controller_manager',
        executable='spawner',
        arguments=[
            'joint_trajectory_controller',
            '--controller-manager', '/controller_manager',
        ],
        output='screen',
    )

    nodes = [
        ros2_control_node,
        robot_state_publisher,
        jsb_spawner,
        arm_spawner,
    ]

    has_gripper = False
    if controllers_yaml and os.path.exists(controllers_yaml):
        try:
            import yaml
            with open(controllers_yaml, 'r', encoding='utf-8') as f:
                c_data = yaml.safe_load(f) or {}
            cm_params = c_data.get('controller_manager', {}).get('ros__parameters', {})
            has_gripper = 'gripper_controller' in cm_params
        except Exception:
            pass

    if has_gripper:
        gripper_spawner = Node(
            package='controller_manager',
            executable='spawner',
            arguments=[
                'gripper_controller',
                '--controller-manager', '/controller_manager',
            ],
            output='screen',
        )
        nodes.append(gripper_spawner)

    return nodes


def generate_launch_description():
    declared_args = [
        DeclareLaunchArgument(
            'robot_ip',
            default_value='192.168.2.1',
            description='IP address of the Comau robot controller',
        ),
        DeclareLaunchArgument(
            'config_package',
            default_value='',
            description='ROS 2 package containing robot_control_real.urdf.xacro and ros2_controllers_real.yaml',
        ),
        DeclareLaunchArgument(
            'control_urdf_xacro',
            default_value='',
            description='Absolute path to robot_control_real.urdf.xacro (overrides config_package lookup)',
        ),
        DeclareLaunchArgument(
            'controllers_yaml',
            default_value='',
            description='Absolute path to ros2_controllers_real.yaml (overrides config_package lookup)',
        ),
    ]

    return LaunchDescription(declared_args + [OpaqueFunction(function=launch_setup)])
