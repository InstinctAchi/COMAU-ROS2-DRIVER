#!/usr/bin/env python3
"""Builds the parameter file of a standalone MoveIt Servo node from the files of a rospackage.

Only reads files; used by run_servo.sh. The node name in the file is `servo_node`.
"""
import argparse
import os

import yaml

ap = argparse.ArgumentParser()
ap.add_argument("--config-dir", required=True, help="<rospackage>/config")
ap.add_argument("--out", required=True)
ap.add_argument("--command-out-topic", required=True)
ap.add_argument("--group", default="tool0_group")
ap.add_argument("--scale-linear", type=float, default=0.15, help="m/s: cap of the cartesian speed (twist and pose)")
ap.add_argument("--scale-rotational", type=float, default=0.3, help="rad/s")
a = ap.parse_args()


def read(name):
    with open(os.path.join(a.config_dir, name), encoding="utf-8") as f:
        return f.read()


urdf = read("robot.urdf")
srdf = read("robot.srdf")
joint_limits = yaml.safe_load(read("joint_limits.yaml"))

params = {
    "robot_description": urdf,
    "robot_description_semantic": srdf,
    # Servo works on the Jacobian: KDL is enough and avoids loading the EAIK plugin in this process
    "robot_description_kinematics": {
        a.group: {"kinematics_solver": "kdl_kinematics_plugin/KDLKinematicsPlugin",
                  "kinematics_solver_search_resolution": 0.005,
                  "kinematics_solver_timeout": 0.005},
    },
    "robot_description_planning": joint_limits,
    # needed by online_signal_smoothing::AccelerationLimitedPlugin (as in the MoveIt Servo demo launch)
    "update_period": 0.01,
    "planning_group_name": a.group,
    "moveit_servo": {
        "move_group_name": a.group,
        "publish_period": 0.01,
        "max_expected_latency": 0.2,
        "incoming_command_timeout": 0.1,
        "command_in_type": "speed_units",
        "scale": {"linear": a.scale_linear, "rotational": a.scale_rotational, "joint": 0.3},
        "command_out_type": "trajectory_msgs/JointTrajectory",
        "command_out_topic": a.command_out_topic,
        "publish_joint_positions": True,
        # positions only: the JTC (allow_nonzero_velocity_at_trajectory_end = false) rejects every single-point
        # trajectory whose last point has a non-zero velocity
        "publish_joint_velocities": False,
        "publish_joint_accelerations": False,
        "joint_topic": "/joint_states",
        "status_topic": "~/status",
        "cartesian_command_in_topic": "~/delta_twist_cmds",
        "pose_command_in_topic": "~/pose_target_cmds",
        "joint_command_in_topic": "~/delta_joint_cmds",
        "use_smoothing": True,
        "smoothing_filter_plugin_name": "online_signal_smoothing::AccelerationLimitedPlugin",
        "is_primary_planning_scene_monitor": False,
        "monitored_planning_scene_topic": "/monitored_planning_scene",
        "check_collisions": False,
        "check_octomap_collisions": False,
        "lower_singularity_threshold": 17.0,
        "hard_stop_singularity_threshold": 30.0,
        "leaving_singularity_threshold_multiplier": 2.0,
        "joint_limit_margins": [0.1],
        "apply_twist_commands_about_ee_frame": True,
        "halt_all_joints_in_cartesian_mode": True,
    },
}
with open(a.out, "w", encoding="utf-8") as f:
    yaml.safe_dump({"servo_node": {"ros__parameters": params}}, f, default_flow_style=False, allow_unicode=True)
print(f"servo parameters written to {a.out} (command_out_topic={a.command_out_topic}, group={a.group})")
