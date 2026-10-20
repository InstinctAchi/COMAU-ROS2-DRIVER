"""
kinematics.py - Forward and inverse kinematics for Comau Racer 5-0.80 SE.

Uses modified Denavit-Hartenberg parameters from config.py.
FK returns a 4x4 homogeneous transformation matrix and extracts Cartesian pose.
IK uses iterative Jacobian pseudo-inverse (damped least-squares) for MOVE_LIN.
"""

import math
from typing import List, Optional, Tuple

import numpy as np

from . import config


def dh_transform(a: float, d: float, alpha: float, theta: float) -> np.ndarray:
    """
    Compute modified DH transformation matrix (Craig convention) T_{i-1,i}.

    Args:
        a:     link length [mm]
        d:     link offset [mm]
        alpha: link twist [rad]
        theta: joint angle [rad]

    Returns:
        4x4 numpy homogeneous transformation matrix.
    """
    ct = math.cos(theta)
    st = math.sin(theta)
    ca = math.cos(alpha)
    sa = math.sin(alpha)

    return np.array([
        [ct,       -st,        0.0,    a    ],
        [st * ca,   ct * ca,  -sa,    -d * sa],
        [st * sa,   ct * sa,   ca,     d * ca],
        [0.0,       0.0,       0.0,    1.0  ],
    ], dtype=float)


def forward_kinematics(joint_deg: List[float]) -> Tuple[np.ndarray, List[float]]:
    """
    Compute FK for 6 joint angles.

    Args:
        joint_deg: list of 6 joint angles [degrees]

    Returns:
        (T_0_6, pose_6d) where:
          T_0_6  : 4x4 homogeneous matrix base→TCP
          pose_6d: [x, y, z, rx, ry, rz] in [mm, degrees]
    """
    T = np.eye(4)
    for i, (a, d, alpha, offset) in enumerate(config.DH_PARAMS):
        theta = math.radians(joint_deg[i]) + offset
        T = T @ dh_transform(a, d, alpha, theta)

    pose = matrix_to_pose(T)
    return T, pose


def matrix_to_pose(T: np.ndarray) -> List[float]:
    """
    Extract [x, y, z, rx, ry, rz] from a 4x4 homogeneous matrix.
    Uses ZYX Euler angle convention (matches Comau TP representation).

    Returns:
        [x[mm], y[mm], z[mm], rx[deg], ry[deg], rz[deg]]
    """
    x = T[0, 3]
    y = T[1, 3]
    z = T[2, 3]

    # ZYX Euler angles from rotation matrix
    r = T[:3, :3]
    ry = math.atan2(-r[2, 0], math.sqrt(r[0, 0] ** 2 + r[1, 0] ** 2))

    if abs(math.cos(ry)) > 1e-6:
        rx = math.atan2(r[2, 1] / math.cos(ry), r[2, 2] / math.cos(ry))
        rz = math.atan2(r[1, 0] / math.cos(ry), r[0, 0] / math.cos(ry))
    else:
        # Gimbal lock
        rx = 0.0
        rz = math.atan2(-r[0, 1], r[1, 1]) if r[2, 0] < 0 else math.atan2(r[0, 1], -r[1, 1])

    return [x, y, z, math.degrees(rx), math.degrees(ry), math.degrees(rz)]


def _jacobian(joint_deg: List[float]) -> np.ndarray:
    """
    Compute the 6x6 geometric Jacobian numerically (finite differences).

    Returns:
        J: (6, 6) Jacobian matrix. Rows: [vx, vy, vz, wx, wy, wz].
    """
    delta = 1e-4  # radians
    joint_rad = [math.radians(j) for j in joint_deg]
    _, base_pose = forward_kinematics(joint_deg)

    J = np.zeros((6, 6))
    for i in range(6):
        perturbed = list(joint_deg)
        perturbed[i] += math.degrees(delta)
        _, pert_pose = forward_kinematics(perturbed)
        J[:, i] = (np.array(pert_pose) - np.array(base_pose)) / delta

    return J


def inverse_kinematics(
    target_pose: List[float],
    initial_joints: List[float],
    max_iter: int = 200,
    tol_mm: float = 0.1,
    tol_deg: float = 0.1,
) -> Optional[List[float]]:
    """
    Numerical IK using damped least-squares Jacobian pseudo-inverse.

    Args:
        target_pose:    [x, y, z, rx, ry, rz] target [mm, degrees]
        initial_joints: starting joint angles [degrees]
        max_iter:       maximum iterations
        tol_mm:         position tolerance [mm]
        tol_deg:        orientation tolerance [degrees]

    Returns:
        List of 6 joint angles [degrees], or None if IK failed.
    """
    q = list(initial_joints)
    target = np.array(target_pose)
    lam = 0.05  # damping factor

    for _ in range(max_iter):
        _, current_pose = forward_kinematics(q)
        error = target - np.array(current_pose)

        pos_err = np.linalg.norm(error[:3])
        ori_err = np.linalg.norm(error[3:])
        if pos_err < tol_mm and ori_err < tol_deg:
            return q

        J = _jacobian(q)
        # Damped least-squares: dq = J^T (J J^T + λ²I)^{-1} e
        JJT = J @ J.T + (lam ** 2) * np.eye(6)
        dq_rad = J.T @ np.linalg.solve(JJT, error)

        for i in range(6):
            q[i] += math.degrees(dq_rad[i])

        # Clamp to joint limits
        limits = config.JOINT_LIMITS_DEG
        for i in range(6):
            q[i] = max(limits[i][0], min(limits[i][1], q[i]))

    return None  # IK did not converge


def check_joint_limits(joint_deg: List[float]) -> Optional[int]:
    """
    Check joint limits. Returns index (0-based) of first violated joint, else None.
    """
    for i, (lo, hi) in enumerate(config.JOINT_LIMITS_DEG):
        if joint_deg[i] < lo or joint_deg[i] > hi:
            return i
    return None
