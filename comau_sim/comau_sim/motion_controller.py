"""
motion_controller.py - Trapezoidal velocity profile joint interpolator.

Runs as a persistent asyncio task at MOTION_LOOP_HZ. Waits on an asyncio.Event
(state.motion_event) set by the TCP server when a new move is requested.

Profile:
  Phase 1: acceleration from current velocity to max speed
  Phase 2: constant velocity (may be skipped for short moves)
  Phase 3: deceleration to zero

Speed is scaled by state.speed_override (percentage).
Joint limit violations during motion raise alarm 20012 and abort the move.
"""

import asyncio
import math
import time
from typing import Callable, Coroutine, List, Optional

from . import config
from .alarm_manager import AlarmManager
from .kinematics import check_joint_limits, forward_kinematics
from .robot_state import RobotState, RobotStatus


async def motion_loop(
    state: RobotState,
    alarm_mgr: AlarmManager,
    notify_done_cb: Optional[Callable[[List[float]], Coroutine]] = None,
) -> None:
    """
    Main motion controller loop. Runs forever until task is cancelled.

    Args:
        state:          shared robot state
        alarm_mgr:      alarm manager instance
        notify_done_cb: async callback called with final joint positions when
                        a move completes (used to push MOTION_DONE to client)
    """
    dt = 1.0 / config.MOTION_LOOP_HZ

    while True:
        # Wait for a motion request
        await state.motion_event.wait()
        state.motion_event.clear()

        async with state.lock:
            if alarm_mgr.blocks_motion():
                state.status = RobotStatus.IDLE
                continue

            target = state.motion_target_joints
            if target is None:
                state.status = RobotStatus.IDLE
                continue

            start = list(state.joint_pos)
            ovr = max(0, min(100, state.speed_override)) / 100.0
            state.status = RobotStatus.MOVING

        # Compute per-axis move plan using trapezoidal profile
        plans = []
        for i in range(6):
            delta = target[i] - start[i]
            max_v = config.JOINT_MAX_SPEED_DEG_S[i] * ovr
            accel = config.JOINT_ACCEL_DEG_S2[i]
            plans.append(_trapezoid_plan(delta, max_v, accel))

        # Duration of the longest-axis move
        total_time = max(p["total_time"] for p in plans)
        elapsed = 0.0
        prev_time = time.monotonic()

        while elapsed < total_time:
            await asyncio.sleep(dt)

            now = time.monotonic()
            elapsed += now - prev_time
            prev_time = now
            t = min(elapsed, total_time)

            new_joints = [
                start[i] + _trapezoid_position(plans[i], t * (plans[i]["total_time"] / total_time) if total_time > 0 else plans[i]["total_time"])
                for i in range(6)
            ]

            # Check joint limits every step
            violated = check_joint_limits(new_joints)
            if violated is not None:
                alarm_mgr.raise_alarm(20012, f"J{violated + 1}={new_joints[violated]:.2f}deg")
                async with state.lock:
                    state.status = RobotStatus.HOLD
                    state.motion_target_joints = None
                break

            # Check if another request came in (preempt)
            async with state.lock:
                if state.motion_event.is_set():
                    break

                state.joint_pos = new_joints
                state.joint_vel = _compute_velocities(plans, t, total_time)

                # Update cartesian pose via FK
                _, pose = forward_kinematics(new_joints)
                state.cartesian.x  = pose[0]
                state.cartesian.y  = pose[1]
                state.cartesian.z  = pose[2]
                state.cartesian.rx = pose[3]
                state.cartesian.ry = pose[4]
                state.cartesian.rz = pose[5]

        # Move complete
        async with state.lock:
            if state.status == RobotStatus.MOVING:
                # Snap to exact target
                state.joint_pos = list(target)
                state.joint_vel = [0.0] * 6
                _, pose = forward_kinematics(target)
                state.cartesian.x  = pose[0]
                state.cartesian.y  = pose[1]
                state.cartesian.z  = pose[2]
                state.cartesian.rx = pose[3]
                state.cartesian.ry = pose[4]
                state.cartesian.rz = pose[5]
                state.status = RobotStatus.IDLE
                final_joints = list(state.joint_pos)
            else:
                final_joints = None

            state.motion_target_joints = None

        if final_joints and notify_done_cb:
            await notify_done_cb(final_joints)


# ---------------------------------------------------------------------------
# Trapezoidal velocity profile helpers
# ---------------------------------------------------------------------------

def _trapezoid_plan(delta: float, max_v: float, accel: float) -> dict:
    """
    Compute a trapezoidal profile for a scalar displacement delta [deg].

    Returns a dict with keys:
      direction, accel, max_v, t_accel, t_const, t_decel, total_time, delta_abs
    """
    if max_v <= 0 or accel <= 0:
        return {"direction": 0, "accel": 1, "max_v": 0.001,
                "t_accel": 0, "t_const": 0, "t_decel": 0,
                "total_time": 0.001, "delta_abs": 0}

    direction = 1 if delta >= 0 else -1
    delta_abs = abs(delta)

    t_accel = max_v / accel
    d_accel = 0.5 * accel * t_accel ** 2

    if 2 * d_accel >= delta_abs:
        # Triangle profile (short move, never reaches max_v)
        t_accel = math.sqrt(delta_abs / accel)
        t_const = 0.0
        t_decel = t_accel
        actual_max_v = accel * t_accel
    else:
        t_const = (delta_abs - 2 * d_accel) / max_v
        t_decel = t_accel
        actual_max_v = max_v

    return {
        "direction": direction,
        "accel": accel,
        "max_v": actual_max_v,
        "t_accel": t_accel,
        "t_const": t_const,
        "t_decel": t_decel,
        "total_time": t_accel + t_const + t_decel,
        "delta_abs": delta_abs,
    }


def _trapezoid_position(plan: dict, t: float) -> float:
    """Return displacement [deg] at time t for the given trapezoidal plan."""
    t_a = plan["t_accel"]
    t_c = plan["t_const"]
    t_d = plan["t_decel"]
    a   = plan["accel"]
    v   = plan["max_v"]
    d   = plan["direction"]

    if t <= t_a:
        pos = 0.5 * a * t ** 2
    elif t <= t_a + t_c:
        pos = 0.5 * a * t_a ** 2 + v * (t - t_a)
    else:
        t_rem = t - t_a - t_c
        pos = 0.5 * a * t_a ** 2 + v * t_c + v * t_rem - 0.5 * a * t_rem ** 2

    return d * min(pos, plan["delta_abs"])


def _compute_velocities(plans: list, t: float, total_time: float) -> List[float]:
    """Instantaneous velocities [deg/s] for each axis at time t."""
    vels = []
    for i, plan in enumerate(plans):
        local_t = t * (plan["total_time"] / total_time) if total_time > 0 else plan["total_time"]
        t_a = plan["t_accel"]
        t_c = plan["t_const"]
        a   = plan["accel"]
        v   = plan["max_v"]
        d   = plan["direction"]

        if local_t <= t_a:
            vel = a * local_t
        elif local_t <= t_a + t_c:
            vel = v
        else:
            t_rem = local_t - t_a - t_c
            vel = max(0.0, v - a * t_rem)

        vels.append(d * vel)
    return vels
