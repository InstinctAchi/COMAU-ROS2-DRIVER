"""
alarm_manager.py - Comau C5G alarm system for the robot simulator.

Error codes and severity levels mirror the Comau C5G Maintenance Manual format:
    XXXXX-LL : description
where XXXXX = 5-digit error code, LL = severity level (00-12).

Severity levels:
    00-01 : Informative (no system impact)
    02    : Warning
    04    : PDL2 program error (program paused)
    06    : PDL2 program error (all programs paused)
    08    : Hold
    10-12 : Drive Off / Deactivate
"""

from typing import Dict, List, Optional, Tuple


# ---------------------------------------------------------------------------
# Alarm catalogue - (code, level, short description)
# ---------------------------------------------------------------------------
ALARM_CATALOGUE: Dict[int, Tuple[int, str]] = {
    20001: (12, "Emergency stop active"),
    20002: (10, "Drive fault: overcurrent on axis"),
    20003: (10, "Drive fault: power module error"),
    20004: (10, "Drive fault: encoder communication lost"),
    20012: (10, "Joint position limit exceeded"),
    20013:  (8, "Joint velocity limit exceeded"),
    20020:  (8, "Path following error too large"),
    10005:  (8, "Communication timeout: client heartbeat lost"),
    10006:  (4, "Socket framing error: bad checksum"),
    10007:  (4, "Unknown message type received"),
    30001:  (4, "PDL2 program error: invalid command sequence"),
    30002:  (4, "PDL2 program error: motion target unreachable (IK failed)"),
    40001:  (2, "Motion not executed: robot in HOLD state"),
    40002:  (2, "Speed override out of range (must be 0-100%)"),
    50010:  (0, "Gripper feedback mismatch: DI not confirmed"),
    60001:  (4, "Protocol error: unknown MSG_TYPE"),
    60002:  (4, "Protocol error: invalid payload length"),
}


class AlarmManager:
    """
    Manages the active alarm list for the simulated Comau C5G controller.

    Mirrors the controller ERROR.LOG behaviour:
    - Each alarm is stored once (no duplicates by code).
    - Clearing alarms does NOT automatically re-enable drives (caller must do it).
    """

    def __init__(self) -> None:
        # code -> (level, description, timestamp)
        self._active: Dict[int, Tuple[int, str]] = {}

    def raise_alarm(self, code: int, extra_info: str = "") -> Tuple[int, str]:
        """
        Raise an alarm by code. Looks up level and description from catalogue.
        Returns (level, description).
        """
        if code in ALARM_CATALOGUE:
            level, desc = ALARM_CATALOGUE[code]
        else:
            level, desc = (4, f"Unknown alarm code {code}")

        if extra_info:
            desc = f"{desc}: {extra_info}"

        self._active[code] = (level, desc)
        return level, desc

    def clear_all(self) -> None:
        """Clear all active alarms."""
        self._active.clear()

    def clear_alarm(self, code: int) -> bool:
        """Clear a specific alarm. Returns True if it was present."""
        return self._active.pop(code, None) is not None

    def is_active(self, code: int) -> bool:
        return code in self._active

    def max_severity(self) -> int:
        """Return the highest active severity level, or 0 if no alarms."""
        if not self._active:
            return 0
        return max(level for level, _ in self._active.values())

    def blocks_motion(self) -> bool:
        """Return True if any active alarm prevents motion (level >= 8)."""
        return self.max_severity() >= 8

    def active_list(self) -> List[Tuple[int, int, str]]:
        """Return list of (code, level, description) for all active alarms."""
        return [(code, level, desc) for code, (level, desc) in self._active.items()]

    def format_tp_style(self) -> List[str]:
        """
        Format alarm list as it would appear on the Comau Teach Pendant:
            0001.20012-10 : Joint position limit exceeded
        """
        lines = []
        for i, (code, (level, desc)) in enumerate(self._active.items(), start=1):
            lines.append(f"{i:04d}.{code:05d}-{level:02d} : {desc}")
        return lines
