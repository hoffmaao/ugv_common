# ugv_common/types.py
from dataclasses import dataclass
from enum import Enum
from typing import Optional
import math

def wrap_angle(rad: float) -> float:
    """Wrap angle to [-pi, pi]."""
    while rad > math.pi:
        rad -= 2.0 * math.pi
    while rad < -math.pi:
        rad += 2.0 * math.pi
    return rad

@dataclass
class Pose2D:
    x: float = 0.0     # meters (local ENU/map frame)
    y: float = 0.0
    yaw: float = 0.0   # radians

@dataclass
class Twist2D:
    v: float = 0.0     # forward speed (m/s)
    omega: float = 0.0 # yaw rate (rad/s)

@dataclass
class Cov2D:
    # Minimal covariance summary (could be extended to full 5x5 later)
    var_x: float = 1.0
    var_y: float = 1.0
    var_yaw: float = 0.5

class FixStatus(Enum):
    NO_FIX = 0
    FIX_2D = 1
    FIX_3D = 2
    RTK_FLOAT = 4
    RTK_FIXED = 5

@dataclass
class BearingSteer:
    distance_m: float
    bearing_rad: float          # target bearing from rover to waypoint
    heading_error_rad: float    # target bearing - rover yaw (wrapped)
    suggested_speed_mps: float
    suggested_omega_rps: float
    reached: bool