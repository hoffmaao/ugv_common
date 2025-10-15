from dataclasses import dataclass
import math

def wrap_angle(rad: float) -> float:
    while rad >  math.pi: rad -= 2.0*math.pi
    while rad < -math.pi: rad += 2.0*math.pi
    return rad

@dataclass
class Pose2D:
    x: float = 0.0
    y: float = 0.0
    yaw: float = 0.0  # radians

@dataclass
class Twist2D:
    v: float = 0.0
    omega: float = 0.0

@dataclass
class BearingSteer:
    distance_m: float
    bearing_rad: float
    heading_error_rad: float
    suggested_speed_mps: float
    suggested_omega_rps: float
    reached: bool