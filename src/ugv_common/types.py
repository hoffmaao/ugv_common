from dataclasses import dataclass
import math

def wrap_angle(rad: float) -> float:
    """
    Normalizes an angle to the range [-π, π].

    Args:
        rad (float): Angle in radians.

    Returns:
        float: Normalized angle in radians.
    """
    while rad >  math.pi: rad -= 2.0*math.pi
    while rad < -math.pi: rad += 2.0*math.pi
    return rad

@dataclass
class Pose2D:
    """
    Represents a 2D pose with position and orientation.

    Attributes:
        x (float): X-coordinate in meters.
        y (float): Y-coordinate in meters.
        yaw (float): Orientation in radians.
    """
    x: float = 0.0
    y: float = 0.0
    yaw: float = 0.0  # radians

@dataclass
class Twist2D:
    """
    Represents a 2D twist with linear and angular velocities.

    Attributes:
        v (float): Linear velocity in meters per second.
        omega (float): Angular velocity in radians per second.
    """
    v: float = 0.0
    omega: float = 0.0

@dataclass
class BearingSteer:
    """
    Represents steering information based on bearing and distance.

    Attributes:
        distance_m (float): Distance to the target in meters.
        bearing_rad (float): Bearing to the target in radians.
        heading_error_rad (float): Heading error in radians.
        suggested_speed_mps (float): Suggested speed in meters per second.
        suggested_omega_rps (float): Suggested angular velocity in radians per second.
        reached (bool): Whether the target has been reached.
    """
    distance_m: float
    bearing_rad: float
    heading_error_rad: float
    suggested_speed_mps: float
    suggested_omega_rps: float
    reached: bool