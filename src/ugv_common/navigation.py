from dataclasses import dataclass
from typing import Optional
import math, numpy as np
from .rover import RoverState
from .survey import Survey, Waypoint
from .types import BearingSteer, wrap_angle

@dataclass
class NavigatorConfig:
    """
    Configuration parameters for the Navigator.

    Attributes:
        lookahead_m (float): Lookahead distance in meters for path planning.
        max_speed_mps (float): Maximum speed in meters per second.
        min_speed_mps (float): Minimum speed in meters per second.
        goal_tolerance_m (float): Distance tolerance to consider a waypoint reached.
        slowdown_radius_m (float): Radius within which the rover slows down.
        horizon_s (float): Time horizon for previewing the rover's pose.
    """
    lookahead_m: float = 1.2
    max_speed_mps: float = 1.0
    min_speed_mps: float = 0.2
    goal_tolerance_m: float = 0.5
    slowdown_radius_m: float = 2.5
    horizon_s: float = 0.6  # preview pose this far ahead

class Navigator:
    """
    Handles navigation for a rover based on waypoints and configuration.

    Attributes:
        rover (RoverState): The current state of the rover.
        survey (Survey): The survey containing waypoints for navigation.
        cfg (NavigatorConfig): Configuration parameters for navigation.
        index (int): The current waypoint index.
    """
    def __init__(self, rover: RoverState, survey: Survey, cfg: NavigatorConfig = NavigatorConfig()):
        """
        Initializes the Navigator with the rover state, survey, and configuration.

        Args:
            rover (RoverState): The current state of the rover.
            survey (Survey): The survey containing waypoints for navigation.
            cfg (NavigatorConfig, optional): Configuration parameters for navigation. Defaults to NavigatorConfig().
        """
        self.rover = rover; self.survey = survey; self.cfg = cfg; self.index = 0

    def _wp(self) -> Optional[Waypoint]:
        """
        Retrieves the current waypoint based on the index.

        Returns:
            Optional[Waypoint]: The current waypoint or None if the index is out of range.
        """
        return self.survey.waypoints[self.index] if 0 <= self.index < len(self.survey.waypoints) else None

    def distance_to_next(self) -> float:
        """
        Calculates the distance to the next waypoint.

        Returns:
            float: Distance to the next waypoint in meters.
        """
        wp = self._wp()
        if not wp: return 0.0
        dx,dy = wp.x - self.rover.x_m, wp.y - self.rover.y_m
        return math.hypot(dx,dy)

    def bearing_to_next(self) -> float:
        """
        Calculates the bearing to the next waypoint.

        Returns:
            float: Bearing to the next waypoint in radians.
        """
        wp = self._wp()
        if not wp: return 0.0
        dx,dy = wp.x - self.rover.x_m, wp.y - self.rover.y_m
        return math.atan2(dy,dx)

    def compute_steering(self) -> Optional[BearingSteer]:
        """
        Computes the steering commands to navigate towards the next waypoint.

        Returns:
            Optional[BearingSteer]: Steering commands including distance, bearing, heading error, speed, and omega.
        """
        wp = self._wp()
        if not wp: return None
        # predict a simple future pose using current v, yaw rate ~ 0
        T = self.cfg.horizon_s
        rx = self.rover.x_m + self.rover.speed_mps * math.cos(self.rover.yaw_rad) * T
        ry = self.rover.y_m + self.rover.speed_mps * math.sin(self.rover.yaw_rad) * T
        ryaw = self.rover.yaw_rad

        dx,dy = wp.x - rx, wp.y - ry
        dist = math.hypot(dx,dy)
        bearing = math.atan2(dy,dx)
        heading_err = wrap_angle(bearing - ryaw)

        # pure pursuit curvature
        Ld = max(self.cfg.lookahead_m, min(dist, 3.0*self.cfg.lookahead_m))
        gx =  math.cos(-ryaw)*dx - math.sin(-ryaw)*dy
        gy =  math.sin(-ryaw)*dx + math.cos(-ryaw)*dy
        kappa = (2.0*gy) / max(1e-3, Ld*Ld)

        # speed schedule
        r = self.cfg.slowdown_radius_m
        speed = self.cfg.min_speed_mps + (self.cfg.max_speed_mps-self.cfg.min_speed_mps)*max(0.0, min(1.0, dist/r))
        omega = kappa * speed

        reached = dist <= max(self.cfg.goal_tolerance_m, wp.tolerance_m)
        return BearingSteer(dist, bearing, heading_err, 0.0 if reached else speed, 0.0 if reached else omega, reached)

    def step(self) -> tuple[float, float]:
        """
        Executes a navigation step, updating the rover's speed and angular velocity.

        Returns:
            tuple[float, float]: Suggested speed (m/s) and angular velocity (rad/s).
        """
        steer = self.compute_steering()
        if steer is None or steer.reached:
            if steer and steer.reached: self.index += 1
            return (0.0, 0.0)
        return (steer.suggested_speed_mps, steer.suggested_omega_rps)