# ugv_common/navigation.py
from __future__ import annotations
from dataclasses import dataclass
from typing import Optional
import math
import numpy as np

from .rover import RoverState
from .survey import Survey, Waypoint
from .types import BearingSteer, wrap_angle

@dataclass
class NavigatorConfig:
    lookahead_m: float = 1.2
    max_speed_mps: float = 1.0
    min_speed_mps: float = 0.2
    goal_tolerance_m: float = 0.5
    slowdown_radius_m: float = 2.0
    yaw_kp: float = 1.5     # used if not using curvature directly
    horizon_s: float = 0.5  # predict future mean pose using EnKF ensembles

class Navigator:
    """
    Uses RoverState (with EnKF posterior) and a Survey to compute
    distance/bearing to next waypoint and a steering command (v, omega).
    """
    def __init__(self, rover: RoverState, survey: Survey, cfg: NavigatorConfig = NavigatorConfig()):
        self.rover = rover
        self.survey = survey
        self.cfg = cfg
        self.index = 0
        self._last_cmd = (0.0, 0.0)

    # ---------- Core queries ----------
    def distance_to_next(self) -> float:
        wp = self._wp()
        if not wp:
            return 0.0
        dx = wp.x - self.rover.pose.x
        dy = wp.y - self.rover.pose.y
        return math.hypot(dx, dy)

    def bearing_to_next(self) -> float:
        wp = self._wp()
        if not wp:
            return 0.0
        dx = wp.x - self.rover.pose.x
        dy = wp.y - self.rover.pose.y
        return math.atan2(dy, dx)

    # ---------- Steering (Pure Pursuit with EnKF lookahead) ----------
    def compute_steering(self) -> Optional[BearingSteer]:
        wp = self._wp()
        if not wp:
            return None

        # 1) Predict the ensemble mean T seconds ahead using current v, omega
        T = self.cfg.horizon_s
        # Copy ensembles and propagate deterministically (no noise) for preview
        X = self.rover.enkf.X.copy()
        x, y, yaw, v, omega = X
        x_p = x + v * np.cos(yaw) * T
        y_p = y + v * np.sin(yaw) * T
        yaw_p = yaw + omega * T
        mu_p = np.array([np.mean(x_p), np.mean(y_p), wrap_angle(float(np.mean(yaw_p)))])
        # Use predicted mean as our working pose for steering decisions
        rx, ry, ryaw = mu_p

        # 2) Geometry to waypoint
        dx = wp.x - rx
        dy = wp.y - ry
        dist = math.hypot(dx, dy)
        bearing = math.atan2(dy, dx)
        heading_err = wrap_angle(bearing - ryaw)

        # 3) Pure-pursuit curvature to a lookahead point
        Ld = max(self.cfg.lookahead_m, min(dist, 3.0 * self.cfg.lookahead_m))
        # Transform goal to "vehicle frame" at predicted pose
        gx =  math.cos(-ryaw) * dx - math.sin(-ryaw) * dy
        gy =  math.sin(-ryaw) * dx + math.cos(-ryaw) * dy
        # κ = 2*y / Ld^2  (y is lateral error of the lookahead point)
        kappa = (2.0 * gy) / max(1e-3, Ld * Ld)

        # 4) Speed schedule (slow near goal)
        r = self.cfg.slowdown_radius_m
        speed = self.cfg.min_speed_mps + (self.cfg.max_speed_mps - self.cfg.min_speed_mps) * max(0.0, min(1.0, dist / r))

        # Convert curvature to omega: ω = κ * v
        omega_cmd = float(kappa * speed)

        reached = dist <= wp.tolerance_m
        return BearingSteer(
            distance_m=dist,
            bearing_rad=bearing,
            heading_error_rad=heading_err,
            suggested_speed_mps=(0.0 if reached else speed),
            suggested_omega_rps=(0.0 if reached else omega_cmd),
            reached=reached
        )

    def step(self) -> tuple[float, float]:
        """Compute and advance waypoint when reached. Returns (v, omega)."""
        steer = self.compute_steering()
        if steer is None:
            return (0.0, 0.0)
        if steer.reached:
            self.index += 1
            return (0.0, 0.0)
        self._last_cmd = (steer.suggested_speed_mps, steer.suggested_omega_rps)
        return self._last_cmd

    # ---------- Future: linked rovers ----------
    def set_partner_pose(self, x: float, y: float):
        """
        Placeholder for linked travel: later use (x,y) of partner to enforce
        midpoint or separation constraints in steering computation.
        """
        pass

    # ---------- Helpers ----------
    def _wp(self) -> Optional[Waypoint]:
        return self.survey.next_waypoint(self.index)