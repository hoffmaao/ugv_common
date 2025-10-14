from __future__ import annotations
from dataclasses import dataclass, field
from typing import Optional, Callable
import numpy as np
import time

from .types import Pose2D, Twist2D, Cov2D, FixStatus
from .geodesy import GeoOrigin, wgs84_to_enu

# --------------------- EnKF (5D vehicle model) ---------------------

class EnKF:
    """
    Ensemble Kalman Filter for rover state x = [x, y, yaw, v, omega].
    - Process model: constant v, omega with noise; kinematic update.
    - Measurements (examples):
        * GNSS position: z = [x, y]
        * GNSS course (headMot) when moving: z = [yaw]
        * Wheel odom speed: z = [v]
        * IMU yaw rate: z = [omega]
    """
    def __init__(self, N: int = 50, q_diag=(0.01, 0.01, 0.001, 0.05, 0.02)):
        self.N = N
        self.nx = 5
        self.X = np.zeros((self.nx, N))  # ensembles as columns
        self.Q = np.diag(q_diag)         # process noise per step (units^2)
        self._rng = np.random.default_rng()

    def set_mean(self, mu: np.ndarray, P: np.ndarray | None = None):
        assert mu.shape == (self.nx,)
        if P is None:
            P = np.diag([0.5, 0.5, 0.2, 0.5, 0.2])
        L = np.linalg.cholesky(P + 1e-9 * np.eye(self.nx))
        self.X = mu.reshape(-1, 1) + L @ self._rng.standard_normal((self.nx, self.N))

    def mean(self) -> np.ndarray:
        return np.mean(self.X, axis=1)

    def cov(self) -> np.ndarray:
        Xm = self.X - self.mean().reshape(-1, 1)
        return (Xm @ Xm.T) / (self.N - 1)

    def predict(self, dt: float):
        # Process noise
        q = np.linalg.cholesky(self.Q) @ self._rng.standard_normal((self.nx, self.N))
        # State unpack
        x, y, yaw, v, omega = self.X
        # Kinematic bicycle (point mass) update
        x_new   = x + v * np.cos(yaw) * dt
        y_new   = y + v * np.sin(yaw) * dt
        yaw_new = yaw + omega * dt
        v_new   = v                    # random walk; noise will diffuse
        om_new  = omega
        self.X = np.vstack((x_new, y_new, yaw_new, v_new, om_new)) + q

    def update_linear(self, z: np.ndarray, H: np.ndarray, R: np.ndarray):
        """
        Linear measurement model: z = H x + noise, with perturbed obs EnKF.
        """
        assert z.ndim == 1
        # Predicted obs for each ensemble
        Y = H @ self.X
        # Perturbed observations
        v = np.linalg.cholesky(R) @ self._rng.standard_normal((z.size, self.N))
        Z = z.reshape(-1, 1) + v
        # Statistics
        Xbar = self.mean().reshape(-1, 1)
        Ybar = np.mean(Y, axis=1, keepdims=True)
        Xm = self.X - Xbar
        Ym = Y - Ybar
        Pxy = (Xm @ Ym.T) / (self.N - 1)
        Pyy = (Ym @ Ym.T) / (self.N - 1)
        K = Pxy @ np.linalg.pinv(Pyy + 1e-9 * np.eye(Pyy.shape[0]))
        # Update ensembles
        self.X = self.X + K @ (Z - Y)

# --------------------- Rover state wrapper ---------------------

@dataclass
class RoverState:
    """
    Maintains rover pose (map/ENU), velocity, and covariances.
    Fusion is done via an internal EnKF on state [x, y, yaw, v, omega].
    """
    origin: GeoOrigin
    pose: Pose2D = field(default_factory=Pose2D)
    twist: Twist2D = field(default_factory=Twist2D)
    cov: Cov2D = field(default_factory=Cov2D)
    fix: FixStatus = FixStatus.NO_FIX
    last_update_s: float = field(default_factory=lambda: time.time())
    enkf: EnKF = field(default_factory=lambda: EnKF(N=60))

    def __post_init__(self):
        # Initialize EnKF around starting pose
        mu = np.array([self.pose.x, self.pose.y, self.pose.yaw, self.twist.v, self.twist.omega])
        P0 = np.diag([4.0, 4.0, 0.5, 1.0, 0.5])
        self.enkf.set_mean(mu, P0)

    # ---------- Propagation ----------
    def propagate(self, dt: float):
        self.enkf.predict(dt)
        self._sync_from_filter()

    # ---------- GNSS update (UBX-NAV-PVT) ----------
    def update_from_gnss(self,
                         lat_deg: float, lon_deg: float, alt_m: float,
                         hAcc_m: float,
                         headMot_rad: Optional[float] = None,
                         gSpeed_mps: Optional[float] = None,
                         fix_status: FixStatus = FixStatus.FIX_3D):
        ex, ny, _ = wgs84_to_enu(lat_deg, lon_deg, alt_m, self.origin)
        # Position measurement z = [x, y]
        z = np.array([ex, ny])
        H = np.zeros((2, 5)); H[0, 0] = 1.0; H[1, 1] = 1.0
        # Measurement covariance from hAcc (1-sigma horizontal accuracy)
        sigma = max(0.5, hAcc_m)  # floor at 0.5 m to avoid overconfidence
        R = np.diag([sigma**2, sigma**2])
        self.enkf.update_linear(z, H, R)

        # Optional: course over ground (headMot) if moving
        if headMot_rad is not None and gSpeed_mps is not None and gSpeed_mps > 0.25:
            z_yaw = np.array([headMot_rad])
            H_yaw = np.zeros((1, 5)); H_yaw[0, 2] = 1.0
            R_yaw = np.array([[np.deg2rad(10.0)**2]])  # ~10 deg when moving
            self.enkf.update_linear(z_yaw, H_yaw, R_yaw)

        # Optional: speed update from gSpeed
        if gSpeed_mps is not None:
            z_v = np.array([gSpeed_mps])
            H_v = np.zeros((1, 5)); H_v[0, 3] = 1.0
            R_v = np.array([[0.25**2]])  # 0.25 m/s sigma
            self.enkf.update_linear(z_v, H_v, R_v)

        self.fix = fix_status
        self._sync_from_filter()

    # ---------- IMU yaw-rate update ----------
    def update_from_imu(self, gyro_z_rps: float, sigma_rps: float = 0.05):
        z = np.array([gyro_z_rps])
        H = np.zeros((1, 5)); H[0, 4] = 1.0
        R = np.array([[sigma_rps**2]])
        self.enkf.update_linear(z, H, R)
        self._sync_from_filter()

    # ---------- Wheel odometry (speed) ----------
    def update_from_wheel_odom(self, speed_mps: float, sigma_mps: float = 0.1):
        z = np.array([speed_mps])
        H = np.zeros((1, 5)); H[0, 3] = 1.0
        R = np.array([[sigma_mps**2]])
        self.enkf.update_linear(z, H, R)
        self._sync_from_filter()

    # ---------- Helpers ----------
    def _sync_from_filter(self):
        mu = self.enkf.mean()
        self.pose.x, self.pose.y, self.pose.yaw = float(mu[0]), float(mu[1]), float(mu[2])
        self.twist.v, self.twist.omega = float(mu[3]), float(mu[4])
        P = self.enkf.cov()
        self.cov.var_x, self.cov.var_y, self.cov.var_yaw = float(P[0,0]), float(P[1,1]), float(P[2,2])
        self.last_update_s = time.time()