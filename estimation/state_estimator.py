"""
state_estimator.py — Extended Kalman Filter (EKF) for Navigation
==================================================================
Fuses high-rate IMU predictions with low-rate GPS and barometric
corrections to produce a smoothed position/velocity estimate with
uncertainty (covariance) bounds.

State vector:  x = [x, y, z, vx, vy, vz]ᵀ   (6 states)

References
----------
* Bar-Shalom, Y. et al.  *Estimation with Applications to Tracking
  and Navigation*, Wiley, 2001.
* Groves, P. D.  *Principles of GNSS, Inertial, and Multisensor
  Integrated Navigation Systems*, 2nd ed., Artech House, 2013.
"""

import numpy as np
from typing import Optional, Tuple

from .filters import matrix_inverse, ensure_positive_definite, mahalanobis_distance


class ExtendedKalmanFilter:
    """6-state position-velocity EKF for projectile navigation.

    Prediction model (constant-acceleration within dt):
        x_{k+1} = x_k + v_k·dt + ½ a·dt²
        v_{k+1} = v_k + a·dt

    Correction sources:
        - GPS: observes [x, y, z, vx, vy, vz]
        - Barometer: observes [z]

    Parameters
    ----------
    process_noise_accel : float
        Process noise spectral density for acceleration [m/s²].
    gps_pos_noise : float
        GPS position measurement noise σ [m].
    gps_vel_noise : float
        GPS velocity measurement noise σ [m/s].
    baro_noise : float
        Barometric altitude noise σ [m].
    gate_threshold : float
        Mahalanobis distance gate for outlier rejection.
    """

    N_STATES = 6  # [x, y, z, vx, vy, vz]

    def __init__(
        self,
        process_noise_accel: float = 10.0,
        gps_pos_noise: float = 2.0,
        gps_vel_noise: float = 0.1,
        baro_noise: float = 3.0,
        gate_threshold: float = 5.0,
    ):
        # State and covariance
        self.x = np.zeros(self.N_STATES)
        self.P = np.diag([100.0, 100.0, 100.0, 50.0, 50.0, 50.0])  # Initial uncertainty

        # Process noise (continuous-time, discretised per predict step)
        self.q_accel = process_noise_accel

        # Measurement noise covariance matrices
        self.R_gps = np.diag([
            gps_pos_noise ** 2, gps_pos_noise ** 2, gps_pos_noise ** 2,
            gps_vel_noise ** 2, gps_vel_noise ** 2, gps_vel_noise ** 2,
        ])
        self.R_baro = np.array([[baro_noise ** 2]])

        self.gate = gate_threshold

    def initialize(self, position: np.ndarray, velocity: np.ndarray):
        """Set the initial state from a known launch condition."""
        self.x[:3] = position
        self.x[3:6] = velocity
        self.P = np.diag([1.0, 1.0, 1.0, 1.0, 1.0, 1.0])

    # ------------------------------------------------------------------ #
    # Prediction step (IMU-rate, e.g. 100 Hz)                              #
    # ------------------------------------------------------------------ #
    def predict(self, accel: np.ndarray, dt: float):
        """Propagate state forward using measured acceleration.

        Parameters
        ----------
        accel : ndarray (3,)
            Measured acceleration (specific force + gravity) [m/s²].
        dt : float
            Time step [s].
        """
        # State transition
        x_new = self.x.copy()
        x_new[0] += self.x[3] * dt + 0.5 * accel[0] * dt ** 2
        x_new[1] += self.x[4] * dt + 0.5 * accel[1] * dt ** 2
        x_new[2] += self.x[5] * dt + 0.5 * accel[2] * dt ** 2
        x_new[3] += accel[0] * dt
        x_new[4] += accel[1] * dt
        x_new[5] += accel[2] * dt
        self.x = x_new

        # State transition Jacobian F (6×6)
        F = np.eye(self.N_STATES)
        F[0, 3] = dt
        F[1, 4] = dt
        F[2, 5] = dt

        # Process noise Q — constant-acceleration noise model
        #   Q_block = q² [ dt³/3  dt²/2 ;  dt²/2  dt ]   per axis
        q = self.q_accel
        dt2 = dt * dt
        dt3 = dt2 * dt
        Q = np.zeros((self.N_STATES, self.N_STATES))
        for i in range(3):
            Q[i, i] = q ** 2 * dt3 / 3.0
            Q[i, i + 3] = q ** 2 * dt2 / 2.0
            Q[i + 3, i] = q ** 2 * dt2 / 2.0
            Q[i + 3, i + 3] = q ** 2 * dt

        self.P = F @ self.P @ F.T + Q
        self.P = ensure_positive_definite(self.P)

    # ------------------------------------------------------------------ #
    # GPS correction step (10 Hz)                                          #
    # ------------------------------------------------------------------ #
    def update_gps(self, gps_pos: np.ndarray, gps_vel: np.ndarray) -> bool:
        """Correct state with GPS position and velocity observation.

        Returns True if the measurement was accepted (passed gating).
        """
        H = np.eye(self.N_STATES)  # Full-state observation
        z = np.concatenate([gps_pos, gps_vel])

        innovation = z - H @ self.x
        S = H @ self.P @ H.T + self.R_gps

        # Mahalanobis gating
        md = mahalanobis_distance(innovation, S)
        if md > self.gate:
            return False  # Reject outlier

        # Kalman gain
        K = self.P @ H.T @ matrix_inverse(S)

        # State update
        self.x = self.x + K @ innovation

        # Covariance update — Joseph form for numerical stability
        # P = (I - KH) P (I - KH)^T + K R K^T
        I_KH = np.eye(self.N_STATES) - K @ H
        self.P = I_KH @ self.P @ I_KH.T + K @ self.R_gps @ K.T
        self.P = ensure_positive_definite(self.P)
        return True

    # ------------------------------------------------------------------ #
    # Barometric altitude correction step (20 Hz)                          #
    # ------------------------------------------------------------------ #
    def update_baro(self, baro_alt: float) -> bool:
        """Correct state with barometric altitude observation."""
        H = np.zeros((1, self.N_STATES))
        H[0, 2] = 1.0  # Observes z only

        z = np.array([baro_alt])
        innovation = z - H @ self.x
        S = H @ self.P @ H.T + self.R_baro

        md = mahalanobis_distance(innovation.ravel(), S)
        if md > self.gate:
            return False

        K = self.P @ H.T @ matrix_inverse(S)
        self.x = self.x + (K @ innovation).ravel()

        I_KH = np.eye(self.N_STATES) - K @ H
        self.P = I_KH @ self.P @ I_KH.T + K @ self.R_baro @ K.T
        self.P = ensure_positive_definite(self.P)
        return True

    # ------------------------------------------------------------------ #
    # Accessors                                                            #
    # ------------------------------------------------------------------ #
    def get_state(self) -> Tuple[np.ndarray, np.ndarray]:
        """Return (state_vector, covariance_diagonal)."""
        return self.x.copy(), np.diag(self.P).copy()

    def get_position(self) -> np.ndarray:
        return self.x[:3].copy()

    def get_velocity(self) -> np.ndarray:
        return self.x[3:6].copy()

    def get_position_sigma(self) -> np.ndarray:
        """Return 1-σ position uncertainty [m] per axis."""
        return np.sqrt(np.diag(self.P)[:3])

    def reset(self):
        """Reset filter to default state (for Monte Carlo reuse)."""
        self.x = np.zeros(self.N_STATES)
        self.P = np.diag([100.0, 100.0, 100.0, 50.0, 50.0, 50.0])
