"""
sensors.py — Sensor Simulation Models
=======================================
Realistic stochastic sensor models for IMU (accelerometer + gyroscope),
GNSS/GPS receiver, and barometric altimeter, including bias, noise, dropout,
and saturation effects.

References
----------
* IEEE Std 952-1997 — IMU terminology & error models
* Groves, P. D.  *Principles of GNSS, Inertial, and Multisensor
  Integrated Navigation Systems*, 2nd ed., Artech House, 2013.
"""

import numpy as np
from typing import Tuple, Optional


class IMUSensor:
    """Simulated tactical-grade MEMS IMU (3-axis accel + gyro).

    Error model:  measurement = truth + bias + white_noise
    Acceleration values are clipped to the sensor saturation limit.

    Parameters
    ----------
    accel_noise_sigma : float
        Accelerometer white noise σ [m/s²].
    gyro_noise_sigma : float
        Gyroscope white noise σ [rad/s].
    accel_bias : float
        Constant accelerometer bias [m/s²] (applied in each axis).
    gyro_bias : float
        Constant gyroscope bias [rad/s].
    saturation_g : float
        Maximum measurable acceleration [g].
    seed : int or None
        Random seed for reproducibility.
    """

    def __init__(
        self,
        accel_noise_sigma: float = 5.0,
        gyro_noise_sigma: float = 0.01,
        accel_bias: float = 0.5,
        gyro_bias: float = 0.001,
        saturation_g: float = 20_000,
        seed: int | None = None,
    ):
        self.accel_noise_sigma = accel_noise_sigma
        self.gyro_noise_sigma = gyro_noise_sigma
        self.saturation_ms2 = saturation_g * 9.80665
        self.rng = np.random.default_rng(seed)

        # Fixed biases (randomized direction per axis at init)
        self.accel_bias_vec = self.rng.uniform(-1, 1, 3)
        self.accel_bias_vec = self.accel_bias_vec / (np.linalg.norm(self.accel_bias_vec) + 1e-12) * accel_bias

        self.gyro_bias_vec = self.rng.uniform(-1, 1, 3)
        self.gyro_bias_vec = self.gyro_bias_vec / (np.linalg.norm(self.gyro_bias_vec) + 1e-12) * gyro_bias

    def measure_accel(self, true_accel: np.ndarray) -> np.ndarray:
        """Return noisy, biased, saturated accelerometer measurement.

        Parameters
        ----------
        true_accel : ndarray shape (3,)
            True specific-force acceleration [m/s²].

        Returns
        -------
        meas : ndarray shape (3,)
            Measured acceleration [m/s²].
        """
        noise = self.rng.normal(0.0, self.accel_noise_sigma, 3)
        meas = true_accel + self.accel_bias_vec + noise
        # Saturation clipping
        meas = np.clip(meas, -self.saturation_ms2, self.saturation_ms2)
        return meas

    def measure_gyro(self, true_angular_rate: np.ndarray | None = None) -> np.ndarray:
        """Return noisy gyroscope measurement (for completeness).

        Parameters
        ----------
        true_angular_rate : ndarray shape (3,) or None
            True angular rate [rad/s].  Default zeros (non-spinning in 3-DOF).

        Returns
        -------
        meas : ndarray shape (3,)
        """
        if true_angular_rate is None:
            true_angular_rate = np.zeros(3)
        noise = self.rng.normal(0.0, self.gyro_noise_sigma, 3)
        return true_angular_rate + self.gyro_bias_vec + noise


class GPSSensor:
    """Simulated GNSS / GPS receiver with position and velocity noise,
    update-rate limiting, and stochastic dropout modeling.

    Parameters
    ----------
    pos_noise_sigma : float
        Position measurement noise σ [m] (isotropic, 3-axis).
    vel_noise_sigma : float
        Velocity measurement noise σ [m/s].
    update_rate_hz : float
        GPS fix rate [Hz].
    dropout_prob : float
        Per-epoch probability of a missed fix (0 – 1).
    seed : int or None
        Random seed.
    """

    def __init__(
        self,
        pos_noise_sigma: float = 2.0,
        vel_noise_sigma: float = 0.1,
        update_rate_hz: float = 10.0,
        dropout_prob: float = 0.02,
        seed: int | None = None,
    ):
        self.pos_sigma = pos_noise_sigma
        self.vel_sigma = vel_noise_sigma
        self.dt_gps = 1.0 / update_rate_hz
        self.dropout_prob = dropout_prob
        self.rng = np.random.default_rng(seed)
        self._last_update_time = -999.0

    def measure(
        self,
        true_position: np.ndarray,
        true_velocity: np.ndarray,
        t: float,
    ) -> Tuple[Optional[np.ndarray], Optional[np.ndarray], bool]:
        """Return GPS position/velocity fix or None on dropout / off-rate.

        Parameters
        ----------
        true_position : ndarray (3,)
        true_velocity : ndarray (3,)
        t : float   Simulation time [s].

        Returns
        -------
        pos_meas : ndarray (3,) or None
        vel_meas : ndarray (3,) or None
        is_valid : bool
        """
        # Rate limiting — only produce a fix at GPS update intervals
        if (t - self._last_update_time) < self.dt_gps - 1e-9:
            return None, None, False

        self._last_update_time = t

        # Random dropout
        if self.rng.random() < self.dropout_prob:
            return None, None, False

        pos_meas = true_position + self.rng.normal(0.0, self.pos_sigma, 3)
        vel_meas = true_velocity + self.rng.normal(0.0, self.vel_sigma, 3)
        return pos_meas, vel_meas, True

    def reset(self):
        """Reset last-update timer (for Monte Carlo reuse)."""
        self._last_update_time = -999.0


class BaroSensor:
    """Simulated barometric altitude sensor.

    Converts true altitude to a noisy barometric altitude measurement
    using the ISA pressure–altitude relationship plus additive noise.

    Parameters
    ----------
    alt_noise_sigma : float
        Altitude measurement noise σ [m].
    pressure_bias_pa : float
        Static bias in pressure reading [Pa], mapped to ~0.08 m/Pa altitude error.
    seed : int or None
    """

    def __init__(
        self,
        alt_noise_sigma: float = 3.0,
        pressure_bias_pa: float = 50.0,
        seed: int | None = None,
    ):
        self.alt_noise_sigma = alt_noise_sigma
        # Convert pressure bias to approximate altitude bias
        # Near sea level: Δh ≈ -Δp / (ρ g) ≈ -Δp / 12.01  →  ~4.2 m for 50 Pa
        self.alt_bias = -pressure_bias_pa / 12.01
        self.rng = np.random.default_rng(seed)

    def measure(self, true_altitude: float) -> float:
        """Return noisy barometric altitude measurement [m]."""
        noise = self.rng.normal(0.0, self.alt_noise_sigma)
        return true_altitude + self.alt_bias + noise
