"""
actuator_model.py — Canard Actuator Dynamics
==============================================
First-order-lag actuator with rate limiting and deflection saturation,
modelling the physical response of canard servo mechanisms.
"""

import numpy as np
from typing import Tuple


class CanardActuator:
    """Models a pair of coupled pitch/yaw canard actuators with
    deflection limits, slew-rate saturation, and first-order lag.

    Parameters
    ----------
    max_deflection_deg : float
        Hard stop deflection limit [°].
    slew_rate_deg_s : float
        Maximum angular velocity of the actuator [°/s].
    time_constant_s : float
        First-order lag time constant τ [s].
    """

    def __init__(
        self,
        max_deflection_deg: float = 15.0,
        slew_rate_deg_s: float = 300.0,
        time_constant_s: float = 0.02,
    ):
        self.max_defl = max_deflection_deg
        self.slew_rate = slew_rate_deg_s
        self.tau = time_constant_s

        # Current actuator state [°]
        self.pitch_deg = 0.0
        self.yaw_deg = 0.0

    def command(
        self,
        desired_pitch_deg: float,
        desired_yaw_deg: float,
        dt: float,
    ) -> Tuple[float, float]:
        """Process a canard deflection command through actuator dynamics.

        Steps
        -----
        1. Clamp desired values to ±max_deflection.
        2. Apply rate limiting (slew rate).
        3. Apply first-order lag.
        4. Return actual deflection.

        Parameters
        ----------
        desired_pitch_deg, desired_yaw_deg : float
            Commanded deflections [°].
        dt : float
            Time step [s].

        Returns
        -------
        actual_pitch_deg, actual_yaw_deg : float
            Actuator output deflections [°].
        """
        # Clamp command
        cmd_p = np.clip(desired_pitch_deg, -self.max_defl, self.max_defl)
        cmd_y = np.clip(desired_yaw_deg, -self.max_defl, self.max_defl)

        # Rate limiting
        max_delta = self.slew_rate * dt

        delta_p = cmd_p - self.pitch_deg
        delta_p = np.clip(delta_p, -max_delta, max_delta)

        delta_y = cmd_y - self.yaw_deg
        delta_y = np.clip(delta_y, -max_delta, max_delta)

        # First-order lag:  x_new = x + (x_cmd - x) * (dt / τ)
        alpha = min(dt / self.tau, 1.0)  # Avoid overshoot at large dt
        self.pitch_deg += delta_p * alpha
        self.yaw_deg += delta_y * alpha

        # Final saturation guard
        self.pitch_deg = np.clip(self.pitch_deg, -self.max_defl, self.max_defl)
        self.yaw_deg = np.clip(self.yaw_deg, -self.max_defl, self.max_defl)

        return float(self.pitch_deg), float(self.yaw_deg)

    def reset(self):
        """Reset actuator to zero deflection (for Monte Carlo reuse)."""
        self.pitch_deg = 0.0
        self.yaw_deg = 0.0

    def get_state(self) -> Tuple[float, float]:
        """Return current (pitch, yaw) deflection in degrees."""
        return self.pitch_deg, self.yaw_deg
