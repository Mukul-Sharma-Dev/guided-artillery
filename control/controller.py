"""
controller.py — Flight Phase Manager & Canard Controller
==========================================================
Manages the flight phases of the projectile (boost, apogee, midcourse,
terminal) and orchestrates canard deflection command generation through
the guidance law.
"""

import numpy as np
from enum import Enum, auto
from typing import Tuple, Optional

from .guidance_model import ImpactPointPredictor, GuidanceLaw


class FlightPhase(Enum):
    """Enumeration of flight phases for the PGK."""
    BOOST_ASCENT = auto()
    APOGEE_TRANSITION = auto()
    MIDCOURSE_GUIDANCE = auto()
    TERMINAL = auto()
    POST_IMPACT = auto()


class FlightPhaseManager:
    """Determines the current flight phase based on trajectory state.

    Phase transitions
    -----------------
    BOOST_ASCENT → APOGEE_TRANSITION:  vz ≤ 0  (vertical velocity reverses)
    APOGEE_TRANSITION → MIDCOURSE_GUIDANCE:  after 0.5 s in descent
    MIDCOURSE_GUIDANCE → TERMINAL:  altitude < terminal_altitude_m
    TERMINAL → POST_IMPACT:  altitude ≤ 0

    Parameters
    ----------
    terminal_altitude_m : float
        Altitude at which terminal phase begins [m].
    """

    def __init__(self, terminal_altitude_m: float = 500.0):
        self.terminal_alt = terminal_altitude_m
        self.phase = FlightPhase.BOOST_ASCENT
        self._apogee_time: Optional[float] = None
        self._launched = False

    def update(self, state: np.ndarray, t: float) -> FlightPhase:
        """Update flight phase based on current state and time."""
        vz = state[5]
        alt = state[2]

        if not self._launched and t > 0.01:
            self._launched = True

        if self.phase == FlightPhase.BOOST_ASCENT:
            if vz <= 0.0 and self._launched:
                self.phase = FlightPhase.APOGEE_TRANSITION
                self._apogee_time = t

        elif self.phase == FlightPhase.APOGEE_TRANSITION:
            if t - self._apogee_time > 0.5:
                self.phase = FlightPhase.MIDCOURSE_GUIDANCE

        elif self.phase == FlightPhase.MIDCOURSE_GUIDANCE:
            if alt < self.terminal_alt:
                self.phase = FlightPhase.TERMINAL

        elif self.phase == FlightPhase.TERMINAL:
            if alt <= 0.0:
                self.phase = FlightPhase.POST_IMPACT

        return self.phase

    def is_guidance_active(self) -> bool:
        """Return True if guidance commands should be generated."""
        return self.phase in (FlightPhase.MIDCOURSE_GUIDANCE, FlightPhase.TERMINAL)

    def reset(self):
        self.phase = FlightPhase.BOOST_ASCENT
        self._apogee_time = None
        self._launched = False


class CanardController:
    """Top-level controller that integrates guidance law with flight
    phase management to produce canard deflection commands.

    Uses the estimated state, predicts ballistic impact, and computes
    corrective canard commands to steer the impact toward the target.

    Parameters
    ----------
    target : ndarray (3,)
        Target coordinates.
    config : dict
        Configuration dictionary with shell, canard, and guidance params.
    """

    def __init__(self, target: np.ndarray, config: dict):
        guidance_cfg = config.get("guidance", {})
        shell_cfg = config.get("shell", {})
        canard_cfg = config.get("canard", {})

        self.phase_manager = FlightPhaseManager(
            terminal_altitude_m=guidance_cfg.get("terminal_altitude_m", 500.0)
        )

        self.predictor = ImpactPointPredictor(
            mass_kg=shell_cfg.get("mass_kg", 43.2),
            ref_area_m2=shell_cfg.get("reference_area_m2", 0.018869),
            cd_table=shell_cfg.get("cd_table"),
        )

        self.target = np.array(target, dtype=float)
        self.mass = shell_cfg.get("mass_kg", 43.2)
        self.canard_area = canard_cfg.get("canard_area_m2", 0.004)
        self.cl_delta = canard_cfg.get("cl_delta", 3.5)
        self.nav_gain = guidance_cfg.get("nav_gain", 6.0)
        self.terminal_gain_mult = guidance_cfg.get("terminal_gain_multiplier", 2.0)
        self.min_tgo = guidance_cfg.get("min_time_to_go_s", 0.5)
        self.max_defl = canard_cfg.get("max_deflection_deg", 15.0)

        # Yaw gain is much lower than pitch — crossrange errors are
        # driven by wind noise, and the impact-point predictor doesn't
        # model wind, so aggressive yaw creates drift instead of
        # correcting it.
        self.yaw_gain_fraction = 0.3

        # Low-pass filter state for yaw smoothing
        self._yaw_cmd_filtered = 0.0
        self._yaw_alpha = 0.05  # Smoothing factor (lower = smoother)

    def _compute_force_per_deg(self, velocity: np.ndarray, altitude: float) -> float:
        """Compute canard force per degree at the current flight condition.

        Uses the actual velocity and an altitude-dependent density estimate
        for accurate guidance-to-deflection conversion.
        """
        v_mag = np.linalg.norm(velocity)
        if v_mag < 10.0:
            return 1.0

        # Exponential atmosphere approximation for density
        rho = 1.225 * np.exp(-altitude / 8500.0) if altitude > 0 else 1.225

        # F_per_rad = ½ ρ V² S_c C_Lδ
        f_per_rad = 0.5 * rho * v_mag**2 * self.canard_area * self.cl_delta
        f_per_deg = f_per_rad * (np.pi / 180.0)

        return max(f_per_deg, 0.1)

    def compute_commands(
        self,
        estimated_state: np.ndarray,
        t: float,
    ) -> Tuple[float, float]:
        """Compute canard pitch and yaw commands for the current state.

        Pitch uses ZEM (zero-effort miss) from the ballistic impact-point
        predictor — this corrects range errors effectively.

        Yaw uses a direct position-based crossrange error with reduced
        gain and exponential smoothing to avoid wind-noise-induced
        oscillation (the predictor doesn't model wind, so crossrange
        ZEM from prediction is noisy and creates drift).

        Parameters
        ----------
        estimated_state : ndarray (6,)
            EKF-estimated [x, y, z, vx, vy, vz].
        t : float
            Simulation time [s].

        Returns
        -------
        pitch_cmd_deg, yaw_cmd_deg : float
            Commanded canard deflections [°].
        """
        self.phase_manager.update(estimated_state, t)

        if not self.phase_manager.is_guidance_active():
            return 0.0, 0.0

        pos = estimated_state[:3]
        vel = estimated_state[3:6]

        # ── Pitch (range correction via ZEM) ──────────────────────
        impact_pt, t_go = self.predictor.predict_impact(pos, vel)
        t_go = max(t_go, self.min_tgo)

        miss_x = self.target[0] - impact_pt[0]

        gain = self.nav_gain
        if self.phase_manager.phase == FlightPhase.TERMINAL:
            gain *= self.terminal_gain_mult

        a_cmd_x = gain * miss_x / (t_go ** 2)

        # ── Yaw (crossrange correction — direct position-based) ───
        # Use current crossrange error and velocity to estimate
        # what correction is needed, rather than relying on the
        # ballistic predictor (which doesn't model wind).
        crossrange_error = self.target[1] - pos[1]
        # Scale by remaining flight fraction for proportional correction
        yaw_gain = gain * self.yaw_gain_fraction
        a_cmd_y = yaw_gain * crossrange_error / (t_go ** 2)

        # ── Convert to deflection angles ──────────────────────────
        f_per_deg = self._compute_force_per_deg(vel, pos[2])
        accel_per_deg = f_per_deg / self.mass

        pitch_cmd = a_cmd_x / max(accel_per_deg, 1e-6)

        yaw_cmd_raw = a_cmd_y / max(accel_per_deg, 1e-6)
        # Exponential smoothing to prevent yaw oscillation
        self._yaw_cmd_filtered += self._yaw_alpha * (yaw_cmd_raw - self._yaw_cmd_filtered)
        yaw_cmd = self._yaw_cmd_filtered

        # Clamp to physical limits
        pitch_cmd = np.clip(pitch_cmd, -self.max_defl, self.max_defl)
        yaw_cmd = np.clip(yaw_cmd, -self.max_defl, self.max_defl)

        return float(pitch_cmd), float(yaw_cmd)

    def get_phase(self) -> FlightPhase:
        return self.phase_manager.phase

    def reset(self):
        self.phase_manager.reset()
        self._yaw_cmd_filtered = 0.0
