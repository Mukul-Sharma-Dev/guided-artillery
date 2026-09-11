"""
controller.py — Flight Phase Manager & Canard Controller
==========================================================
Manages the flight phases of the projectile (boost, apogee, midcourse,
terminal) and orchestrates canard deflection command generation through
the guidance law.

Design Note — Pitch-Primary Guidance
-------------------------------------
Real PGK systems (M1156-class) primarily correct *range* errors using
pitch canard deflections. Crossrange (yaw) correction is minimal because:
  1. The impact-point predictor doesn't model wind → yaw ZEM is noisy.
  2. Aggressive yaw creates lateral drift worse than the original error.
  3. Gun-lay azimuth accuracy (~1.5 mil) keeps crossrange errors small.

Yaw is limited to a small proportional correction only in the terminal
phase (last few seconds) when the crossrange error can be measured
directly from position with high confidence.
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
    """Top-level controller — pitch-primary guidance with minimal yaw.

    Pitch uses ZEM (zero-effort miss) from the ballistic impact-point
    predictor to correct range errors. This is the primary channel.

    Yaw uses a small direct position-based crossrange correction only
    in the terminal phase. In midcourse, yaw is zero to avoid the
    wind-noise-induced oscillation that creates lateral drift.

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

        # Yaw PD controller gains (crossrange correction)
        # Kp: position-proportional gain, Kd: velocity-damping gain
        self.yaw_kp = guidance_cfg.get("yaw_kp", 3.0)
        self.yaw_kd = guidance_cfg.get("yaw_kd", 2.0)

        # Track guidance activation time for ramp-up
        self._guidance_start_time = None

    def _compute_force_per_deg(self, velocity: np.ndarray, altitude: float) -> float:
        """Compute canard force per degree at current flight conditions."""
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

        Pitch: ZEM-based proportional navigation (impact predictor).
        Yaw:   PD controller on crossrange error (direct position + velocity).

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
            self._guidance_start_time = None
            return 0.0, 0.0

        # Track when guidance first activates (for ramp-up)
        if self._guidance_start_time is None:
            self._guidance_start_time = t

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

        # ── Convert pitch to deflection angle ─────────────────────
        f_per_deg = self._compute_force_per_deg(vel, pos[2])
        accel_per_deg = f_per_deg / self.mass

        pitch_cmd = a_cmd_x / max(accel_per_deg, 1e-6)

        # ── Yaw (PD crossrange controller — midcourse + terminal) ─
        #
        # Uses direct position + velocity instead of impact predictor:
        #   e_y  = target_y − pos_y            (position error)
        #   vy_r = e_y / t_go                  (required crossrange velocity)
        #   Δvy  = vy_r − vel_y                (velocity correction needed)
        #   a_y  = K_p · e_y / t_go² + K_d · Δvy / t_go
        #
        # This is robust to wind because it measures actual position
        # and velocity, not predicted impact (which ignores wind).

        crossrange_error = self.target[1] - pos[1]
        vy_required = crossrange_error / t_go
        vy_correction = vy_required - vel[1]

        # PD gains
        kp = self.yaw_kp
        kd = self.yaw_kd

        # Terminal phase: increase gains for final correction
        if self.phase_manager.phase == FlightPhase.TERMINAL:
            kp *= self.terminal_gain_mult
            kd *= self.terminal_gain_mult

        a_cmd_y = kp * crossrange_error / (t_go ** 2) + kd * vy_correction / t_go

        # Ramp gain smoothly over first 5 seconds of guidance to avoid transients
        dt_active = t - self._guidance_start_time
        ramp = min(dt_active / 5.0, 1.0)
        a_cmd_y *= ramp

        # Negate because canard yaw_dir = cross(v_hat, up) = [0, -1, 0]
        # so positive deflection creates force in -y. We need opposite sign.
        yaw_cmd = -a_cmd_y / max(accel_per_deg, 1e-6)

        # Clamp to physical limits
        pitch_cmd = np.clip(pitch_cmd, -self.max_defl, self.max_defl)
        yaw_cmd = np.clip(yaw_cmd, -self.max_defl, self.max_defl)

        return float(pitch_cmd), float(yaw_cmd)

    def get_phase(self) -> FlightPhase:
        return self.phase_manager.phase

    def reset(self):
        self.phase_manager.reset()
        self._guidance_start_time = None
