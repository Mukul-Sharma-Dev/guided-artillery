"""
guidance_model.py — Impact Point Prediction & Course Correction
================================================================
Implements fast-forward ballistic impact prediction and proportional
navigation guidance for trajectory correction.

References
----------
* Zarchan, P.  *Tactical and Strategic Missile Guidance*, AIAA, 2012.
* Siouris, G. M.  *Missile Guidance and Control Systems*, Springer, 2004.
"""

import numpy as np
from typing import Tuple


class ImpactPointPredictor:
    """Predicts the ballistic impact point by forward-propagating the
    current state under gravity and drag alone (no canard input).

    Uses a fast Euler integration with coarse time step for real-time
    onboard computation feasibility.

    Parameters
    ----------
    mass_kg : float
        Projectile mass.
    ref_area_m2 : float
        Aerodynamic reference area.
    cd_table : list of [Mach, Cd]
        Mach-dependent drag table.
    dt_predict : float
        Integration step for prediction [s]  (coarse, e.g. 0.1 s).
    """

    def __init__(
        self,
        mass_kg: float = 43.2,
        ref_area_m2: float = 0.018869,
        cd_table: list | None = None,
        dt_predict: float = 0.1,
    ):
        self.mass = mass_kg
        self.ref_area = ref_area_m2
        self.dt = dt_predict
        self.g = 9.80665

        if cd_table is None:
            cd_table = [
                [0.0, 0.15], [0.9, 0.25], [1.0, 0.35],
                [1.2, 0.33], [2.0, 0.25], [3.0, 0.20],
            ]
        cd_arr = np.array(cd_table, dtype=float)
        self._mach_pts = cd_arr[:, 0]
        self._cd_pts = cd_arr[:, 1]

    def _cd(self, mach: float) -> float:
        return float(np.interp(mach, self._mach_pts, self._cd_pts))

    def predict_impact(
        self,
        position: np.ndarray,
        velocity: np.ndarray,
    ) -> Tuple[np.ndarray, float]:
        """Forward-propagate ballistic trajectory until ground impact (z ≤ 0).

        Parameters
        ----------
        position : ndarray (3,)   Current [x, y, z].
        velocity : ndarray (3,)   Current [vx, vy, vz].

        Returns
        -------
        impact_point : ndarray (3,)
            Predicted ground impact coordinates [x, y, 0].
        time_to_impact : float
            Estimated time remaining to impact [s].
        """
        pos = position.copy()
        vel = velocity.copy()
        t_go = 0.0

        # ISA sea-level approximation for fast prediction
        rho_sl = 1.225
        a_sl = 340.29

        for _ in range(50_000):  # Safety cap
            v_mag = np.linalg.norm(vel)
            if v_mag < 1e-3:
                break

            mach = v_mag / a_sl
            cd = self._cd(mach)

            # Simple altitude-dependent density (exponential approximation)
            rho = rho_sl * np.exp(-pos[2] / 8500.0) if pos[2] > 0 else rho_sl
            q = 0.5 * rho * v_mag ** 2
            drag_accel = (q * cd * self.ref_area) / self.mass
            a_drag = -drag_accel * (vel / v_mag)

            accel = a_drag + np.array([0.0, 0.0, -self.g])

            # Euler step
            vel = vel + accel * self.dt
            pos = pos + vel * self.dt
            t_go += self.dt

            if pos[2] <= 0.0:
                # Linear interpolation to exact ground crossing
                if vel[2] != 0:
                    dt_back = pos[2] / (-vel[2])  # Time past ground
                    pos -= vel * dt_back
                    t_go -= dt_back
                pos[2] = 0.0
                break

        return pos, max(t_go, 0.01)


class GuidanceLaw:
    """Proportional Navigation (PN) guidance for trajectory correction.

    Computes commanded lateral accelerations to steer the predicted
    impact point toward the target, then converts to canard deflections.

    The correction is based on the "zero-effort miss" (ZEM) concept:
        a_cmd = N × ZEM / t_go²

    Parameters
    ----------
    target : ndarray (3,)
        Target coordinates [x, y, z].
    nav_gain : float
        Navigation constant N (typically 3–5).
    terminal_gain_mult : float
        Gain multiplier in terminal phase for more aggressive correction.
    min_tgo : float
        Minimum time-to-go to avoid singularity [s].
    canard_force_per_deg : float
        Approximate force per degree of canard deflection [N/°],
        used to convert acceleration commands to deflection angles.
    """

    def __init__(
        self,
        target: np.ndarray,
        nav_gain: float = 4.0,
        terminal_gain_mult: float = 1.5,
        min_tgo: float = 0.5,
        canard_force_per_deg: float = 50.0,
        mass_kg: float = 43.2,
    ):
        self.target = np.array(target, dtype=float)
        self.N = nav_gain
        self.terminal_mult = terminal_gain_mult
        self.min_tgo = min_tgo
        self.force_per_deg = canard_force_per_deg
        self.mass = mass_kg

    def compute_correction(
        self,
        predicted_impact: np.ndarray,
        time_to_impact: float,
        altitude: float,
        terminal_alt: float = 500.0,
    ) -> Tuple[float, float]:
        """Compute canard pitch and yaw deflection commands.

        Parameters
        ----------
        predicted_impact : ndarray (3,)
            Predicted unguided impact point.
        time_to_impact : float
            Estimated time to ground [s].
        altitude : float
            Current altitude [m]  (for terminal gain scheduling).
        terminal_alt : float
            Altitude below which terminal gain is applied [m].

        Returns
        -------
        pitch_cmd_deg, yaw_cmd_deg : float
            Canard deflection commands [°].
        """
        t_go = max(time_to_impact, self.min_tgo)

        # Zero-effort miss vector (in ground plane)
        miss = self.target - predicted_impact  # [Δx, Δy, Δz]

        # Gain scheduling
        gain = self.N
        if altitude < terminal_alt:
            gain *= self.terminal_mult

        # Required corrective acceleration  a = N × miss / t_go²
        a_cmd_x = gain * miss[0] / (t_go ** 2)  # Downrange correction (pitch)
        a_cmd_y = gain * miss[1] / (t_go ** 2)  # Crossrange correction (yaw)

        # Convert acceleration → force → canard deflection [°]
        f_x = a_cmd_x * self.mass
        f_y = a_cmd_y * self.mass

        pitch_cmd_deg = f_x / max(self.force_per_deg, 1e-6)
        yaw_cmd_deg = f_y / max(self.force_per_deg, 1e-6)

        return float(pitch_cmd_deg), float(yaw_cmd_deg)
