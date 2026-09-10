"""
dynamics.py — 3-DOF Point-Mass Trajectory Dynamics
====================================================
Models the forces acting on a 155 mm projectile in flight:
drag (Mach-dependent), gravity, wind, and canard lift.

Coordinate system:
  x = downrange (East)
  y = crossrange (North)
  z = altitude (Up)

References
----------
* McCoy, R. L.  *Modern Exterior Ballistics*, Schiffer, 2012.
* STANAG 4355  (Modified Point-Mass Trajectory Model)
"""

import numpy as np
from typing import Tuple, List

from .environment import ISAAtmosphere, GravityModel


class ProjectileDynamics:
    """Three-degree-of-freedom point-mass dynamics for a 155 mm shell
    with optional canard steering forces.

    Parameters
    ----------
    mass_kg : float
        Projectile mass.
    ref_area_m2 : float
        Aerodynamic reference area (cross-section).
    cd_table : list of [Mach, Cd] pairs
        Mach-dependent drag coefficient lookup.
    canard_area_m2 : float
        Total canard planform area for lift computation.
    cl_delta : float
        Canard lift-curve slope [1/rad].
    atmosphere : ISAAtmosphere
        Atmosphere model instance.
    gravity : GravityModel
        Gravity model instance.
    """

    def __init__(
        self,
        mass_kg: float,
        ref_area_m2: float,
        cd_table: List[List[float]],
        canard_area_m2: float = 0.001,
        cl_delta: float = 3.0,
        atmosphere: ISAAtmosphere | None = None,
        gravity: GravityModel | None = None,
    ):
        self.mass = mass_kg
        self.ref_area = ref_area_m2
        self.canard_area = canard_area_m2
        self.cl_delta = cl_delta
        self.atmosphere = atmosphere or ISAAtmosphere()
        self.gravity = gravity or GravityModel()

        # Pre-process Cd table for interpolation
        cd_arr = np.array(cd_table, dtype=float)
        self._mach_pts = cd_arr[:, 0]
        self._cd_pts = cd_arr[:, 1]

    # --------------------------------------------------------------------- #
    # Aerodynamic helpers                                                      #
    # --------------------------------------------------------------------- #
    def _lookup_cd(self, mach: float) -> float:
        """Linearly interpolate drag coefficient from Mach table."""
        return float(np.interp(mach, self._mach_pts, self._cd_pts))

    def compute_drag(
        self,
        velocity: np.ndarray,
        altitude_m: float,
        wind: np.ndarray | None = None,
    ) -> np.ndarray:
        """Compute aerodynamic drag force vector [N].

        Drag is computed relative to airspeed (ground velocity minus wind).

        F_drag = -½ ρ |V_air|² Cd A_ref V̂_air
        """
        if wind is None:
            wind = np.zeros(3)

        v_air = velocity - wind
        v_mag = np.linalg.norm(v_air)
        if v_mag < 1e-6:
            return np.zeros(3)

        _, _, rho, a = self.atmosphere.get_properties(altitude_m)
        mach = v_mag / a
        cd = self._lookup_cd(mach)

        q = 0.5 * rho * v_mag ** 2  # Dynamic pressure
        drag_magnitude = q * cd * self.ref_area
        drag_force = -drag_magnitude * (v_air / v_mag)

        return drag_force

    def compute_canard_force(
        self,
        deflection_pitch_rad: float,
        deflection_yaw_rad: float,
        velocity: np.ndarray,
        altitude_m: float,
    ) -> np.ndarray:
        """Compute canard-generated aerodynamic steering force.

        The canard lift acts perpendicular to the velocity vector in the
        pitch (vertical) and yaw (lateral) planes.

        F_canard = ½ ρ V² S_c C_Lδ δ   (in the corresponding plane)
        """
        v_mag = np.linalg.norm(velocity)
        if v_mag < 1e-6:
            return np.zeros(3)

        _, _, rho, _ = self.atmosphere.get_properties(altitude_m)
        q = 0.5 * rho * v_mag ** 2
        base_lift = q * self.canard_area * self.cl_delta  # [N/rad]

        # Build orthogonal unit vectors in pitch and yaw planes
        v_hat = velocity / v_mag

        # Pitch plane: force in the vertical plane containing V
        # This raises/lowers the trajectory to extend/shorten range.
        # Compute the upward component perpendicular to V in the xz-plane.
        v_horiz = np.array([velocity[0], velocity[1], 0.0])
        v_horiz_mag = np.linalg.norm(v_horiz)

        if v_horiz_mag > 1e-6:
            # Pitch force: lift the nose up (positive deflection → more range)
            # Direction is perpendicular to V, in the vertical plane of V
            up = np.array([0.0, 0.0, 1.0])
            pitch_dir = up - np.dot(up, v_hat) * v_hat
            p_norm = np.linalg.norm(pitch_dir)
            if p_norm > 1e-8:
                pitch_dir /= p_norm
            else:
                pitch_dir = np.array([0.0, 0.0, 1.0])

            # Yaw force: sideways, perpendicular to both V and up
            # This is in the horizontal plane for crossrange correction
            yaw_dir = np.cross(v_hat, up)
            y_norm = np.linalg.norm(yaw_dir)
            if y_norm > 1e-8:
                yaw_dir /= y_norm
            else:
                yaw_dir = np.array([0.0, 1.0, 0.0])
        else:
            pitch_dir = np.array([0.0, 0.0, 1.0])
            yaw_dir = np.array([0.0, 1.0, 0.0])

        f_pitch = base_lift * deflection_pitch_rad * pitch_dir
        f_yaw = base_lift * deflection_yaw_rad * yaw_dir

        return f_pitch + f_yaw

    # --------------------------------------------------------------------- #
    # Equations of motion                                                      #
    # --------------------------------------------------------------------- #
    def derivatives(
        self,
        t: float,
        state: np.ndarray,
        canard_pitch_rad: float = 0.0,
        canard_yaw_rad: float = 0.0,
        wind: np.ndarray | None = None,
    ) -> np.ndarray:
        """Compute state derivatives for RK4 integration.

        State vector: [x, y, z, vx, vy, vz]

        Returns
        -------
        dstate : np.ndarray  shape (6,)
            [vx, vy, vz, ax, ay, az]
        """
        pos = state[:3]
        vel = state[3:6]
        alt = max(pos[2], 0.0)

        if wind is None:
            wind = np.zeros(3)

        # Forces
        g = self.gravity.g(alt)
        f_gravity = np.array([0.0, 0.0, -g * self.mass])
        f_drag = self.compute_drag(vel, alt, wind)
        f_canard = self.compute_canard_force(canard_pitch_rad, canard_yaw_rad, vel, alt)

        f_total = f_gravity + f_drag + f_canard
        accel = f_total / self.mass

        return np.array([vel[0], vel[1], vel[2], accel[0], accel[1], accel[2]])
