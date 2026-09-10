"""
environment.py — Atmospheric, Gravity, Wind, and Earth Models
===============================================================
International Standard Atmosphere (ISA) with non-standard temperature,
latitude-dependent gravity with Coriolis effect, and stochastic wind/gust
environment for the 155 mm PGK flight simulator.

References
----------
* ICAO Standard Atmosphere (Doc 7488/3)
* WGS-84 gravity model
* MIL-HDBK-310 wind profile guidance
* Somigliana gravity formula
"""

import numpy as np
from typing import Tuple

# ---------------------------------------------------------------------------
# Physical Constants
# ---------------------------------------------------------------------------
R_SPECIFIC_AIR = 287.05287   # Specific gas constant for dry air [J/(kg·K)]
GAMMA_AIR = 1.4              # Ratio of specific heats for air
G0 = 9.80665                 # Standard gravity at sea level [m/s²]
R_EARTH = 6_371_000.0        # Mean Earth radius [m]
OMEGA_EARTH = 7.2921159e-5   # Earth rotation rate [rad/s]


class ISAAtmosphere:
    """International Standard Atmosphere up to 20 km altitude with
    non-standard temperature deviation support.

    Layer structure (troposphere + lower stratosphere):
      - 0 – 11 000 m:  linear lapse rate  L = 6.5 K/km
      - 11 000 – 20 000 m:  isothermal at 216.65 K

    Parameters
    ----------
    T0 : float
        Sea-level temperature [K]  (default 288.15 K).
    P0 : float
        Sea-level pressure [Pa]  (default 101 325 Pa).
    delta_T : float
        Non-standard temperature deviation [K]. Positive = hot day,
        negative = cold day. Applied uniformly to all altitudes.
    """

    def __init__(self, T0: float = 288.15, P0: float = 101_325.0,
                 delta_T: float = 0.0):
        self.T0 = T0 + delta_T
        self.P0 = P0
        self.delta_T = delta_T
        self.L = 0.0065  # Temperature lapse rate [K/m]
        self.h_tropopause = 11_000.0  # Tropopause altitude [m]

        # Pre-compute tropopause state for stratosphere calculations
        self.T_trop = self.T0 - self.L * self.h_tropopause
        self.P_trop = self.P0 * (self.T_trop / self.T0) ** (G0 / (self.L * R_SPECIFIC_AIR))

    def get_properties(self, altitude_m: float) -> Tuple[float, float, float, float]:
        """Compute atmospheric properties at a given geometric altitude.

        Parameters
        ----------
        altitude_m : float
            Geometric altitude above sea level [m].  Clamped to [0, 20000].

        Returns
        -------
        temperature_K : float
        pressure_Pa : float
        density_kg_m3 : float
        speed_of_sound_ms : float
        """
        h = np.clip(altitude_m, 0.0, 20_000.0)

        if h <= self.h_tropopause:
            # Troposphere — linear lapse
            T = self.T0 - self.L * h
            P = self.P0 * (T / self.T0) ** (G0 / (self.L * R_SPECIFIC_AIR))
        else:
            # Lower stratosphere — isothermal
            T = self.T_trop
            P = self.P_trop * np.exp(-G0 * (h - self.h_tropopause) / (R_SPECIFIC_AIR * self.T_trop))

        rho = P / (R_SPECIFIC_AIR * T)
        a = np.sqrt(GAMMA_AIR * R_SPECIFIC_AIR * T)

        return float(T), float(P), float(rho), float(a)


class GravityModel:
    """Latitude- and altitude-dependent gravitational acceleration.

    Uses Somigliana formula for latitude variation at sea level,
    then free-air correction for altitude:
        g(φ, h) = g_φ × (R_E / (R_E + h))²

    where g_φ (Somigliana) gives:
        g_equator = 9.7803253359 m/s²
        g_pole    = 9.8321849378 m/s²

    Parameters
    ----------
    latitude_deg : float
        Geographic latitude [°]. Default 26.0 (Jaipur, India).
    """

    def __init__(self, latitude_deg: float = 26.0, R_e: float = R_EARTH):
        self.R_e = R_e
        self.latitude_rad = np.deg2rad(latitude_deg)
        self.latitude_deg = latitude_deg

        # Somigliana formula for sea-level gravity
        sin2 = np.sin(self.latitude_rad) ** 2
        g_e = 9.7803253359  # Equatorial gravity
        k = 0.00193185265241  # Somigliana constant
        e2 = 0.00669437999014  # First eccentricity squared
        self.g0 = g_e * (1 + k * sin2) / np.sqrt(1 - e2 * sin2)

    def g(self, altitude_m: float) -> float:
        """Return gravitational acceleration at *altitude_m* [m/s²]."""
        return self.g0 * (self.R_e / (self.R_e + max(altitude_m, 0.0))) ** 2


class CoriolisModel:
    """Coriolis and centrifugal acceleration for rotating Earth frame.

    For a projectile at latitude φ with velocity v, the Coriolis
    acceleration is: a_cor = -2 (Ω × v)

    In the ENU (x=East, y=North, z=Up) frame:
        Ω = ω_e × [0, cos(φ), sin(φ)]

    Parameters
    ----------
    latitude_deg : float
        Geographic latitude [°].
    enabled : bool
        If False, returns zero acceleration (useful for toggling).
    """

    def __init__(self, latitude_deg: float = 26.0, enabled: bool = True):
        self.enabled = enabled
        self.latitude_rad = np.deg2rad(latitude_deg)

        # Earth angular velocity vector in ENU frame
        # Ω = [0, ω·cos(φ), ω·sin(φ)]
        self.omega_vec = np.array([
            0.0,
            OMEGA_EARTH * np.cos(self.latitude_rad),
            OMEGA_EARTH * np.sin(self.latitude_rad),
        ])

    def acceleration(self, velocity: np.ndarray) -> np.ndarray:
        """Compute Coriolis acceleration: a = -2(Ω × v).

        Parameters
        ----------
        velocity : ndarray (3,)
            Velocity in ENU frame [m/s].

        Returns
        -------
        a_coriolis : ndarray (3,)
            Coriolis acceleration [m/s²].
        """
        if not self.enabled:
            return np.zeros(3)

        return -2.0 * np.cross(self.omega_vec, velocity)


class WindModel:
    """Steady wind + Dryden-style turbulence gust model.

    The steady component is a constant horizontal vector specified by
    speed and meteorological direction (direction the wind blows FROM).
    Turbulence adds band-limited Gaussian noise scaled by intensity.

    Parameters
    ----------
    speed_ms : float
        Mean wind speed [m/s].
    direction_deg : float
        Direction the wind is blowing FROM [°], meteorological convention
        (0 = from North, 90 = from East).
    turbulence_intensity : float
        Fraction of mean wind speed used as gust σ  (e.g. 0.1 = 10 %).
    seed : int or None
        Random seed for reproducible gust sequences.
    """

    def __init__(
        self,
        speed_ms: float = 5.0,
        direction_deg: float = 90.0,
        turbulence_intensity: float = 0.1,
        seed: int | None = None,
    ):
        self.speed = speed_ms
        self.direction_rad = np.deg2rad(direction_deg)
        self.turb_intensity = turbulence_intensity
        self.rng = np.random.default_rng(seed)

        # Steady wind vector in sim frame (x=East, y=North, z=Up)
        # Meteorological "FROM" direction → flip 180° for velocity direction
        self.wx_steady = -self.speed * np.sin(self.direction_rad)
        self.wy_steady = -self.speed * np.cos(self.direction_rad)

    def get_wind(self, altitude_m: float, t: float) -> Tuple[float, float, float]:
        """Return wind velocity vector (wx, wy, wz) at given altitude and time.

        A simple altitude shear (log-law) scales the steady wind, and
        band-limited Gaussian gusts are added.

        Parameters
        ----------
        altitude_m : float
            Current altitude [m].
        t : float
            Current simulation time [s]  (unused for steady; seeds gusts).

        Returns
        -------
        wx, wy, wz : float
            Wind velocity components [m/s] in the simulation frame.
        """
        # Log-law wind shear (reference height 10 m)
        h_ref = 10.0
        h = max(altitude_m, 1.0)
        shear_factor = np.log(h / 0.01) / np.log(h_ref / 0.01)
        shear_factor = np.clip(shear_factor, 0.5, 2.5)

        # Turbulence gusts
        gust_sigma = self.turb_intensity * self.speed
        gust_x = self.rng.normal(0.0, max(gust_sigma, 0.01))
        gust_y = self.rng.normal(0.0, max(gust_sigma, 0.01))
        gust_z = self.rng.normal(0.0, max(gust_sigma * 0.3, 0.003))  # Vertical gusts smaller

        wx = self.wx_steady * shear_factor + gust_x
        wy = self.wy_steady * shear_factor + gust_y
        wz = gust_z  # No mean vertical wind

        return float(wx), float(wy), float(wz)
