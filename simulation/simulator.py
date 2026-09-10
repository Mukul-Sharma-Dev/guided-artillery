"""
simulator.py — Main Flight Simulation Loop
============================================
Integrates all subsystems (dynamics, sensors, EKF, guidance, fuze)
into a complete closed-loop flight simulation with RK4 integration.
"""

import numpy as np
import yaml
from typing import Dict, Optional
from pathlib import Path

from .environment import ISAAtmosphere, GravityModel, CoriolisModel, WindModel
from .dynamics import ProjectileDynamics
from .sensors import IMUSensor, GPSSensor, BaroSensor
from .actuator_model import CanardActuator

from estimation.state_estimator import ExtendedKalmanFilter
from control.controller import CanardController, FlightPhase
from fuze.fuze_simulator import ElectronicFuze, FuzeState, FuzeMode


def load_config(config_path: str = "config/mission_config.yaml") -> dict:
    """Load mission configuration from YAML file."""
    with open(config_path, "r") as f:
        return yaml.safe_load(f)


class FlightSimulator:
    """End-to-end closed-loop flight simulator for the 155 mm PGK.

    Orchestrates:
      1. RK4 numerical integration of projectile dynamics
      2. Sensor measurement generation (IMU, GPS, Baro)
      3. EKF state estimation
      4. Guidance & canard control (if guided)
      5. Fuze state machine
      6. Comprehensive data logging

    Parameters
    ----------
    config : dict
        Full mission configuration dictionary.
    """

    def __init__(self, config: dict):
        self.config = config
        self.dt = config["simulation"]["dt_s"]
        self.t_max = config["simulation"]["t_max_s"]

    def _build_subsystems(self, seed: int, guided: bool):
        """Instantiate all subsystem objects with given seed."""
        cfg = self.config
        shell = cfg["shell"]
        canard_cfg = cfg["canard"]
        sensor_cfg = cfg["sensors"]
        fuze_cfg = cfg["fuze"]
        env_cfg = cfg["environment"]
        guidance_cfg = cfg.get("guidance", {})

        # Environment
        latitude = env_cfg.get("latitude_deg", 26.0)
        self.atmosphere = ISAAtmosphere(
            T0=env_cfg.get("sea_level_temp_K", 288.15),
            P0=env_cfg.get("sea_level_pressure_Pa", 101325.0),
            delta_T=env_cfg.get("temperature_deviation_K", 0.0),
        )
        self.gravity = GravityModel(latitude_deg=latitude)
        self.coriolis = CoriolisModel(
            latitude_deg=latitude,
            enabled=env_cfg.get("coriolis_enabled", True),
        )
        self.wind_model = WindModel(
            speed_ms=env_cfg.get("wind_speed_ms", 5.0),
            direction_deg=env_cfg.get("wind_direction_deg", 90.0),
            turbulence_intensity=env_cfg.get("turbulence_intensity", 0.1),
            seed=seed + 100,
        )

        # Launch and target elevation
        self.launch_elevation_asl = env_cfg.get("launch_elevation_asl_m", 0.0)
        self.target_elevation_asl = cfg["target"].get("elevation_asl_m",
                                                       self.launch_elevation_asl)

        # Dynamics (with Coriolis)
        self.dynamics = ProjectileDynamics(
            mass_kg=shell["mass_kg"],
            ref_area_m2=shell["reference_area_m2"],
            cd_table=shell["cd_table"],
            canard_area_m2=canard_cfg.get("canard_area_m2", 0.004),
            cl_delta=canard_cfg.get("cl_delta", 3.5),
            atmosphere=self.atmosphere,
            gravity=self.gravity,
            coriolis=self.coriolis,
        )

        # Sensors
        self.imu = IMUSensor(
            accel_noise_sigma=sensor_cfg["imu"]["accel_noise_sigma_ms2"],
            gyro_noise_sigma=sensor_cfg["imu"]["gyro_noise_sigma_rads"],
            accel_bias=sensor_cfg["imu"]["accel_bias_ms2"],
            gyro_bias=sensor_cfg["imu"]["gyro_bias_rads"],
            saturation_g=sensor_cfg["imu"]["saturation_g"],
            seed=seed + 200,
        )
        self.gps = GPSSensor(
            pos_noise_sigma=sensor_cfg["gps"]["position_noise_sigma_m"],
            vel_noise_sigma=sensor_cfg["gps"]["velocity_noise_sigma_ms"],
            update_rate_hz=sensor_cfg["gps"]["update_rate_hz"],
            dropout_prob=sensor_cfg["gps"]["dropout_probability"],
            seed=seed + 300,
        )
        self.baro = BaroSensor(
            alt_noise_sigma=sensor_cfg["baro"]["altitude_noise_sigma_m"],
            pressure_bias_pa=sensor_cfg["baro"]["pressure_bias_pa"],
            seed=seed + 400,
        )

        # Actuator
        self.actuator = CanardActuator(
            max_deflection_deg=canard_cfg["max_deflection_deg"],
            slew_rate_deg_s=canard_cfg["slew_rate_deg_s"],
            time_constant_s=canard_cfg.get("lag_time_constant_s", 0.02),
        )

        # EKF
        self.ekf = ExtendedKalmanFilter(
            process_noise_accel=10.0,
            gps_pos_noise=sensor_cfg["gps"]["position_noise_sigma_m"],
            gps_vel_noise=sensor_cfg["gps"]["velocity_noise_sigma_ms"],
            baro_noise=sensor_cfg["baro"]["altitude_noise_sigma_m"],
        )

        # Target
        target = np.array([
            cfg["target"]["x_m"],
            cfg["target"]["y_m"],
            cfg["target"]["z_m"],
        ])

        # Controller
        self.controller = CanardController(target=target, config=cfg) if guided else None

        # Fuze
        self.fuze = ElectronicFuze(fuze_cfg)

        self.guided = guided
        self.target = target

    def _rk4_step(self, t, state, canard_p_rad, canard_y_rad, wind):
        """Standard 4th-order Runge-Kutta integration step."""
        dt = self.dt

        k1 = self.dynamics.derivatives(t, state, canard_p_rad, canard_y_rad, wind)
        k2 = self.dynamics.derivatives(t + dt / 2, state + dt / 2 * k1, canard_p_rad, canard_y_rad, wind)
        k3 = self.dynamics.derivatives(t + dt / 2, state + dt / 2 * k2, canard_p_rad, canard_y_rad, wind)
        k4 = self.dynamics.derivatives(t + dt, state + dt * k3, canard_p_rad, canard_y_rad, wind)

        return state + (dt / 6.0) * (k1 + 2 * k2 + 2 * k3 + k4)

    def run_single(
        self,
        seed: int = 42,
        guided: bool = True,
        muzzle_velocity: Optional[float] = None,
        elevation_deg: Optional[float] = None,
        azimuth_deg: Optional[float] = None,
        fuze_mode: Optional[str] = None,
        wind_speed: Optional[float] = None,
        cd_scale: float = 1.0,
    ) -> Dict:
        """Execute a single flight simulation.

        Parameters
        ----------
        seed : int
            Random seed for this run.
        guided : bool
            If True, canard guidance is active during descent.
        muzzle_velocity : float or None
            Override muzzle velocity [m/s].
        elevation_deg : float or None
            Override launch elevation [°].
        azimuth_deg : float or None
            Override launch azimuth [°].
        fuze_mode : str or None
            Override fuze mode ("PROXIMITY", "TIME", "IMPACT").
        wind_speed : float or None
            Override wind speed [m/s].
        cd_scale : float
            Multiplicative factor on drag coefficient.

        Returns
        -------
        results : dict
            Comprehensive simulation results with time histories.
        """
        self._build_subsystems(seed, guided)

        # Override parameters if provided
        cfg = self.config
        v0 = muzzle_velocity or cfg["shell"]["muzzle_velocity_ms"]
        el = np.deg2rad(elevation_deg or cfg["shell"]["launch_elevation_deg"])
        az = np.deg2rad(azimuth_deg or cfg["shell"]["launch_azimuth_deg"])

        if wind_speed is not None:
            self.wind_model.speed = wind_speed
            self.wind_model.wx_steady = -wind_speed * np.sin(self.wind_model.direction_rad)
            self.wind_model.wy_steady = -wind_speed * np.cos(self.wind_model.direction_rad)

        if cd_scale != 1.0:
            self.dynamics._cd_pts = self.dynamics._cd_pts * cd_scale

        if fuze_mode:
            try:
                self.fuze.set_mode(FuzeMode[fuze_mode.upper()])
            except KeyError:
                pass

        # Initial conditions (z starts at launch elevation ASL)
        z0 = self.launch_elevation_asl
        state = np.array([
            0.0, 0.0, z0,                                # Position at launch point
            v0 * np.cos(el) * np.cos(az),                # vx
            v0 * np.cos(el) * np.sin(az),                # vy
            v0 * np.sin(el),                             # vz
        ])

        # Initialize EKF with launch state
        self.ekf.initialize(state[:3], state[3:6])

        # Pre-simulate launch shock for fuze arming (single-step)
        self.fuze.update(
            t=0.0,
            acceleration_g=15_000,  # Simulated setback
            altitude=0.0,
            distance_to_ground=0.0,
            velocity_z=state[5],
            downrange_distance=0.0,
        )

        # ── Data logging arrays ──────────────────────────────────────
        max_steps = int(self.t_max / self.dt) + 10
        log_interval = max(1, int(0.01 / self.dt))  # Log every ~10ms

        times = []
        true_positions = []
        true_velocities = []
        ekf_positions = []
        ekf_velocities = []
        ekf_sigmas = []
        gps_positions = []
        canard_pitches = []
        canard_yaws = []
        fuze_states = []
        flight_phases = []
        mach_numbers = []
        accelerations_g = []

        t = 0.0
        step = 0
        launched = True
        apogee_alt = 0.0

        # ── Main simulation loop ─────────────────────────────────────
        while t < self.t_max and step < max_steps:
            pos = state[:3]
            vel = state[3:6]
            alt = pos[2]

            # Terminate if at/below target elevation (after gaining altitude)
            if alt < self.target_elevation_asl and t > 1.0:
                state[2] = self.target_elevation_asl
                # Simulate ground impact deceleration spike for fuze
                downrange = np.sqrt(state[0] ** 2 + state[1] ** 2)
                self.fuze.update(
                    t=t, acceleration_g=2000,  # Impact deceleration spike
                    altitude=0.0, distance_to_ground=0.0,
                    velocity_z=vel[2], downrange_distance=downrange,
                )
                break

            # Track apogee
            if alt > apogee_alt:
                apogee_alt = alt

            # Wind
            wind = np.array(self.wind_model.get_wind(alt, t))

            # ── Sensors ──────────────────────────────────────────────
            # True acceleration (for IMU)
            g = self.gravity.g(alt)
            true_accel = (self.dynamics.compute_drag(vel, alt, wind) +
                          np.array([0.0, 0.0, -g * self.dynamics.mass])) / self.dynamics.mass
            # Add gravity back for specific force (what IMU measures)
            specific_force = true_accel + np.array([0.0, 0.0, g])

            imu_accel = self.imu.measure_accel(specific_force)
            gps_pos, gps_vel, gps_valid = self.gps.measure(pos, vel, t)
            baro_alt = self.baro.measure(alt)

            # ── EKF ─────────────────────────────────────────────────
            # Feed IMU as acceleration (subtract gravity estimate)
            ekf_accel = imu_accel - np.array([0.0, 0.0, g])
            self.ekf.predict(ekf_accel, self.dt)

            if gps_valid and gps_pos is not None:
                self.ekf.update_gps(gps_pos, gps_vel)

            # Baro update at its rate (~20 Hz → every 50ms)
            baro_interval = max(1, int(1.0 / (cfg["sensors"]["baro"]["update_rate_hz"] * self.dt)))
            if step % baro_interval == 0:
                self.ekf.update_baro(baro_alt)

            ekf_state, ekf_cov_diag = self.ekf.get_state()

            # ── Guidance & Control ───────────────────────────────────
            pitch_cmd, yaw_cmd = 0.0, 0.0
            if self.guided and self.controller is not None:
                pitch_cmd, yaw_cmd = self.controller.compute_commands(ekf_state, t)
                actual_p, actual_y = self.actuator.command(pitch_cmd, yaw_cmd, self.dt)
            else:
                actual_p, actual_y = 0.0, 0.0

            # ── Fuze ─────────────────────────────────────────────────
            accel_mag_g = np.linalg.norm(true_accel) / 9.80665
            downrange = np.sqrt(pos[0] ** 2 + pos[1] ** 2)
            fuze_state, fuze_event = self.fuze.update(
                t=t,
                acceleration_g=accel_mag_g,
                altitude=alt,
                distance_to_ground=max(alt, 0.0),
                velocity_z=vel[2],
                downrange_distance=downrange,
            )

            # ── Logging ──────────────────────────────────────────────
            if step % log_interval == 0:
                v_mag = np.linalg.norm(vel)
                _, _, _, a_sound = self.atmosphere.get_properties(alt)
                mach = v_mag / a_sound if a_sound > 0 else 0.0

                times.append(t)
                true_positions.append(pos.copy())
                true_velocities.append(vel.copy())
                ekf_positions.append(ekf_state[:3].copy())
                ekf_velocities.append(ekf_state[3:6].copy())
                ekf_sigmas.append(np.sqrt(ekf_cov_diag[:3]))
                gps_positions.append(gps_pos.copy() if gps_valid and gps_pos is not None else np.full(3, np.nan))
                canard_pitches.append(actual_p)
                canard_yaws.append(actual_y)
                fuze_states.append(fuze_state.name)
                flight_phases.append(
                    self.controller.get_phase().name if self.controller else "UNGUIDED"
                )
                mach_numbers.append(mach)
                accelerations_g.append(accel_mag_g)

            # ── RK4 Integration ──────────────────────────────────────
            canard_p_rad = np.deg2rad(actual_p)
            canard_y_rad = np.deg2rad(actual_y)
            state = self._rk4_step(t, state, canard_p_rad, canard_y_rad, wind)

            t += self.dt
            step += 1

            # Check fuze detonation
            if fuze_state == FuzeState.DETONATED:
                break

        # ── Build results dictionary ─────────────────────────────────
        true_pos_arr = np.array(true_positions)
        ekf_pos_arr = np.array(ekf_positions)

        impact_point = state[:3].copy()
        impact_point[2] = max(impact_point[2], 0.0)

        miss_distance = np.sqrt(
            (impact_point[0] - self.target[0]) ** 2 +
            (impact_point[1] - self.target[1]) ** 2
        )

        results = {
            "time": np.array(times),
            "true_position": true_pos_arr,
            "true_velocity": np.array(true_velocities),
            "ekf_position": ekf_pos_arr,
            "ekf_velocity": np.array(ekf_velocities),
            "ekf_sigma": np.array(ekf_sigmas),
            "gps_position": np.array(gps_positions),
            "canard_pitch": np.array(canard_pitches),
            "canard_yaw": np.array(canard_yaws),
            "fuze_state": fuze_states,
            "flight_phase": flight_phases,
            "mach_number": np.array(mach_numbers),
            "acceleration_g": np.array(accelerations_g),
            "impact_point": impact_point,
            "target": self.target,
            "miss_distance_m": miss_distance,
            "max_altitude_m": apogee_alt,
            "flight_time_s": t,
            "guided": guided,
            "fuze_telemetry": self.fuze.get_telemetry(),
        }

        return results
