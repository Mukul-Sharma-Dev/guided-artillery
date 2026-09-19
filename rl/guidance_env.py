"""
guidance_env.py — Gymnasium Environment for RL-Based PGK Guidance
==================================================================
Wraps the FlightSimulator as a reinforcement learning environment.
The RL agent controls canard deflections during the descent phase
to minimize miss distance at impact.

Observation Space (8-dim, normalized):
  [0] range_error / 5000       (target_x - pos_x, normalized)
  [1] cross_error / 2000       (target_y - pos_y, normalized)
  [2] altitude / 10000         (current altitude, normalized)
  [3] vx / 800                 (downrange velocity, normalized)
  [4] vy / 100                 (crossrange velocity, normalized)
  [5] vz / 500                 (vertical velocity, normalized)
  [6] t_go_est / 60            (estimated time-to-go, normalized)
  [7] q_bar / 50000            (dynamic pressure, normalized)

Action Space (2-dim, continuous [-1, 1]):
  [0] pitch_cmd               mapped to [-15°, +15°]
  [1] yaw_cmd                 mapped to [-15°, +15°] (sign-corrected)

Reward:
  Per-step:  shaped reward based on reducing predicted miss
  Terminal:  large bonus/penalty based on final miss distance
"""

import numpy as np
import copy
import gymnasium as gym
from gymnasium import spaces


class PGKGuidanceEnv(gym.Env):
    """Gymnasium environment for training RL guidance agent.

    The agent takes control after apogee (descent phase) and outputs
    canard pitch/yaw commands every `control_dt` seconds. The simulation
    runs at the internal dt (0.01s) between control decisions.

    Parameters
    ----------
    base_config : dict
        Mission configuration dictionary.
    control_dt : float
        Time between RL agent decisions [s]. Default 0.5s.
    randomize : bool
        If True, randomize MV, elevation, wind per episode.
    """

    metadata = {"render_modes": []}

    def __init__(self, base_config: dict, control_dt: float = 0.5,
                 randomize: bool = True):
        super().__init__()

        self.base_config = base_config
        self.control_dt = control_dt
        self.randomize = randomize
        self.max_deflection = base_config["canard"]["max_deflection_deg"]

        # Import here to avoid circular imports
        from simulation.simulator import FlightSimulator
        from simulation.environment import ISAAtmosphere
        self._FlightSimulator = FlightSimulator
        self._ISAAtmosphere = ISAAtmosphere

        # Target
        self.target = np.array([
            base_config["target"]["x_m"],
            base_config["target"]["y_m"],
            base_config["target"]["z_m"],
        ])

        # Observation: 8-dimensional normalized state
        self.observation_space = spaces.Box(
            low=-5.0, high=5.0, shape=(8,), dtype=np.float32
        )

        # Action: [pitch_cmd, yaw_cmd] in [-1, 1]
        self.action_space = spaces.Box(
            low=-1.0, high=1.0, shape=(2,), dtype=np.float32
        )

        # Internal state
        self.sim = None
        self.state = None
        self.t = 0.0
        self.dt = base_config["simulation"]["dt_s"]
        self.t_max = base_config["simulation"]["t_max_s"]
        self.episode_seed = 42
        self._step_count = 0
        self._max_steps = 500  # Safety limit
        self._past_apogee = False
        self._prev_miss_estimate = None

    def _get_perturbations(self, rng):
        """Generate random perturbations for this episode."""
        mc = self.base_config.get("monte_carlo", {})
        mv_sigma = mc.get("muzzle_velocity_sigma_ms", 5.0)
        el_sigma = mc.get("elevation_sigma_mil", 1.5) * 0.05625
        wind_sigma = mc.get("wind_speed_sigma_ms", 3.0)
        cd_sigma = mc.get("drag_variation_pct", 3.0) / 100.0

        return {
            "muzzle_velocity": 820.0 + rng.normal(0, mv_sigma),
            "elevation_deg": 51.0 + rng.normal(0, el_sigma),
            "wind_speed": max(0, 5.0 + rng.normal(0, wind_sigma)),
            "wind_direction_deg": rng.uniform(0, 360),
            "cd_scale": 1.0 + rng.normal(0, cd_sigma),
        }

    def _build_observation(self):
        """Build normalized observation vector from current state."""
        pos = self.state[:3]
        vel = self.state[3:6]

        range_error = self.target[0] - pos[0]
        cross_error = self.target[1] - pos[1]
        alt = pos[2]

        # Estimate time-to-go from altitude and vertical velocity
        if vel[2] < -1.0 and alt > 0:
            t_go_est = -alt / vel[2]  # Simple linear estimate
        else:
            t_go_est = 60.0  # Default

        # Dynamic pressure approximation
        rho = 1.225 * np.exp(-alt / 8500.0) if alt > 0 else 1.225
        v_mag = np.linalg.norm(vel)
        q_bar = 0.5 * rho * v_mag**2

        obs = np.array([
            range_error / 5000.0,
            cross_error / 2000.0,
            alt / 10000.0,
            vel[0] / 800.0,
            vel[1] / 100.0,
            vel[2] / 500.0,
            t_go_est / 60.0,
            q_bar / 50000.0,
        ], dtype=np.float32)

        return np.clip(obs, -5.0, 5.0)

    def _estimate_miss(self):
        """Quick miss distance estimate from current state."""
        pos = self.state[:3]
        return np.sqrt((self.target[0] - pos[0])**2 +
                       (self.target[1] - pos[1])**2)

    def reset(self, seed=None, options=None):
        """Reset environment for a new episode."""
        super().reset(seed=seed)

        if seed is not None:
            self.episode_seed = seed
        else:
            self.episode_seed = np.random.randint(0, 1_000_000)

        rng = np.random.default_rng(self.episode_seed)

        # Build simulator with perturbations
        config = copy.deepcopy(self.base_config)

        if self.randomize:
            perturb = self._get_perturbations(rng)
            config["environment"]["wind_direction_deg"] = perturb["wind_direction_deg"]
        else:
            perturb = {
                "muzzle_velocity": 820.0,
                "elevation_deg": 51.0,
                "wind_speed": 5.0,
                "wind_direction_deg": 90.0,
                "cd_scale": 1.0,
            }

        self.sim = self._FlightSimulator(config)
        self.sim._build_subsystems(self.episode_seed, guided=True)

        # Override parameters
        v0 = perturb["muzzle_velocity"]
        el = np.deg2rad(perturb["elevation_deg"])
        az = 0.0

        if perturb["wind_speed"] != 5.0:
            self.sim.wind_model.speed = perturb["wind_speed"]
            self.sim.wind_model.wx_steady = -perturb["wind_speed"] * np.sin(self.sim.wind_model.direction_rad)
            self.sim.wind_model.wy_steady = -perturb["wind_speed"] * np.cos(self.sim.wind_model.direction_rad)

        if perturb["cd_scale"] != 1.0:
            self.sim.dynamics._cd_pts = self.sim.dynamics._cd_pts * perturb["cd_scale"]

        # Initial state
        z0 = self.sim.launch_elevation_asl
        self.state = np.array([
            0.0, 0.0, z0,
            v0 * np.cos(el), 0.0, v0 * np.sin(el),
        ])

        self.t = 0.0
        self._step_count = 0
        self._past_apogee = False
        self._prev_miss_estimate = None

        # Run ballistic phase (ascent) — no agent control needed
        self._run_to_apogee()

        obs = self._build_observation()
        return obs, {}

    def _run_to_apogee(self):
        """Advance simulation to apogee (no canard control)."""
        from fuze.fuze_simulator import FuzeState
        while self.t < self.t_max:
            wind = np.array(self.sim.wind_model.get_wind(self.state[2], self.t))

            # RK4 step (no canard)
            dt = self.dt
            k1 = self.sim.dynamics.derivatives(self.t, self.state, 0, 0, wind)
            k2 = self.sim.dynamics.derivatives(self.t + dt/2, self.state + dt/2 * k1, 0, 0, wind)
            k3 = self.sim.dynamics.derivatives(self.t + dt/2, self.state + dt/2 * k2, 0, 0, wind)
            k4 = self.sim.dynamics.derivatives(self.t + dt, self.state + dt * k3, 0, 0, wind)
            self.state += (dt / 6.0) * (k1 + 2*k2 + 2*k3 + k4)
            self.t += dt

            # Update fuze
            accel_g = np.linalg.norm(k1[3:6]) / 9.80665
            downrange = np.sqrt(self.state[0]**2 + self.state[1]**2)
            self.sim.fuze.update(self.t, accel_g, self.state[2],
                                 self.state[2], self.state[5], downrange)

            # Check apogee (vz changes from + to -)
            if self.state[5] <= 0 and self.t > 1.0:
                self._past_apogee = True
                # Add 0.5s stabilization delay
                for _ in range(int(0.5 / self.dt)):
                    wind = np.array(self.sim.wind_model.get_wind(self.state[2], self.t))
                    k1 = self.sim.dynamics.derivatives(self.t, self.state, 0, 0, wind)
                    k2 = self.sim.dynamics.derivatives(self.t + dt/2, self.state + dt/2 * k1, 0, 0, wind)
                    k3 = self.sim.dynamics.derivatives(self.t + dt/2, self.state + dt/2 * k2, 0, 0, wind)
                    k4 = self.sim.dynamics.derivatives(self.t + dt, self.state + dt * k3, 0, 0, wind)
                    self.state += (dt / 6.0) * (k1 + 2*k2 + 2*k3 + k4)
                    self.t += dt
                break

        self._prev_miss_estimate = self._estimate_miss()

    def step(self, action):
        """Execute one control step (advance sim by control_dt seconds).

        Parameters
        ----------
        action : ndarray (2,)
            [pitch_cmd, yaw_cmd] in [-1, 1]

        Returns
        -------
        obs, reward, terminated, truncated, info
        """
        # Map actions to degrees
        pitch_cmd_deg = float(action[0]) * self.max_deflection
        # Negate yaw (sign correction for canard force direction)
        yaw_cmd_deg = -float(action[1]) * self.max_deflection

        pitch_rad = np.deg2rad(pitch_cmd_deg)
        yaw_rad = np.deg2rad(yaw_cmd_deg)

        # Advance simulation for control_dt seconds
        n_steps = int(self.control_dt / self.dt)
        dt = self.dt
        terminated = False
        target_elev = self.sim.target_elevation_asl

        for _ in range(n_steps):
            wind = np.array(self.sim.wind_model.get_wind(self.state[2], self.t))

            # Apply actuator dynamics
            actual_p, actual_y = self.sim.actuator.command(
                pitch_cmd_deg, yaw_cmd_deg, dt
            )
            actual_p_rad = np.deg2rad(actual_p)
            actual_y_rad = np.deg2rad(actual_y)

            # RK4 step
            k1 = self.sim.dynamics.derivatives(self.t, self.state, actual_p_rad, actual_y_rad, wind)
            k2 = self.sim.dynamics.derivatives(self.t + dt/2, self.state + dt/2 * k1, actual_p_rad, actual_y_rad, wind)
            k3 = self.sim.dynamics.derivatives(self.t + dt/2, self.state + dt/2 * k2, actual_p_rad, actual_y_rad, wind)
            k4 = self.sim.dynamics.derivatives(self.t + dt, self.state + dt * k3, actual_p_rad, actual_y_rad, wind)
            self.state += (dt / 6.0) * (k1 + 2*k2 + 2*k3 + k4)
            self.t += dt

            # Ground hit check
            if self.state[2] < target_elev and self.t > 1.0:
                self.state[2] = target_elev
                terminated = True
                break

        self._step_count += 1

        # Compute reward
        current_miss = self._estimate_miss()
        reward = 0.0

        if terminated:
            # Terminal reward — big bonus/penalty based on miss distance
            miss = np.sqrt((self.target[0] - self.state[0])**2 +
                           (self.target[1] - self.state[1])**2)
            if miss < 10:
                reward = 100.0       # Excellent hit
            elif miss < 30:
                reward = 50.0        # Good hit (under CEP target)
            elif miss < 100:
                reward = 20.0 - miss * 0.1
            elif miss < 500:
                reward = -miss * 0.01
            else:
                reward = -5.0 - miss * 0.002
        else:
            # Shaping: reward for reducing estimated miss
            if self._prev_miss_estimate is not None:
                improvement = self._prev_miss_estimate - current_miss
                reward = improvement * 0.01  # Small per-step shaping

            # Small penalty for large deflections (encourage efficiency)
            defl_penalty = 0.001 * (abs(action[0]) + abs(action[1]))
            reward -= defl_penalty

        self._prev_miss_estimate = current_miss

        # Truncate if too many steps
        truncated = (self._step_count >= self._max_steps) or (self.t >= self.t_max)

        obs = self._build_observation()
        info = {
            "miss_distance_m": current_miss,
            "time_s": self.t,
            "altitude_m": self.state[2],
        }

        return obs, reward, terminated, truncated, info
