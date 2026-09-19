"""
train.py — PPO Training Script for RL Guidance Agent
======================================================
Trains a neural network guidance policy using Proximal Policy
Optimization (PPO) from Stable Baselines3.

The agent learns to output canard pitch/yaw commands that minimize
miss distance across randomized launch conditions, wind, and
perturbations.

Usage
-----
    python3 -m rl.train --timesteps 200000 --save-path models/ppo_pgk

Architecture
------------
    Policy network: MLP [64, 64] with Tanh activation
    Value network:  MLP [64, 64] with Tanh activation
    Algorithm:      PPO (clip_range=0.2, n_epochs=10, batch_size=64)
"""

import argparse
import os
import sys
import yaml
import numpy as np
import time

# Add project root to path
sys.path.insert(0, os.path.dirname(os.path.dirname(os.path.abspath(__file__))))


def make_env(config, seed=None):
    """Create a PGK guidance environment."""
    from rl.guidance_env import PGKGuidanceEnv

    def _init():
        env = PGKGuidanceEnv(config, control_dt=0.5, randomize=True)
        if seed is not None:
            env.reset(seed=seed)
        return env
    return _init


def train(timesteps=200_000, save_path="models/ppo_pgk", config_path=None):
    """Train the PPO agent.

    Parameters
    ----------
    timesteps : int
        Total training timesteps.
    save_path : str
        Path to save the trained model.
    config_path : str
        Path to mission config YAML.
    """
    from stable_baselines3 import PPO
    from stable_baselines3.common.vec_env import SubprocVecEnv, DummyVecEnv
    from stable_baselines3.common.callbacks import (
        EvalCallback, CheckpointCallback, BaseCallback
    )

    # Load config
    if config_path is None:
        config_path = os.path.join(
            os.path.dirname(os.path.dirname(os.path.abspath(__file__))),
            "config", "mission_config.yaml"
        )
    with open(config_path) as f:
        config = yaml.safe_load(f)

    print("=" * 60)
    print("  PPO Training — PGK Guidance Agent")
    print("=" * 60)
    print(f"  Timesteps:    {timesteps:,}")
    print(f"  Save path:    {save_path}")
    print(f"  Control dt:   0.5 s")
    print(f"  Randomized:   MV, elevation, wind, Cd")
    print("=" * 60)

    # Create vectorized environments (parallel training)
    n_envs = 4
    print(f"\n  Creating {n_envs} parallel environments...")

    env = DummyVecEnv([make_env(config, seed=i * 1000) for i in range(n_envs)])

    # Evaluation environment
    eval_env = DummyVecEnv([make_env(config, seed=99999)])

    # Custom callback to log miss distances
    class MissDistanceCallback(BaseCallback):
        def __init__(self, verbose=0):
            super().__init__(verbose)
            self.episode_misses = []
            self.best_mean_miss = float("inf")

        def _on_step(self):
            # Check for episode completion
            for info in self.locals.get("infos", []):
                if "miss_distance_m" in info and info.get("TimeLimit.truncated", False) or \
                   self.locals.get("dones", [False])[0]:
                    self.episode_misses.append(info["miss_distance_m"])

            # Log every 50 episodes
            if len(self.episode_misses) >= 50:
                mean_miss = np.mean(self.episode_misses[-50:])
                min_miss = np.min(self.episode_misses[-50:])
                if mean_miss < self.best_mean_miss:
                    self.best_mean_miss = mean_miss
                print(f"  [{self.num_timesteps:>8,} steps] "
                      f"Mean miss: {mean_miss:>7.1f}m | "
                      f"Best miss: {min_miss:>6.1f}m | "
                      f"Best mean: {self.best_mean_miss:>7.1f}m")
                self.episode_misses = []

            return True

    # PPO agent with tuned hyperparameters
    model = PPO(
        "MlpPolicy",
        env,
        learning_rate=3e-4,
        n_steps=2048,           # Steps per rollout
        batch_size=64,
        n_epochs=10,
        gamma=0.99,             # Discount factor
        gae_lambda=0.95,        # GAE lambda
        clip_range=0.2,
        ent_coef=0.01,          # Entropy bonus for exploration
        vf_coef=0.5,
        max_grad_norm=0.5,
        policy_kwargs=dict(
            net_arch=dict(pi=[64, 64], vf=[64, 64]),  # 2-layer MLP
        ),
        verbose=0,
        seed=42,
    )

    print(f"\n  Model parameters: {sum(p.numel() for p in model.policy.parameters()):,}")
    print(f"  Training started...\n")

    t_start = time.time()

    # Train
    model.learn(
        total_timesteps=timesteps,
        callback=MissDistanceCallback(),
        progress_bar=True,
    )

    elapsed = time.time() - t_start
    print(f"\n  Training completed in {elapsed / 60:.1f} minutes")

    # Save model
    os.makedirs(os.path.dirname(save_path) if os.path.dirname(save_path) else ".", exist_ok=True)
    model.save(save_path)
    print(f"  Model saved to: {save_path}.zip")

    env.close()
    eval_env.close()

    return model


def evaluate(model_path="models/ppo_pgk", n_episodes=20, config_path=None):
    """Evaluate a trained model and compare with classical controller.

    Parameters
    ----------
    model_path : str
        Path to the saved PPO model.
    n_episodes : int
        Number of evaluation episodes.
    config_path : str
        Path to mission config YAML.
    """
    import copy
    from stable_baselines3 import PPO
    from rl.guidance_env import PGKGuidanceEnv
    from simulation.simulator import FlightSimulator
    from experiments.monte_carlo import MonteCarloRunner

    if config_path is None:
        config_path = os.path.join(
            os.path.dirname(os.path.dirname(os.path.abspath(__file__))),
            "config", "mission_config.yaml"
        )
    with open(config_path) as f:
        config = yaml.safe_load(f)

    model = PPO.load(model_path)

    print("=" * 60)
    print("  RL vs Classical Guidance Comparison")
    print("=" * 60)

    rl_misses = []
    classical_misses = []
    unguided_misses = []
    runner = MonteCarloRunner(config, n_runs=n_episodes)

    for i in range(n_episodes):
        seed = 42 + i * 1000
        p = runner.generate_perturbations(seed)
        cfg_copy = copy.deepcopy(config)
        cfg_copy["environment"]["wind_direction_deg"] = p["wind_direction_deg"]

        # RL agent
        env = PGKGuidanceEnv(cfg_copy, control_dt=0.5, randomize=False)
        # Manually set perturbations to match
        obs, _ = env.reset(seed=seed)
        # Re-run with same perturbations
        env2 = PGKGuidanceEnv(config, control_dt=0.5, randomize=True)
        obs, _ = env2.reset(seed=seed)
        done = False
        while not done:
            action, _ = model.predict(obs, deterministic=True)
            obs, reward, terminated, truncated, info = env2.step(action)
            done = terminated or truncated
        rl_misses.append(info["miss_distance_m"])

        # Classical controller
        sim_c = FlightSimulator(cfg_copy)
        r_c = sim_c.run_single(seed=seed, guided=True,
                               muzzle_velocity=p["muzzle_velocity"],
                               elevation_deg=p["elevation_deg"],
                               azimuth_deg=p.get("azimuth_deg", 0),
                               wind_speed=p["wind_speed"],
                               cd_scale=p["cd_scale"])
        classical_misses.append(r_c["miss_distance_m"])

        # Unguided
        sim_u = FlightSimulator(cfg_copy)
        r_u = sim_u.run_single(seed=seed, guided=False,
                               muzzle_velocity=p["muzzle_velocity"],
                               elevation_deg=p["elevation_deg"],
                               azimuth_deg=p.get("azimuth_deg", 0),
                               wind_speed=p["wind_speed"],
                               cd_scale=p["cd_scale"])
        unguided_misses.append(r_u["miss_distance_m"])

        if (i + 1) % 5 == 0:
            print(f"  Episode {i+1}/{n_episodes} | "
                  f"RL: {rl_misses[-1]:.0f}m | "
                  f"Classical: {classical_misses[-1]:.0f}m | "
                  f"Unguided: {unguided_misses[-1]:.0f}m")

    rl = np.array(rl_misses)
    cl = np.array(classical_misses)
    un = np.array(unguided_misses)

    print(f"\n{'':>20} {'RL Agent':>12} {'Classical':>12} {'Unguided':>12}")
    print(f"  {'CEP50:':>18} {np.median(rl):>10.1f}m {np.median(cl):>10.1f}m {np.median(un):>10.1f}m")
    print(f"  {'Mean miss:':>18} {np.mean(rl):>10.1f}m {np.mean(cl):>10.1f}m {np.mean(un):>10.1f}m")
    print(f"  {'Max miss:':>18} {np.max(rl):>10.1f}m {np.max(cl):>10.1f}m {np.max(un):>10.1f}m")
    print(f"  {'Min miss:':>18} {np.min(rl):>10.1f}m {np.min(cl):>10.1f}m {np.min(un):>10.1f}m")
    print()
    rl_wins = np.sum(rl < cl)
    print(f"  RL beats Classical: {rl_wins}/{n_episodes} runs")
    print(f"  RL improvement over Unguided: {np.median(un)/max(np.median(rl),0.1):.1f}x")


if __name__ == "__main__":
    parser = argparse.ArgumentParser(description="Train PPO guidance agent")
    parser.add_argument("--timesteps", type=int, default=200_000,
                        help="Total training timesteps")
    parser.add_argument("--save-path", type=str, default="models/ppo_pgk",
                        help="Model save path")
    parser.add_argument("--config", type=str, default=None,
                        help="Path to mission config YAML")
    parser.add_argument("--evaluate", type=str, default=None,
                        help="Evaluate a trained model instead of training")
    parser.add_argument("--eval-episodes", type=int, default=20,
                        help="Number of evaluation episodes")

    args = parser.parse_args()

    if args.evaluate:
        evaluate(args.evaluate, args.eval_episodes, args.config)
    else:
        train(args.timesteps, args.save_path, args.config)
