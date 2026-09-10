"""
monte_carlo.py — Batch Monte Carlo Simulation Runner
======================================================
Executes parametrically perturbed flight simulations to statistically
validate guidance accuracy (CEP) under realistic operational uncertainties.
"""

import numpy as np
import copy
import time
from typing import Dict, List, Optional, Tuple

from simulation.simulator import FlightSimulator, load_config


class MonteCarloRunner:
    """Batch runner for Monte Carlo trajectory simulations.

    Generates stochastic perturbations of launch conditions, aerodynamic
    parameters, wind, and sensor biases to produce statistically
    meaningful dispersion data.

    Parameters
    ----------
    base_config : dict
        Nominal mission configuration.
    n_runs : int
        Number of Monte Carlo trials per batch.
    """

    def __init__(self, base_config: dict, n_runs: int = 100):
        self.base_config = base_config
        self.n_runs = n_runs
        mc_cfg = base_config.get("monte_carlo", {})
        self.mv_sigma = mc_cfg.get("muzzle_velocity_sigma_ms", 10.0)
        self.el_sigma_mil = mc_cfg.get("elevation_sigma_mil", 2.0)
        self.az_sigma_mil = mc_cfg.get("azimuth_sigma_mil", 2.0)
        self.wind_sigma = mc_cfg.get("wind_speed_sigma_ms", 4.0)
        self.drag_var_pct = mc_cfg.get("drag_variation_pct", 5.0)

    def generate_perturbations(self, seed: int) -> Dict:
        """Generate a single set of randomized perturbations.

        Parameters
        ----------
        seed : int
            Random seed for this trial.

        Returns
        -------
        perturbations : dict
            Dictionary of perturbed parameters.
        """
        rng = np.random.default_rng(seed)

        # 1 mil ≈ 0.05625°
        mil_to_deg = 0.05625

        base_mv = self.base_config["shell"]["muzzle_velocity_ms"]
        base_el = self.base_config["shell"]["launch_elevation_deg"]
        base_az = self.base_config["shell"]["launch_azimuth_deg"]
        base_wind = self.base_config["environment"]["wind_speed_ms"]

        return {
            "muzzle_velocity": base_mv + rng.normal(0, self.mv_sigma),
            "elevation_deg": base_el + rng.normal(0, self.el_sigma_mil * mil_to_deg),
            "azimuth_deg": base_az + rng.normal(0, self.az_sigma_mil * mil_to_deg),
            "wind_speed": max(0.0, base_wind + rng.normal(0, self.wind_sigma)),
            "wind_direction_deg": rng.uniform(0, 360),
            "cd_scale": 1.0 + rng.normal(0, self.drag_var_pct / 100.0),
            "seed": seed,
        }

    def run_batch(
        self,
        guided: bool = True,
        progress_callback=None,
    ) -> List[Dict]:
        """Execute a batch of Monte Carlo simulations.

        Parameters
        ----------
        guided : bool
            Whether canard guidance is active.
        progress_callback : callable or None
            Optional callback(run_index, n_runs, result) for progress.

        Returns
        -------
        results : list of dict
            Each dict contains impact_point, miss_distance, and metadata.
        """
        simulator = FlightSimulator(self.base_config)
        results = []
        base_seed = self.base_config["simulation"].get("seed", 42)

        t_start = time.time()

        for i in range(self.n_runs):
            run_seed = base_seed + i * 1000
            perturbation = self.generate_perturbations(run_seed)

            # Apply wind direction perturbation
            config_copy = copy.deepcopy(self.base_config)
            config_copy["environment"]["wind_direction_deg"] = perturbation["wind_direction_deg"]
            sim = FlightSimulator(config_copy)

            try:
                result = sim.run_single(
                    seed=run_seed,
                    guided=guided,
                    muzzle_velocity=perturbation["muzzle_velocity"],
                    elevation_deg=perturbation["elevation_deg"],
                    azimuth_deg=perturbation["azimuth_deg"],
                    wind_speed=perturbation["wind_speed"],
                    cd_scale=perturbation["cd_scale"],
                )

                run_data = {
                    "run_index": i,
                    "impact_point": result["impact_point"][:2],  # [x, y]
                    "miss_distance_m": result["miss_distance_m"],
                    "flight_time_s": result["flight_time_s"],
                    "max_altitude_m": result["max_altitude_m"],
                    "perturbation": perturbation,
                    "guided": guided,
                }
                results.append(run_data)

            except Exception as e:
                print(f"  [WARNING] Run {i} failed: {e}")
                continue

            # Progress reporting
            if (i + 1) % 10 == 0 or i == 0:
                elapsed = time.time() - t_start
                rate = (i + 1) / elapsed if elapsed > 0 else 0
                eta = (self.n_runs - i - 1) / rate if rate > 0 else 0
                mode = "GUIDED" if guided else "UNGUIDED"
                print(
                    f"  [{mode}] Run {i+1}/{self.n_runs} | "
                    f"Miss: {run_data['miss_distance_m']:.1f}m | "
                    f"Rate: {rate:.1f} runs/s | ETA: {eta:.0f}s"
                )

            if progress_callback:
                progress_callback(i, self.n_runs, run_data)

        elapsed = time.time() - t_start
        print(f"  Batch complete: {len(results)}/{self.n_runs} runs in {elapsed:.1f}s")
        return results

    def run_comparison(self, progress_callback=None) -> Dict:
        """Run both guided and unguided batches for CEP comparison.

        Returns
        -------
        comparison : dict
            Keys: 'guided', 'unguided', each containing list of run results.
        """
        print("=" * 60)
        print("MONTE CARLO CEP COMPARISON")
        print(f"  Runs per batch: {self.n_runs}")
        print("=" * 60)

        print("\n── Unguided Batch ─────────────────────────────────")
        unguided = self.run_batch(guided=False, progress_callback=progress_callback)

        print("\n── Guided Batch ───────────────────────────────────")
        guided = self.run_batch(guided=True, progress_callback=progress_callback)

        return {
            "guided": guided,
            "unguided": unguided,
        }


def quick_monte_carlo(
    config_path: str = "config/mission_config.yaml",
    n_runs: int = 50,
) -> Dict:
    """Convenience function to run a quick Monte Carlo comparison.

    Returns dict with 'guided' and 'unguided' result lists.
    """
    config = load_config(config_path)
    runner = MonteCarloRunner(config, n_runs=n_runs)
    return runner.run_comparison()


if __name__ == "__main__":
    results = quick_monte_carlo(n_runs=50)
    print(f"\nGuided runs: {len(results['guided'])}")
    print(f"Unguided runs: {len(results['unguided'])}")
