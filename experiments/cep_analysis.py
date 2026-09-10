"""
cep_analysis.py — Circular Error Probable (CEP) Statistical Analysis
=====================================================================
Computes standard artillery accuracy metrics from Monte Carlo impact
dispersion data:  CEP50, CEP90, CEP95, Probable Errors, Mean Point
of Impact, and bias.

References
----------
* NATO STANAG 4119 — Adoption of a Standard Cannon Artillery Firing Table Format
* Grubbs, F. E. (1964). "Statistical Measures of Accuracy for Riflemen
  and Missile Engineers", BRL Report.
"""

import numpy as np
import matplotlib
matplotlib.use("Agg")  # Non-interactive backend
import matplotlib.pyplot as plt
from typing import Dict, List, Optional, Tuple


class CEPAnalyzer:
    """Computes artillery accuracy metrics from impact-point dispersion data.

    Parameters
    ----------
    impact_points : list of ndarray(2,) or list of [x, y]
        Impact point coordinates for each trial.
    target : ndarray(2,) or [x, y]
        Target coordinates.
    """

    def __init__(
        self,
        impact_points: List,
        target: List | np.ndarray,
    ):
        self.impacts = np.array(impact_points, dtype=float)
        self.target = np.array(target[:2], dtype=float)
        self._miss_distances: Optional[np.ndarray] = None

    @property
    def n_samples(self) -> int:
        return len(self.impacts)

    def compute_miss_distances(self) -> np.ndarray:
        """Compute radial miss distance for each impact from the target.

        Returns
        -------
        miss : ndarray (n,)
            Euclidean miss distances [m].
        """
        diffs = self.impacts - self.target
        self._miss_distances = np.sqrt(diffs[:, 0] ** 2 + diffs[:, 1] ** 2)
        return self._miss_distances

    def _get_miss(self) -> np.ndarray:
        if self._miss_distances is None:
            self.compute_miss_distances()
        return self._miss_distances

    def compute_cep50(self) -> float:
        """CEP50: median miss distance (50th percentile)."""
        return float(np.percentile(self._get_miss(), 50))

    def compute_cep90(self) -> float:
        """CEP90: 90th percentile of miss distances."""
        return float(np.percentile(self._get_miss(), 90))

    def compute_cep95(self) -> float:
        """CEP95: 95th percentile of miss distances."""
        return float(np.percentile(self._get_miss(), 95))

    def compute_pe_range(self) -> float:
        """Probable Error in Range (PE_R).

        PE_R = 0.6745 × σ_range   (50% of rounds within ±PE_R in range)
        """
        range_errors = self.impacts[:, 0] - self.target[0]
        return float(0.6745 * np.std(range_errors))

    def compute_pe_deflection(self) -> float:
        """Probable Error in Deflection (PE_D).

        PE_D = 0.6745 × σ_deflection   (50% within ±PE_D crossrange)
        """
        defl_errors = self.impacts[:, 1] - self.target[1]
        return float(0.6745 * np.std(defl_errors))

    def compute_mean_point_of_impact(self) -> np.ndarray:
        """Mean Point of Impact (MPI)."""
        return np.mean(self.impacts, axis=0)

    def compute_bias(self) -> float:
        """Distance from MPI to target (systematic bias)."""
        mpi = self.compute_mean_point_of_impact()
        return float(np.linalg.norm(mpi - self.target))

    def get_full_report(self) -> Dict:
        """Compute all metrics and return as a dictionary."""
        return {
            "n_samples": self.n_samples,
            "CEP50_m": self.compute_cep50(),
            "CEP90_m": self.compute_cep90(),
            "CEP95_m": self.compute_cep95(),
            "PE_Range_m": self.compute_pe_range(),
            "PE_Deflection_m": self.compute_pe_deflection(),
            "MPI": self.compute_mean_point_of_impact().tolist(),
            "Bias_m": self.compute_bias(),
            "Mean_miss_m": float(np.mean(self._get_miss())),
            "Max_miss_m": float(np.max(self._get_miss())),
            "Min_miss_m": float(np.min(self._get_miss())),
            "Std_miss_m": float(np.std(self._get_miss())),
        }

    def print_report(self, label: str = ""):
        """Pretty-print the full accuracy report."""
        report = self.get_full_report()
        header = f"═══ CEP Analysis Report{' — ' + label if label else ''} ═══"
        print(f"\n{'═' * len(header)}")
        print(header)
        print(f"{'═' * len(header)}")
        print(f"  Samples:           {report['n_samples']}")
        print(f"  CEP50:             {report['CEP50_m']:.1f} m")
        print(f"  CEP90:             {report['CEP90_m']:.1f} m")
        print(f"  CEP95:             {report['CEP95_m']:.1f} m")
        print(f"  PE Range:          {report['PE_Range_m']:.1f} m")
        print(f"  PE Deflection:     {report['PE_Deflection_m']:.1f} m")
        print(f"  Mean Miss:         {report['Mean_miss_m']:.1f} m")
        print(f"  Max Miss:          {report['Max_miss_m']:.1f} m")
        print(f"  Bias:              {report['Bias_m']:.1f} m")
        print(f"  MPI:               ({report['MPI'][0]:.1f}, {report['MPI'][1]:.1f})")
        print(f"{'═' * len(header)}\n")


def plot_dispersion(
    impacts_guided: np.ndarray,
    impacts_unguided: np.ndarray,
    target: np.ndarray,
    save_path: Optional[str] = None,
):
    """Create publication-quality dispersion scatter plot with CEP circles.

    Displays guided vs unguided side-by-side with bullseye rings.
    """
    fig, (ax1, ax2) = plt.subplots(1, 2, figsize=(16, 7))

    target_2d = target[:2]
    ring_radii = [10, 30, 50, 100, 200]
    ring_colors = ["green", "green", "gold", "orange", "red"]

    for ax, impacts, title in [
        (ax1, impacts_unguided, "UNGUIDED"),
        (ax2, impacts_guided, "GUIDED (PGK)"),
    ]:
        # Bullseye rings
        for r, c in zip(ring_radii, ring_colors):
            circle = plt.Circle(target_2d, r, fill=False, color=c,
                                linestyle="--", linewidth=0.8, alpha=0.7)
            ax.add_patch(circle)
            ax.annotate(f"{r}m", xy=(target_2d[0] + r * 0.707, target_2d[1] + r * 0.707),
                        fontsize=7, color=c, alpha=0.8)

        # Impact scatter
        offsets = impacts - target_2d
        miss = np.sqrt(offsets[:, 0] ** 2 + offsets[:, 1] ** 2)
        cep50 = np.percentile(miss, 50)

        ax.scatter(offsets[:, 0], offsets[:, 1], s=12, alpha=0.6,
                   c="red" if "UN" in title else "blue", edgecolors="none")

        # CEP50 circle
        cep_circle = plt.Circle((0, 0), cep50, fill=False, color="black",
                                linewidth=2.5, linestyle="-")
        ax.add_patch(cep_circle)

        # Target crosshair
        ax.axhline(0, color="gray", linewidth=0.5, alpha=0.5)
        ax.axvline(0, color="gray", linewidth=0.5, alpha=0.5)
        ax.plot(0, 0, "k+", markersize=15, markeredgewidth=2)

        max_r = max(ring_radii[-1], np.max(np.abs(offsets)) * 1.2)
        ax.set_xlim(-max_r, max_r)
        ax.set_ylim(-max_r, max_r)
        ax.set_aspect("equal")
        ax.set_xlabel("Downrange Error [m]")
        ax.set_ylabel("Crossrange Error [m]")
        ax.set_title(f"{title}\nCEP50 = {cep50:.1f} m  (n={len(impacts)})", fontsize=13)
        ax.grid(True, alpha=0.3)

    fig.suptitle("155 mm PGK Monte Carlo Dispersion Analysis",
                 fontsize=15, fontweight="bold", y=1.01)
    plt.tight_layout()

    if save_path:
        fig.savefig(save_path, dpi=200, bbox_inches="tight")
        print(f"  Dispersion plot saved to: {save_path}")
    plt.close(fig)
    return fig


def plot_cep_histogram(
    miss_guided: np.ndarray,
    miss_unguided: np.ndarray,
    save_path: Optional[str] = None,
):
    """Plot miss distance histograms with CEP50 markers."""
    fig, (ax1, ax2) = plt.subplots(1, 2, figsize=(14, 5))

    for ax, miss, title, color in [
        (ax1, miss_unguided, "UNGUIDED", "red"),
        (ax2, miss_guided, "GUIDED (PGK)", "blue"),
    ]:
        cep50 = np.percentile(miss, 50)
        bins = max(15, int(np.sqrt(len(miss))))

        ax.hist(miss, bins=bins, alpha=0.7, color=color, edgecolor="white", linewidth=0.5)
        ax.axvline(cep50, color="black", linewidth=2.5, linestyle="--",
                   label=f"CEP50 = {cep50:.1f} m")
        ax.axvline(30, color="green", linewidth=1.5, linestyle=":",
                   label="Target: 30 m")
        ax.set_xlabel("Miss Distance [m]")
        ax.set_ylabel("Count")
        ax.set_title(f"{title} Miss Distribution")
        ax.legend()
        ax.grid(True, alpha=0.3)

    fig.suptitle("CEP Histogram — Guided vs Unguided",
                 fontsize=14, fontweight="bold")
    plt.tight_layout()

    if save_path:
        fig.savefig(save_path, dpi=200, bbox_inches="tight")
        print(f"  Histogram saved to: {save_path}")
    plt.close(fig)
    return fig
