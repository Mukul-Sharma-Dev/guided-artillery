"""
filters.py — Matrix Utilities & Filter Primitives
===================================================
Helper functions for Kalman filtering, covariance management,
and outlier rejection used by the state estimator.
"""

import numpy as np


def matrix_inverse(M: np.ndarray) -> np.ndarray:
    """Invert matrix *M* with singularity protection.

    If *M* is near-singular (condition number > 1e12), a small
    ridge is added to the diagonal before inversion.
    """
    cond = np.linalg.cond(M)
    if cond > 1e12:
        M = M + np.eye(M.shape[0]) * 1e-9
    return np.linalg.inv(M)


def ensure_positive_definite(P: np.ndarray, eps: float = 1e-8) -> np.ndarray:
    """Force *P* to be symmetric positive-definite.

    1. Symmetrise:  P ← (P + P^T) / 2
    2. Eigenvalue repair:  clamp all eigenvalues ≥ eps
    """
    P = 0.5 * (P + P.T)
    eigvals, eigvecs = np.linalg.eigh(P)
    eigvals = np.maximum(eigvals, eps)
    return eigvecs @ np.diag(eigvals) @ eigvecs.T


def mahalanobis_distance(innovation: np.ndarray, S: np.ndarray) -> float:
    """Compute Mahalanobis distance  d² = y^T S⁻¹ y.

    Parameters
    ----------
    innovation : ndarray (n,)
        Measurement innovation vector.
    S : ndarray (n, n)
        Innovation covariance matrix.

    Returns
    -------
    d : float
        Mahalanobis distance (scalar, not squared).
    """
    S_inv = matrix_inverse(S)
    d_sq = float(innovation @ S_inv @ innovation)
    return np.sqrt(max(d_sq, 0.0))


def complementary_filter(
    measurement: np.ndarray,
    prediction: np.ndarray,
    alpha: float,
) -> np.ndarray:
    """Simple complementary filter:  output = α·meas + (1-α)·pred."""
    return alpha * measurement + (1.0 - alpha) * prediction
