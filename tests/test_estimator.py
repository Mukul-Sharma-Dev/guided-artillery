"""
test_estimator.py — Unit Tests for Extended Kalman Filter
==========================================================
Validates EKF convergence, GPS/Baro correction, and outlier rejection.
"""

import pytest
import numpy as np
import sys
import os

sys.path.insert(0, os.path.join(os.path.dirname(__file__), ".."))

from estimation.state_estimator import ExtendedKalmanFilter
from estimation.filters import (
    matrix_inverse, ensure_positive_definite, mahalanobis_distance,
    complementary_filter,
)


class TestMatrixUtilities:
    """Test filter utility functions."""

    def test_matrix_inverse_identity(self):
        I = np.eye(4)
        I_inv = matrix_inverse(I)
        assert np.allclose(I_inv, I)

    def test_matrix_inverse_2x2(self):
        M = np.array([[2.0, 1.0], [1.0, 3.0]])
        M_inv = matrix_inverse(M)
        product = M @ M_inv
        assert np.allclose(product, np.eye(2), atol=1e-10)

    def test_ensure_positive_definite(self):
        # Create a matrix with a near-zero eigenvalue
        P = np.array([[1.0, 0.99], [0.99, 1.0]])
        P_fixed = ensure_positive_definite(P)
        eigvals = np.linalg.eigvalsh(P_fixed)
        assert np.all(eigvals > 0), f"Eigenvalues: {eigvals}"

    def test_mahalanobis_distance_identity(self):
        y = np.array([3.0, 4.0])
        S = np.eye(2)
        d = mahalanobis_distance(y, S)
        assert abs(d - 5.0) < 0.01  # √(9+16) = 5

    def test_complementary_filter(self):
        meas = np.array([10.0, 20.0])
        pred = np.array([12.0, 18.0])
        result = complementary_filter(meas, pred, alpha=0.3)
        expected = 0.3 * meas + 0.7 * pred
        assert np.allclose(result, expected)


class TestExtendedKalmanFilter:
    """Test EKF initialization, prediction, and correction."""

    def setup_method(self):
        self.ekf = ExtendedKalmanFilter(
            process_noise_accel=5.0,
            gps_pos_noise=2.0,
            gps_vel_noise=0.1,
            baro_noise=3.0,
            gate_threshold=10.0,
        )

    def test_initialization(self):
        pos = np.array([100.0, 200.0, 300.0])
        vel = np.array([500.0, 10.0, 400.0])
        self.ekf.initialize(pos, vel)

        state, _ = self.ekf.get_state()
        assert np.allclose(state[:3], pos)
        assert np.allclose(state[3:6], vel)

    def test_predict_advances_position(self):
        pos0 = np.array([0.0, 0.0, 1000.0])
        vel0 = np.array([500.0, 0.0, 0.0])
        self.ekf.initialize(pos0, vel0)

        accel = np.array([0.0, 0.0, -9.81])
        dt = 0.01
        self.ekf.predict(accel, dt)

        state, _ = self.ekf.get_state()
        # Position should advance by v*dt + 0.5*a*dt²
        expected_x = 0.0 + 500.0 * dt
        assert abs(state[0] - expected_x) < 0.1

    def test_covariance_grows_during_prediction(self):
        self.ekf.initialize(np.zeros(3), np.array([500, 0, 300]))
        _, cov0 = self.ekf.get_state()

        for _ in range(100):
            self.ekf.predict(np.array([0, 0, -9.81]), 0.01)

        _, cov1 = self.ekf.get_state()
        # Covariance should grow without corrections
        assert np.all(cov1 > cov0)

    def test_gps_correction_reduces_uncertainty(self):
        self.ekf.initialize(np.zeros(3), np.array([500, 0, 300]))

        # Predict for a while to build up uncertainty
        for _ in range(100):
            self.ekf.predict(np.array([0, 0, -9.81]), 0.01)

        state_before, cov_before = self.ekf.get_state()

        # GPS correction — provide measurement NEAR the predicted state
        # so it passes the Mahalanobis gate
        gps_pos = state_before[:3] + np.array([1.0, 0.5, -0.5])
        gps_vel = state_before[3:6] + np.array([0.05, 0.02, -0.03])
        accepted = self.ekf.update_gps(gps_pos, gps_vel)

        assert accepted, "GPS measurement should be accepted by gate"
        _, cov_after = self.ekf.get_state()
        # Position covariance should decrease
        assert cov_after[0] < cov_before[0]

    def test_baro_correction(self):
        self.ekf.initialize(np.zeros(3), np.array([500, 0, 300]))

        for _ in range(100):
            self.ekf.predict(np.array([0, 0, -9.81]), 0.01)

        state_before, cov_before = self.ekf.get_state()
        # Provide baro altitude near the predicted z to pass gate
        baro_alt = state_before[2] + 1.0
        accepted = self.ekf.update_baro(baro_alt)

        assert accepted, "Baro measurement should be accepted by gate"
        _, cov_after = self.ekf.get_state()
        # Z covariance should decrease
        assert cov_after[2] < cov_before[2]

    def test_ekf_tracks_constant_velocity(self):
        """EKF should track a constant-velocity target with GPS."""
        vel = np.array([500.0, 10.0, -50.0])
        pos = np.array([0.0, 0.0, 5000.0])
        self.ekf.initialize(pos, vel)

        rng = np.random.default_rng(42)
        dt = 0.01
        errors = []

        for i in range(1000):
            t = i * dt
            true_pos = pos + vel * t
            true_vel = vel

            self.ekf.predict(np.array([0, 0, -9.81]), dt)

            # GPS every 10 steps (10 Hz at 100 Hz IMU)
            if i % 10 == 0:
                gps_pos = true_pos + rng.normal(0, 2.0, 3)
                gps_vel = true_vel + rng.normal(0, 0.1, 3)
                self.ekf.update_gps(gps_pos, gps_vel)

            est_pos = self.ekf.get_position()
            err = np.linalg.norm(est_pos - true_pos)
            errors.append(err)

        # After convergence, error should be small
        final_errors = errors[-100:]
        mean_error = np.mean(final_errors)
        assert mean_error < 5.0, f"Mean tracking error = {mean_error:.2f} m (expected < 5 m)"

    def test_reset(self):
        self.ekf.initialize(np.array([100, 200, 300]), np.array([500, 0, 300]))
        self.ekf.reset()
        state, _ = self.ekf.get_state()
        assert np.allclose(state, 0.0)


if __name__ == "__main__":
    pytest.main([__file__, "-v"])
