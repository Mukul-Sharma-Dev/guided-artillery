"""
test_dynamics.py — Unit Tests for Trajectory Physics
======================================================
Validates aerodynamic model, ballistic range, energy conservation,
and canard force generation.
"""

import pytest
import numpy as np
import sys
import os

# Add project root to path
sys.path.insert(0, os.path.join(os.path.dirname(__file__), ".."))

from simulation.environment import ISAAtmosphere, GravityModel, WindModel
from simulation.dynamics import ProjectileDynamics
from simulation.actuator_model import CanardActuator


class TestISAAtmosphere:
    """Verify ISA atmosphere model against standard values."""

    def setup_method(self):
        self.atm = ISAAtmosphere()

    def test_sea_level(self):
        T, P, rho, a = self.atm.get_properties(0.0)
        assert abs(T - 288.15) < 0.01, f"Sea-level T = {T}"
        assert abs(P - 101325) < 1.0, f"Sea-level P = {P}"
        assert abs(rho - 1.225) < 0.01, f"Sea-level ρ = {rho}"
        assert abs(a - 340.3) < 0.5, f"Sea-level a = {a}"

    def test_5km_altitude(self):
        T, P, rho, a = self.atm.get_properties(5000.0)
        # ISA at 5 km: T ≈ 255.65 K, P ≈ 54048 Pa
        assert abs(T - 255.65) < 0.5
        assert abs(P - 54048) < 200

    def test_tropopause(self):
        T, P, rho, a = self.atm.get_properties(11000.0)
        assert abs(T - 216.65) < 0.5

    def test_stratosphere_isothermal(self):
        T1, _, _, _ = self.atm.get_properties(12000.0)
        T2, _, _, _ = self.atm.get_properties(15000.0)
        assert abs(T1 - T2) < 0.1, "Stratosphere should be isothermal"

    def test_density_decreases_with_altitude(self):
        _, _, rho0, _ = self.atm.get_properties(0.0)
        _, _, rho5, _ = self.atm.get_properties(5000.0)
        _, _, rho10, _ = self.atm.get_properties(10000.0)
        assert rho0 > rho5 > rho10


class TestGravityModel:
    """Verify gravity model."""

    def test_sea_level(self):
        # At latitude 45°, Somigliana gravity ≈ 9.806 m/s² (close to G0)
        gm = GravityModel(latitude_deg=45.0)
        assert abs(gm.g(0.0) - 9.806) < 0.01
        # Latitude variation: equator < pole
        gm_eq = GravityModel(latitude_deg=0.0)
        gm_pole = GravityModel(latitude_deg=90.0)
        assert gm_eq.g(0.0) < gm_pole.g(0.0)

    def test_gravity_decreases(self):
        gm = GravityModel()
        g0 = gm.g(0.0)
        g10 = gm.g(10000.0)
        assert g10 < g0
        # At 10 km, gravity should decrease by ~0.3%
        assert abs(g10 - g0) / g0 < 0.01


class TestProjectileDynamics:
    """Validate projectile dynamics and ballistic range."""

    def setup_method(self):
        self.cd_table = [
            [0.0, 0.15], [0.9, 0.25], [1.0, 0.35],
            [1.2, 0.33], [2.0, 0.25], [3.0, 0.20],
        ]
        self.dyn = ProjectileDynamics(
            mass_kg=43.2,
            ref_area_m2=0.018869,
            cd_table=self.cd_table,
            canard_area_m2=0.001,
            cl_delta=3.0,
        )

    def test_drag_at_rest(self):
        """Zero velocity should produce zero drag."""
        drag = self.dyn.compute_drag(np.zeros(3), 0.0)
        assert np.allclose(drag, 0.0)

    def test_drag_opposes_motion(self):
        """Drag force must oppose the velocity vector."""
        vel = np.array([500.0, 0.0, 0.0])
        drag = self.dyn.compute_drag(vel, 0.0)
        assert drag[0] < 0, "Drag should oppose +x motion"
        assert abs(drag[1]) < abs(drag[0]) * 0.01
        assert abs(drag[2]) < abs(drag[0]) * 0.01

    def test_drag_increases_with_speed(self):
        """Drag magnitude should increase with speed (quadratic)."""
        v1 = np.array([200.0, 0.0, 0.0])
        v2 = np.array([400.0, 0.0, 0.0])
        d1 = np.linalg.norm(self.dyn.compute_drag(v1, 0.0))
        d2 = np.linalg.norm(self.dyn.compute_drag(v2, 0.0))
        # At same altitude/Mach regime, drag ∝ V², so d2/d1 ≈ 4
        ratio = d2 / d1
        assert 2.0 < ratio < 8.0, f"Drag ratio = {ratio}"

    def test_canard_force_at_rest(self):
        """No canard force at zero velocity."""
        f = self.dyn.compute_canard_force(0.1, 0.0, np.zeros(3), 0.0)
        assert np.allclose(f, 0.0)

    def test_canard_force_direction(self):
        """Canard pitch deflection should produce a vertical force component."""
        vel = np.array([300.0, 0.0, -100.0])
        f = self.dyn.compute_canard_force(0.1, 0.0, vel, 5000.0)
        assert abs(f[2]) > 0, "Pitch canard should produce vertical force"

    def test_ballistic_range_order_of_magnitude(self):
        """Simplified ballistic integration should yield ~20-30 km range."""
        # Simple Euler integration to check range is reasonable
        state = np.array([0.0, 0.0, 0.0, 820 * np.cos(np.pi/4), 0.0, 820 * np.sin(np.pi/4)])
        dt = 0.1
        for _ in range(100_000):
            deriv = self.dyn.derivatives(0, state)
            state = state + dt * deriv
            if state[2] < 0 and state[0] > 1000:
                break
        range_km = state[0] / 1000.0
        assert 15.0 < range_km < 35.0, f"Ballistic range = {range_km:.1f} km"

    def test_gravity_pulls_down(self):
        """Derivatives at rest should show downward acceleration."""
        state = np.array([0.0, 0.0, 1000.0, 0.0, 0.0, 0.0])
        deriv = self.dyn.derivatives(0, state)
        assert deriv[5] < -9.0, f"az = {deriv[5]}, expected < -9.0"


class TestCanardActuator:
    """Verify actuator rate limiting and saturation."""

    def test_rate_limiting(self):
        act = CanardActuator(max_deflection_deg=15, slew_rate_deg_s=300, time_constant_s=0.001)
        # Command a large step; at 300°/s and dt=0.01s, max change = 3°
        p, y = act.command(15.0, 0.0, 0.01)
        assert abs(p) <= 3.5, f"Rate limit violated: {p}"

    def test_deflection_saturation(self):
        act = CanardActuator(max_deflection_deg=15, slew_rate_deg_s=10000, time_constant_s=0.001)
        # Command well beyond limits
        for _ in range(100):
            p, y = act.command(50.0, -50.0, 0.1)
        assert abs(p) <= 15.01
        assert abs(y) <= 15.01

    def test_reset(self):
        act = CanardActuator()
        act.command(10.0, 5.0, 1.0)
        act.reset()
        assert act.pitch_deg == 0.0
        assert act.yaw_deg == 0.0


if __name__ == "__main__":
    pytest.main([__file__, "-v"])
