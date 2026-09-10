"""
test_fuze.py — Unit Tests for Multi-Mode Electronic Fuze State Machine
========================================================================
Validates correct state transitions, safety interlocks, and all three
fuze modes (Proximity, Time, Impact).
"""

import pytest
import sys
import os

sys.path.insert(0, os.path.join(os.path.dirname(__file__), ".."))

from fuze.fuze_simulator import ElectronicFuze, FuzeState, FuzeMode


@pytest.fixture
def default_config():
    return {
        "arm_distance_m": 500.0,
        "arm_time_s": 5.0,
        "setback_threshold_g": 10000,
        "proximity_hob_m": 7.0,
        "impact_decel_threshold_g": 500,
        "time_mode_delay_s": 60.0,
        "default_mode": "IMPACT",
    }


class TestFuzeSafetyInterlocks:
    """Verify the safety arming sequence prevents premature function."""

    def test_initial_state_is_safe(self, default_config):
        fuze = ElectronicFuze(default_config)
        assert fuze.get_state() == FuzeState.SAFE

    def test_no_arming_without_setback(self, default_config):
        fuze = ElectronicFuze(default_config)
        # Low acceleration should NOT trigger arming
        state, _ = fuze.update(t=0.1, acceleration_g=100, altitude=100,
                               distance_to_ground=100, velocity_z=100,
                               downrange_distance=100)
        assert state == FuzeState.SAFE

    def test_setback_triggers_arming(self, default_config):
        fuze = ElectronicFuze(default_config)
        state, event = fuze.update(t=0.001, acceleration_g=15000, altitude=0,
                                   distance_to_ground=0, velocity_z=500,
                                   downrange_distance=0)
        assert state == FuzeState.ARMING
        assert "ARMING" in event

    def test_safe_separation_required(self, default_config):
        fuze = ElectronicFuze(default_config)
        # Launch
        fuze.update(t=0.001, acceleration_g=15000, altitude=0,
                    distance_to_ground=0, velocity_z=500, downrange_distance=0)

        # Not enough distance yet
        state, _ = fuze.update(t=1.0, acceleration_g=10, altitude=1000,
                               distance_to_ground=1000, velocity_z=400,
                               downrange_distance=200)
        assert state == FuzeState.ARMING  # Still arming, not armed

    def test_arms_after_safe_separation(self, default_config):
        fuze = ElectronicFuze(default_config)
        # Launch
        fuze.update(t=0.001, acceleration_g=15000, altitude=0,
                    distance_to_ground=0, velocity_z=500, downrange_distance=0)

        # Achieve safe separation (>500m AND >5s)
        state, _ = fuze.update(t=6.0, acceleration_g=10, altitude=3000,
                               distance_to_ground=3000, velocity_z=300,
                               downrange_distance=1000)
        assert state == FuzeState.ARMED

    def test_cannot_detonate_before_armed(self, default_config):
        """Fuze must NOT detonate directly from SAFE."""
        fuze = ElectronicFuze(default_config)
        # Try high deceleration without arming sequence
        state, _ = fuze.update(t=0.5, acceleration_g=1000, altitude=0,
                               distance_to_ground=0, velocity_z=-200,
                               downrange_distance=100)
        assert state != FuzeState.DETONATED


class TestFuzeImpactMode:
    """Test Impact mode (deceleration detection)."""

    def test_impact_detonation(self, default_config):
        fuze = ElectronicFuze(default_config)
        fuze.set_mode(FuzeMode.IMPACT)

        # Run through full arming sequence
        fuze.update(t=0.001, acceleration_g=15000, altitude=0,
                    distance_to_ground=0, velocity_z=500, downrange_distance=0)
        fuze.update(t=6.0, acceleration_g=10, altitude=3000,
                    distance_to_ground=3000, velocity_z=300, downrange_distance=1000)
        # ARMED → ACTIVE (immediate)
        fuze.update(t=6.001, acceleration_g=10, altitude=3000,
                    distance_to_ground=3000, velocity_z=300, downrange_distance=1001)

        # Impact spike
        state, event = fuze.update(t=60.0, acceleration_g=800, altitude=0,
                                   distance_to_ground=0, velocity_z=-200,
                                   downrange_distance=24000)
        assert state == FuzeState.DETONATED
        assert "IMPACT" in event


class TestFuzeProximityMode:
    """Test Proximity mode (height-of-burst airburst)."""

    def test_proximity_airburst(self, default_config):
        fuze = ElectronicFuze(default_config)
        fuze.set_mode(FuzeMode.PROXIMITY)

        # Arming sequence
        fuze.update(t=0.001, acceleration_g=15000, altitude=0,
                    distance_to_ground=0, velocity_z=500, downrange_distance=0)
        fuze.update(t=6.0, acceleration_g=10, altitude=3000,
                    distance_to_ground=3000, velocity_z=300, downrange_distance=1000)
        fuze.update(t=6.001, acceleration_g=10, altitude=3000,
                    distance_to_ground=3000, velocity_z=300, downrange_distance=1001)

        # Approaching ground — above HOB threshold
        state, _ = fuze.update(t=59.0, acceleration_g=5, altitude=20,
                               distance_to_ground=20, velocity_z=-200,
                               downrange_distance=23500)
        assert state == FuzeState.ACTIVE  # Not yet triggered

        # Below HOB threshold AND descending
        state, event = fuze.update(t=59.5, acceleration_g=5, altitude=5,
                                   distance_to_ground=5, velocity_z=-200,
                                   downrange_distance=23800)
        assert state == FuzeState.DETONATED
        assert "PROXIMITY" in event


class TestFuzeTimeMode:
    """Test Time mode (electronic timer)."""

    def test_time_detonation(self, default_config):
        config = default_config.copy()
        config["time_mode_delay_s"] = 55.0  # Set timer for 55s after launch
        fuze = ElectronicFuze(config)
        fuze.set_mode(FuzeMode.TIME)

        # Arming sequence
        fuze.update(t=0.001, acceleration_g=15000, altitude=0,
                    distance_to_ground=0, velocity_z=500, downrange_distance=0)
        fuze.update(t=6.0, acceleration_g=10, altitude=3000,
                    distance_to_ground=3000, velocity_z=300, downrange_distance=1000)
        fuze.update(t=6.001, acceleration_g=10, altitude=3000,
                    distance_to_ground=3000, velocity_z=300, downrange_distance=1001)

        # Before timer — should NOT detonate
        state, _ = fuze.update(t=50.0, acceleration_g=5, altitude=1000,
                               distance_to_ground=1000, velocity_z=-200,
                               downrange_distance=20000)
        assert state == FuzeState.ACTIVE

        # After timer — should detonate
        state, event = fuze.update(t=55.5, acceleration_g=5, altitude=500,
                                   distance_to_ground=500, velocity_z=-200,
                                   downrange_distance=22000)
        assert state == FuzeState.DETONATED
        assert "TIME" in event


class TestFuzeReset:
    """Test fuze reset for Monte Carlo reuse."""

    def test_reset_returns_to_safe(self, default_config):
        fuze = ElectronicFuze(default_config)
        fuze.update(t=0.001, acceleration_g=15000, altitude=0,
                    distance_to_ground=0, velocity_z=500, downrange_distance=0)
        assert fuze.get_state() == FuzeState.ARMING

        fuze.reset()
        assert fuze.get_state() == FuzeState.SAFE

    def test_telemetry_after_reset(self, default_config):
        fuze = ElectronicFuze(default_config)
        fuze.reset()
        telem = fuze.get_telemetry()
        assert telem["state"] == "SAFE"
        assert telem["setback_detected"] is False


if __name__ == "__main__":
    pytest.main([__file__, "-v"])
