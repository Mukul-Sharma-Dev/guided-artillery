"""
fuze_simulator.py — Multi-Mode Electronic Fuze State Machine
==============================================================
Simulates the safety-arming-detonation sequence of a multi-mode
electronic fuze supporting Proximity (airburst), Time, and Impact modes.

Safety interlocks follow MIL-STD-1316 / STANAG 4187 principles:
  - Launch setback detection required before arming
  - Safe separation distance/time must be achieved
  - No premature detonation before ARMED state

This is a SOFTWARE SIMULATION ONLY — no energetics or pyrotechnic
circuits are modeled.
"""

from enum import Enum, auto
from typing import Tuple, Dict, Optional


class FuzeMode(Enum):
    """Selectable fuze operating modes."""
    PROXIMITY = auto()   # Height-of-burst airburst
    TIME = auto()        # Electronic timer
    IMPACT = auto()      # Ground-contact deceleration


class FuzeState(Enum):
    """Fuze arming/detonation states (sequential safety chain)."""
    SAFE = auto()        # Pre-launch: all interlocks engaged
    ARMING = auto()      # Launch detected: setback confirmed
    ARMED = auto()       # Safe separation achieved
    ACTIVE = auto()      # Fuze sensors/mode logic powered on
    DETONATED = auto()   # Burst event triggered
    DUDE = auto()        # Dud / fail-safe (did not function)


class ElectronicFuze:
    """Multi-mode electronic fuze state machine.

    Parameters
    ----------
    config : dict
        Fuze configuration with keys:
        - arm_distance_m (float): Min downrange for arming [m].
        - arm_time_s (float): Min flight time for arming [s].
        - setback_threshold_g (float): Launch detection threshold [g].
        - proximity_hob_m (float): Height-of-burst for proximity [m].
        - impact_decel_threshold_g (float): Impact detection [g].
        - time_mode_delay_s (float): Programmable timer setting [s].
        - default_mode (str): Default fuze mode name.
    """

    def __init__(self, config: dict):
        self.arm_distance = config.get("arm_distance_m", 500.0)
        self.arm_time = config.get("arm_time_s", 5.0)
        self.setback_threshold = config.get("setback_threshold_g", 10_000)
        self.proximity_hob = config.get("proximity_hob_m", 7.0)
        self.impact_decel_g = config.get("impact_decel_threshold_g", 500)
        self.time_delay = config.get("time_mode_delay_s", 0.0)

        # Current state
        self.state = FuzeState.SAFE

        # Parse mode from config
        mode_str = config.get("default_mode", "IMPACT").upper()
        try:
            self.mode = FuzeMode[mode_str]
        except KeyError:
            self.mode = FuzeMode.IMPACT

        # Internal tracking
        self._setback_detected = False
        self._launch_time: Optional[float] = None
        self._detonation_time: Optional[float] = None
        self._event_log: list = []

    def set_mode(self, mode: FuzeMode):
        """Configure fuze mode (must be called before launch)."""
        if self.state == FuzeState.SAFE:
            self.mode = mode
            self._event_log.append(f"Mode set to {mode.name}")

    def set_time_delay(self, delay_s: float):
        """Set programmable time delay for TIME mode [s]."""
        self.time_delay = delay_s

    def update(
        self,
        t: float,
        acceleration_g: float,
        altitude: float,
        distance_to_ground: float,
        velocity_z: float,
        downrange_distance: float,
    ) -> Tuple[FuzeState, str]:
        """Advance fuze state machine by one time step.

        Parameters
        ----------
        t : float
            Simulation time [s].
        acceleration_g : float
            Current axial acceleration magnitude [g].
        altitude : float
            Current altitude AGL [m].
        distance_to_ground : float
            Slant range to ground or target [m].
        velocity_z : float
            Vertical velocity [m/s]  (negative = descending).
        downrange_distance : float
            Horizontal distance from launch [m].

        Returns
        -------
        state : FuzeState
        event : str
            Description of any state transition, or empty string.
        """
        event = ""

        if self.state == FuzeState.DETONATED or self.state == FuzeState.DUDE:
            return self.state, ""

        # ── SAFE → ARMING ──────────────────────────────────────────────
        if self.state == FuzeState.SAFE:
            if acceleration_g >= self.setback_threshold:
                self._setback_detected = True
                self._launch_time = t
                self.state = FuzeState.ARMING
                event = f"SAFE→ARMING: Setback detected ({acceleration_g:.0f}g) at t={t:.3f}s"
                self._event_log.append(event)

        # ── ARMING → ARMED ─────────────────────────────────────────────
        elif self.state == FuzeState.ARMING:
            flight_time = (t - self._launch_time) if self._launch_time is not None else 0.0
            if downrange_distance >= self.arm_distance and flight_time >= self.arm_time:
                self.state = FuzeState.ARMED
                event = f"ARMING→ARMED: Safe separation ({downrange_distance:.0f}m, {flight_time:.1f}s)"
                self._event_log.append(event)

        # ── ARMED → ACTIVE (immediate) ─────────────────────────────────
        elif self.state == FuzeState.ARMED:
            self.state = FuzeState.ACTIVE
            event = "ARMED→ACTIVE: Fuze sensors powered on"
            self._event_log.append(event)

        # ── ACTIVE → DETONATED (mode-dependent) ────────────────────────
        elif self.state == FuzeState.ACTIVE:
            flight_time = (t - self._launch_time) if self._launch_time is not None else 0.0

            if self.mode == FuzeMode.PROXIMITY:
                # Airburst: trigger when close to ground AND descending
                if distance_to_ground <= self.proximity_hob and velocity_z < 0:
                    self.state = FuzeState.DETONATED
                    self._detonation_time = t
                    event = (
                        f"ACTIVE→DETONATED [PROXIMITY]: "
                        f"HOB={distance_to_ground:.1f}m at t={t:.3f}s"
                    )
                    self._event_log.append(event)

            elif self.mode == FuzeMode.TIME:
                # Timer: trigger at programmed time after launch
                if self.time_delay > 0 and flight_time >= self.time_delay:
                    self.state = FuzeState.DETONATED
                    self._detonation_time = t
                    event = (
                        f"ACTIVE→DETONATED [TIME]: "
                        f"Timer expired at t={t:.3f}s (set={self.time_delay:.3f}s)"
                    )
                    self._event_log.append(event)

            elif self.mode == FuzeMode.IMPACT:
                # Impact: trigger on high deceleration spike
                if acceleration_g >= self.impact_decel_g:
                    self.state = FuzeState.DETONATED
                    self._detonation_time = t
                    event = (
                        f"ACTIVE→DETONATED [IMPACT]: "
                        f"Deceleration spike {acceleration_g:.0f}g at t={t:.3f}s"
                    )
                    self._event_log.append(event)

        return self.state, event

    def get_state(self) -> FuzeState:
        return self.state

    def get_telemetry(self) -> Dict:
        """Return fuze telemetry dict for logging/display."""
        return {
            "state": self.state.name,
            "mode": self.mode.name,
            "setback_detected": self._setback_detected,
            "launch_time": self._launch_time,
            "detonation_time": self._detonation_time,
            "event_log": list(self._event_log),
        }

    def reset(self):
        """Reset fuze to SAFE state for Monte Carlo reuse."""
        self.state = FuzeState.SAFE
        self._setback_detected = False
        self._launch_time = None
        self._detonation_time = None
        self._event_log = []
