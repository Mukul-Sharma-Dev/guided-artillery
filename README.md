# 🎯 Precision Guidance Kit (PGK) — 155mm Artillery Shell Demonstrator

A scientifically grounded **software technology demonstrator** for a Precision Guidance Kit with Canard Actuation and Multi-Mode Electronic Fuze for a 155mm artillery shell.

> ⚠️ **Safe Demonstrator**: This is a purely software-based simulation. No operational energetics, live hardware, or weapon systems are involved.

---

## 📋 Table of Contents

- [Overview](#overview)
- [Key Results](#key-results)
- [Architecture](#architecture)
- [Project Structure](#project-structure)
- [Getting Started](#getting-started)
  - [Prerequisites](#prerequisites)
  - [Installation](#installation)
  - [Run Unit Tests](#run-unit-tests)
  - [Run a Single Simulation](#run-a-single-simulation)
  - [Run Monte Carlo CEP Validation](#run-monte-carlo-cep-validation)
  - [Launch Streamlit Dashboard](#launch-streamlit-dashboard)
- [Module Descriptions](#module-descriptions)
- [Configuration](#configuration)
- [How Guided vs Unguided Works](#how-guided-vs-unguided-works)

---

## Overview

This project implements a complete closed-loop simulation of a GPS-guided artillery projectile with:

- **3-DOF Flight Dynamics** — RK4 integration, Mach-dependent drag (M107-class ogive + boat-tail)
- **Sensor Models** — IMU (noise + bias), GPS (10 Hz with dropout), Barometric altimeter
- **Extended Kalman Filter (EKF)** — 6-state estimation `[x, y, z, vx, vy, vz]`
- **Proportional Navigation Guidance** — Impact point prediction + ZEM-based correction
- **Canard Actuation** — 4 canards, ±15° deflection, 300°/s slew rate
- **Multi-Mode Electronic Fuze** — Impact / Proximity / Time modes with MIL-STD-1316 safety interlocks
- **Monte Carlo CEP Analysis** — Statistical validation across stochastic perturbations
- **Interactive Dashboard** — 5-tab Streamlit app with Plotly 3D visualization

---

## Key Results

| Metric | Guided (PGK) | Unguided | Improvement |
|--------|:-----------:|:--------:|:-----------:|
| **CEP50** | **17.1 m** | 1260 m | **73.8×** |
| Miss (best run) | 10.9 m | — | — |
| Flight Time | ~93 s | ~93 s | — |
| Max Altitude | ~9.8 km | ~9.8 km | — |
| Range | 24.0 km | 22.6 km | — |

### Fuze State Machine Validation
```
SAFE → ARMING    : Setback detected (15,000g) at t=0.000s
ARMING → ARMED   : Safe separation (2,320m, 5.0s)
ARMED → ACTIVE   : Fuze sensors powered on
ACTIVE → DETONATED: Impact deceleration spike at t=92.9s
```

---

## Architecture

```
┌─────────────────────────────────────────────────────────┐
│                    FlightSimulator                       │
│                                                         │
│  ┌──────────┐   ┌──────────┐   ┌──────────────────┐    │
│  │ ISA Atmo │   │ Gravity  │   │ Dryden Wind      │    │
│  │ + Density│   │ WGS-84   │   │ + Turbulence     │    │
│  └──────────┘   └──────────┘   └──────────────────┘    │
│                                                         │
│  ┌──────────────────────────────────────────────────┐   │
│  │           ProjectileDynamics (RK4)                │   │
│  │  • Mach-dependent Cd lookup                       │   │
│  │  • Canard lift forces (pitch + yaw)               │   │
│  └──────────────────────────────────────────────────┘   │
│         │                              ▲                │
│         ▼                              │                │
│  ┌─────────────┐    ┌────────────────────────────┐     │
│  │ Sensors     │    │ CanardController            │     │
│  │ IMU/GPS/Baro│───▶│  ├─ FlightPhaseManager     │     │
│  └─────────────┘    │  ├─ ImpactPointPredictor   │     │
│         │           │  └─ PN GuidanceLaw         │     │
│         ▼           └────────────────────────────┘     │
│  ┌─────────────┐              ▲                        │
│  │   EKF       │──────────────┘                        │
│  │ 6-state     │                                       │
│  └─────────────┘    ┌────────────────────────────┐     │
│                     │ ElectronicFuze              │     │
│                     │ SAFE→ARMING→ARMED→ACTIVE→DET│     │
│                     │ Modes: Impact/Proximity/Time│     │
│                     └────────────────────────────┘     │
└─────────────────────────────────────────────────────────┘
```

---

## Project Structure

```
smart-guidance-demonstrator/
│
├── config/
│   └── mission_config.yaml        # All mission parameters
│
├── simulation/                    # Physics & environment
│   ├── environment.py             # ISA atmosphere, gravity, wind
│   ├── dynamics.py                # Projectile dynamics, canard forces
│   ├── sensors.py                 # IMU, GPS, Barometric sensor models
│   ├── actuator_model.py          # Canard actuator (rate limit, lag)
│   └── simulator.py              # Main flight loop (RK4 integration)
│
├── estimation/                    # State estimation
│   ├── filters.py                 # Matrix utilities, Mahalanobis gate
│   └── state_estimator.py         # 6-state Extended Kalman Filter
│
├── control/                       # Guidance & control
│   ├── guidance_model.py          # Impact predictor + PN guidance law
│   └── controller.py              # Flight phase manager + canard controller
│
├── fuze/                          # Electronic fuze
│   └── fuze_simulator.py          # Multi-mode state machine (3 modes)
│
├── experiments/                   # Statistical validation
│   ├── monte_carlo.py             # Batch runner with perturbations
│   └── cep_analysis.py            # CEP50/90/95 statistics & plots
│
├── visualization/                 # Plotting
│   ├── plots.py                   # Matplotlib publication plots
│   └── telemetry_stream.py        # Telemetry logger (CSV/JSON)
│
├── tests/                         # Unit tests (40 tests)
│   ├── test_dynamics.py           # Atmosphere, gravity, drag, canard
│   ├── test_fuze.py               # Safety interlocks, all 3 modes
│   └── test_estimator.py          # EKF init, predict, correct, track
│
├── embedded/                      # Hardware-in-the-Loop
│   ├── PGK_Flight_Computer.ino    # Arduino/ESP32 C++ firmware
│   └── README_HARDWARE.md         # Wiring & bench test guide
│
├── docs/                          # Documentation
│   ├── SYSTEM_ARCHITECTURE.md
│   ├── HIGH_G_SURVIVAL_ANALYSIS.md
│   └── PRESENTATION_DECK_GUIDE.md
│
├── app.py                         # Streamlit 5-tab dashboard
└── requirements.txt               # Python dependencies
```

---

## Getting Started

### Prerequisites

- **Python 3.10+** (tested on 3.13)
- **pip** (Python package manager)

### Installation

```bash
# Clone the repository
git clone https://github.com/Mukul-Sharma-Dev/guided-artillery.git
cd guided-artillery

# Install dependencies
pip install -r requirements.txt
```

### Run Unit Tests

```bash
python3 -m pytest tests/ -v
```

Expected output: `40 passed`

### Run a Single Simulation

```bash
python3 -c "
import yaml
from simulation.simulator import FlightSimulator

with open('config/mission_config.yaml') as f:
    config = yaml.safe_load(f)

sim = FlightSimulator(config)

# Guided run
r = sim.run_single(seed=42, guided=True)
print(f'Guided miss:   {r[\"miss_distance_m\"]:.1f} m')
print(f'Fuze state:    {r[\"fuze_telemetry\"][\"state\"]}')
print(f'Flight time:   {r[\"flight_time_s\"]:.1f} s')
print(f'Max altitude:  {r[\"max_altitude_m\"]/1000:.1f} km')

# Unguided comparison
r2 = sim.run_single(seed=42, guided=False)
print(f'Unguided miss: {r2[\"miss_distance_m\"]:.1f} m')
print(f'Improvement:   {r2[\"miss_distance_m\"]/r[\"miss_distance_m\"]:.1f}x')
"
```

### Run Monte Carlo CEP Validation

```bash
python3 -c "
import yaml, numpy as np
from simulation.simulator import FlightSimulator

with open('config/mission_config.yaml') as f:
    config = yaml.safe_load(f)

mc = config['monte_carlo']
n = 50
guided, unguided = [], []

for i in range(n):
    seed = 42 + i * 1000
    rng = np.random.default_rng(seed)
    mv = 820.0 + rng.normal(0, mc['muzzle_velocity_sigma_ms'])
    el = 51.0 + rng.normal(0, mc['elevation_sigma_mil'] * 0.05625)
    ws = max(0, 5.0 + rng.normal(0, mc['wind_speed_sigma_ms']))
    cd = 1.0 + rng.normal(0, mc['drag_variation_pct'] / 100.0)

    sim = FlightSimulator(config)
    r = sim.run_single(seed=seed, guided=True, muzzle_velocity=mv,
                       elevation_deg=el, wind_speed=ws, cd_scale=cd)
    guided.append(r['miss_distance_m'])

    sim2 = FlightSimulator(config)
    r2 = sim2.run_single(seed=seed, guided=False, muzzle_velocity=mv,
                         elevation_deg=el, wind_speed=ws, cd_scale=cd)
    unguided.append(r2['miss_distance_m'])

    if (i+1) % 10 == 0:
        print(f'  Completed {i+1}/{n} runs')

gm, um = np.array(guided), np.array(unguided)
print(f'\\nGuided   CEP50: {np.median(gm):.1f} m')
print(f'Unguided CEP50: {np.median(um):.1f} m')
print(f'Improvement:    {np.median(um)/np.median(gm):.1f}x')
"
```

> ⏱️ Takes ~5–6 minutes for 50 runs.

### Launch Streamlit Dashboard

```bash
streamlit run app.py
```

Opens an interactive browser dashboard with 5 tabs:
1. **3D Trajectory** — Plotly 3D flight path with velocity colormap
2. **GNC & Sensor Fusion** — EKF errors, canard activity, Mach profile
3. **Fuze Timeline** — State machine transitions with safety interlocks
4. **Monte Carlo CEP** — Dispersion scatter plot with CEP circles
5. **System Overview** — Architecture diagram and SWaP-C calculator

---

## Module Descriptions

| Module | Key Classes | Purpose |
|--------|-------------|---------|
| `simulation/dynamics.py` | `ProjectileDynamics` | Mach-dependent drag, canard lift forces, equations of motion |
| `simulation/simulator.py` | `FlightSimulator` | Main RK4 loop integrating all subsystems |
| `estimation/state_estimator.py` | `ExtendedKalmanFilter` | 6-state EKF with IMU prediction, GPS/Baro correction |
| `control/controller.py` | `CanardController` | Flight phase management + guidance-to-deflection conversion |
| `control/guidance_model.py` | `ImpactPointPredictor`, `GuidanceLaw` | Ballistic impact prediction + proportional navigation |
| `fuze/fuze_simulator.py` | `ElectronicFuze` | 5-state safety chain, 3 detonation modes |

---

## Configuration

All parameters are in [`config/mission_config.yaml`](config/mission_config.yaml):

| Section | Key Parameters |
|---------|----------------|
| `shell` | Mass (43.2 kg), muzzle velocity (820 m/s), elevation (51°), drag table |
| `canard` | Area (0.004 m²), ±15° max deflection, 300°/s slew rate |
| `sensors` | IMU noise/bias, GPS rate (10 Hz), Baro rate (20 Hz) |
| `guidance` | Nav gain (6.0), terminal multiplier (2.0), min t_go (0.5s) |
| `fuze` | Arm distance (500 m), arm time (5 s), setback threshold (10,000g) |
| `monte_carlo` | MV σ (5 m/s), elevation σ (1.5 mil), wind σ (3 m/s), Cd σ (3%) |

---

## How Guided vs Unguided Works

The single difference is in `simulation/simulator.py` (line ~299):

```python
# GUIDED: controller computes canard deflections from EKF state
if self.guided and self.controller is not None:
    pitch_cmd, yaw_cmd = self.controller.compute_commands(ekf_state, t)
    actual_p, actual_y = self.actuator.command(pitch_cmd, yaw_cmd, self.dt)

# UNGUIDED: canards stay at zero — pure ballistic flight
else:
    actual_p, actual_y = 0.0, 0.0
```

The canard deflections generate aerodynamic lift forces in `dynamics.py` → `compute_canard_force()`, which steer the trajectory toward the target.

---

## License

This project is for educational and demonstration purposes only.
