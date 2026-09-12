"""
app.py — PGK-155 Smart Guidance Demonstrator Dashboard
========================================================
Streamlit interactive dashboard for SIH 2026 demonstration.
Integrates with all simulation, estimation, control, and analysis modules.
"""

import streamlit as st
import numpy as np
import plotly.graph_objects as go
import pandas as pd
import yaml
import sys
import os
import time

# Ensure project root is on path
sys.path.insert(0, os.path.dirname(os.path.abspath(__file__)))

# ── Page Configuration ─────────────────────────────────────────────
st.set_page_config(
    page_title="PGK-155 Smart Guidance Demonstrator",
    layout="wide",
    page_icon="🎯",
)

# ── Module Import ──────────────────────────────────────────────────
try:
    from simulation.simulator import FlightSimulator, load_config
    from experiments.cep_analysis import CEPAnalyzer
    MODULES_OK = True
except ImportError as e:
    MODULES_OK = False
    _import_err = str(e)

# ── Custom CSS ─────────────────────────────────────────────────────
st.markdown("""
<style>
    [data-testid="stMetric"] {
        background-color: #1a2634;
        padding: 12px 16px;
        border-radius: 8px;
        border-left: 4px solid #4caf50;
    }
    .stTabs [data-baseweb="tab-list"] {
        gap: 8px;
    }
    .stTabs [data-baseweb="tab"] {
        padding: 8px 20px;
        font-weight: 600;
    }
</style>
""", unsafe_allow_html=True)

st.title("🎯 PGK-155 Smart Multi-Mode Electronic Fuze Demonstrator")
st.caption("Smart India Hackathon 2026 — Yantra India Limited (YIL) Problem Statement")

if not MODULES_OK:
    st.warning(f"⚠️ Simulation modules not fully loaded: `{_import_err}`. Install deps: `pip install -r requirements.txt`")

# ── Session State ──────────────────────────────────────────────────
if "sim_results" not in st.session_state:
    st.session_state.sim_results = None
if "sim_results_unguided" not in st.session_state:
    st.session_state.sim_results_unguided = None
if "mc_guided" not in st.session_state:
    st.session_state.mc_guided = None
if "mc_unguided" not in st.session_state:
    st.session_state.mc_unguided = None

# ── Load Config ────────────────────────────────────────────────────
config_path = os.path.join(os.path.dirname(__file__), "config", "mission_config.yaml")
try:
    with open(config_path) as f:
        CONFIG = yaml.safe_load(f)
except FileNotFoundError:
    CONFIG = None
    st.error("Config file not found. Ensure `config/mission_config.yaml` exists.")


def run_simulation(v0, theta, wind, wind_dir=90.0, target_x=24000.0, target_y=0.0, target_z=0.0, guided=True, fuze_mode="IMPACT"):
    """Execute a single simulation run with given parameters and target coordinates."""
    if not MODULES_OK or CONFIG is None:
        return _mock_simulation(v0, theta, wind, guided, target_x, target_y, target_z)

    import copy
    cfg = copy.deepcopy(CONFIG)
    cfg["shell"]["muzzle_velocity_ms"] = v0
    cfg["shell"]["launch_elevation_deg"] = theta
    cfg["environment"]["wind_speed_ms"] = wind
    cfg["environment"]["wind_direction_deg"] = wind_dir
    cfg["target"]["x_m"] = float(target_x)
    cfg["target"]["y_m"] = float(target_y)
    cfg["target"]["z_m"] = float(target_z)
    cfg["target"]["elevation_asl_m"] = float(target_z)

    sim = FlightSimulator(cfg)
    try:
        return sim.run_single(seed=42, guided=guided, fuze_mode=fuze_mode,
                              wind_speed=wind, wind_direction_deg=wind_dir)
    except TypeError:
        return sim.run_single(seed=42, guided=guided, fuze_mode=fuze_mode,
                              wind_speed=wind)


def _mock_simulation(v0, theta, wind, guided, target_x=24000.0, target_y=0.0, target_z=0.0):
    """Generate mock results when simulation modules are unavailable."""
    t = np.linspace(0, 80, 2000)
    el = np.radians(theta)
    x = v0 * np.cos(el) * t
    z = v0 * np.sin(el) * t - 0.5 * 9.81 * t**2
    y = wind * 2 * np.sin(t * 0.1)

    mask = z >= 0
    t, x, y, z = t[mask], x[mask], y[mask], z[mask]
    n = len(t)

    rng = np.random.default_rng(42)
    ekf_noise = rng.normal(0, 2.0, (n, 3))
    gps_noise = rng.normal(0, 5.0, (n, 3))

    vx = np.gradient(x, t)
    vy = np.gradient(y, t)
    vz = np.gradient(z, t)
    v_mag = np.sqrt(vx**2 + vy**2 + vz**2)

    target = np.array([float(target_x), float(target_y), float(target_z)])
    miss = np.sqrt((x[-1] - target[0])**2 + (y[-1] - target[1])**2)

    return {
        "time": t,
        "true_position": np.column_stack([x, y, z]),
        "true_velocity": np.column_stack([vx, vy, vz]),
        "ekf_position": np.column_stack([x, y, z]) + ekf_noise,
        "ekf_velocity": np.column_stack([vx, vy, vz]),
        "ekf_sigma": np.ones((n, 3)) * 2.0,
        "gps_position": np.column_stack([x, y, z]) + gps_noise,
        "canard_pitch": np.sin(t * 0.2) * (5 if guided else 0),
        "canard_yaw": np.cos(t * 0.3) * (3 if guided else 0),
        "fuze_state": ["SAFE"] * (n // 4) + ["ARMING"] * (n // 8) + ["ARMED"] * (n // 8) + ["ACTIVE"] * (n - n // 4 - n // 4),
        "flight_phase": ["BOOST_ASCENT"] * (n // 2) + ["MIDCOURSE_GUIDANCE"] * (n - n // 2),
        "mach_number": v_mag / 340.0,
        "acceleration_g": np.abs(np.gradient(v_mag, t)) / 9.81,
        "impact_point": np.array([x[-1], y[-1], 0.0]),
        "target": target,
        "miss_distance_m": miss,
        "max_altitude_m": float(np.max(z)),
        "flight_time_s": float(t[-1]),
        "guided": guided,
        "fuze_telemetry": {"state": "DETONATED", "mode": "IMPACT", "event_log": []},
    }


# ═══════════════════════════════════════════════════════════════════
# TABS
# ═══════════════════════════════════════════════════════════════════
tab1, tab2, tab3, tab4, tab5 = st.tabs([
    "🚀 Mission Control",
    "📡 GNC & Sensor Fusion",
    "💣 Multi-Mode Fuze",
    "🎯 Monte Carlo CEP",
    "⚡ System Architecture",
])

# ── TAB 1: Mission Control & 3D Trajectory ────────────────────────
with tab1:
    st.sidebar.header("🎯 Target Coordinates (User Input)")
    target_x = st.sidebar.number_input(
        "Target Downrange X (m)", min_value=10000.0, max_value=35000.0, value=24000.0, step=250.0,
        help="Target distance along firing axis (+X / East)"
    )
    target_y = st.sidebar.number_input(
        "Target Crossrange Y (m)", min_value=-3000.0, max_value=3000.0, value=0.0, step=50.0,
        help="Lateral offset from firing line (+Y North, -Y South)"
    )
    target_z = st.sidebar.number_input(
        "Target Altitude ASL Z (m)", min_value=0.0, max_value=2500.0, value=0.0, step=25.0,
        help="Target elevation Above Sea Level"
    )

    st.sidebar.header("🎮 Shell & Launch Parameters")
    v0 = st.sidebar.slider("Muzzle Velocity (m/s)", 700.0, 900.0, 820.0, step=5.0)
    theta = st.sidebar.slider("Elevation Angle (°)", 30.0, 60.0, 45.0, step=0.5)

    st.sidebar.header("🌬️ Environmental & Guidance")
    wind = st.sidebar.slider("Wind Speed (m/s)", 0.0, 20.0, 5.0, step=0.5)
    wind_dir = st.sidebar.slider("Wind Direction (°) [FROM]", 0.0, 360.0, 90.0, step=5.0,
                                 help="0°=From North, 90°=From East (Headwind), 180°=From South, 270°=From West (Tailwind)")
    guided = st.sidebar.toggle("Enable PGK Guidance", value=True)
    fuze_mode = st.sidebar.selectbox("Fuze Mode", ["IMPACT", "PROXIMITY", "TIME"])

    if st.sidebar.button("🚀 Launch Simulation", type="primary", use_container_width=True):
        with st.spinner("Simulating flight trajectory..."):
            t0 = time.time()
            st.session_state.sim_results = run_simulation(
                v0, theta, wind, wind_dir, target_x, target_y, target_z, guided, fuze_mode
            )
            if guided:
                st.session_state.sim_results_unguided = run_simulation(
                    v0, theta, wind, wind_dir, target_x, target_y, target_z, guided=False, fuze_mode=fuze_mode
                )
            else:
                st.session_state.sim_results_unguided = None
            elapsed = time.time() - t0
            st.sidebar.success(f"✅ Done in {elapsed:.1f}s")

    if st.session_state.sim_results is not None:
        res = st.session_state.sim_results
        u_res = st.session_state.sim_results_unguided

        # Wind Physics Analysis Banner
        w_head = -wind * np.sin(np.deg2rad(wind_dir))   # Wind along projectile flight path (+X)
        w_cross = -wind * np.cos(np.deg2rad(wind_dir))  # Wind perpendicular (+Y = North, -Y = South)
        
        head_str = f"{abs(w_head):.1f} m/s Headwind (Shortens Range)" if w_head < -0.1 else (
            f"{abs(w_head):.1f} m/s Tailwind (Extends Range)" if w_head > 0.1 else "Zero Head/Tailwind"
        )
        cross_str = f"{abs(w_cross):.1f} m/s Crosswind from North (Drifts South -Y)" if w_cross < -0.1 else (
            f"{abs(w_cross):.1f} m/s Crosswind from South (Drifts North +Y)" if w_cross > 0.1 else "Zero Crosswind"
        )

        st.info(
            f"🌬️ **Atmospheric Conditions**: Wind Speed = **{wind:.1f} m/s**, Direction = **{wind_dir:.0f}°** | "
            f"**{head_str}** | **{cross_str}**  \n"
            f"📌 *Note: At Wind = 0 m/s, unguided shell falls short at ~23.3 km due to natural aerodynamic drag. "
            f"Guided PGK canards deploy at t = 2.0s to glide and hit the 24.0 km target.*"
        )

        # Metrics row
        if u_res is not None:
            c1, c2, c3, c4, c5 = st.columns(5)
            c1.metric("Range", f"{res['impact_point'][0]/1000:.1f} km")
            c2.metric("Max Altitude", f"{res['max_altitude_m']/1000:.1f} km")
            c3.metric("Flight Time", f"{res['flight_time_s']:.1f} s")
            c4.metric("Guided Miss", f"{res['miss_distance_m']:.1f} m",
                       delta=f"{'✅ <30m' if res['miss_distance_m'] < 30 else '❌ >30m'}",
                       delta_color="normal" if res['miss_distance_m'] < 30 else "inverse")
            c5.metric("Unguided Miss", f"{u_res['miss_distance_m']:.1f} m",
                      delta=f"{u_res['miss_distance_m']/max(res['miss_distance_m'], 0.1):.1f}x worse",
                      delta_color="inverse")
        else:
            c1, c2, c3, c4, c5 = st.columns(5)
            c1.metric("Range", f"{res['impact_point'][0]/1000:.1f} km")
            c2.metric("Max Altitude", f"{res['max_altitude_m']/1000:.1f} km")
            c3.metric("Flight Time", f"{res['flight_time_s']:.1f} s")
            c4.metric("Miss Distance", f"{res['miss_distance_m']:.1f} m",
                       delta=f"{'✅ <30m' if res['miss_distance_m'] < 30 else '❌ >30m'}",
                       delta_color="normal" if res['miss_distance_m'] < 30 else "inverse")
            c5.metric("Guidance", "GUIDED" if res["guided"] else "UNGUIDED")

        # ── 3D Trajectory Plot ───────────────────────────────────────
        st.subheader("🌐 3D Flight Trajectory")
        pos = res["true_position"]
        vel = res["true_velocity"]
        v_mag = np.linalg.norm(vel, axis=1)

        fig = go.Figure()

        # 1. Guided Trajectory
        traj_name = "Guided Trajectory (PGK)" if res["guided"] else "Unguided Trajectory"
        traj_color = "#2563eb" if res["guided"] else "#dc2626"
        fig.add_trace(go.Scatter3d(
            x=pos[:, 0], y=pos[:, 1], z=pos[:, 2],
            mode="lines+markers",
            marker=dict(size=1.5, color=v_mag, colorscale="Jet",
                        showscale=True, colorbar=dict(title=dict(text="V (m/s)", font=dict(color="#111827")),
                                                      tickfont=dict(color="#111827"), len=0.5)),
            line=dict(color=traj_color, width=4),
            name=traj_name,
        ))

        # 2. Unguided Trajectory (if guided was simulated)
        if u_res is not None:
            u_pos = u_res["true_position"]
            fig.add_trace(go.Scatter3d(
                x=u_pos[:, 0], y=u_pos[:, 1], z=u_pos[:, 2],
                mode="lines",
                line=dict(color="#ef4444", width=3, dash="dot"),
                name=f"Unguided Trajectory (Miss: {u_res['miss_distance_m']:.1f}m)",
            ))
            fig.add_trace(go.Scatter3d(
                x=[u_pos[-1, 0]], y=[u_pos[-1, 1]], z=[u_pos[-1, 2]],
                mode="markers",
                marker=dict(size=8, color="#991b1b", symbol="circle"),
                name=f"Unguided Impact ({u_pos[-1, 0]/1000:.1f}km)",
            ))

        # 3. Key markers (Launch, Apogee, Guided Impact)
        fig.add_trace(go.Scatter3d(
            x=[pos[0, 0]], y=[pos[0, 1]], z=[pos[0, 2]],
            mode="markers", marker=dict(size=8, color="#16a34a", symbol="circle"),
            name="Launch Point",
        ))
        apogee_idx = np.argmax(pos[:, 2])
        fig.add_trace(go.Scatter3d(
            x=[pos[apogee_idx, 0]], y=[pos[apogee_idx, 1]], z=[pos[apogee_idx, 2]],
            mode="markers", marker=dict(size=6, color="#9333ea", symbol="diamond"),
            name=f"Apogee ({pos[apogee_idx, 2]/1000:.1f} km)",
        ))
        fig.add_trace(go.Scatter3d(
            x=[pos[-1, 0]], y=[pos[-1, 1]], z=[pos[-1, 2]],
            mode="markers", marker=dict(size=8, color="#f97316", symbol="circle"),
            name=f"Guided Impact ({pos[-1, 0]/1000:.1f}km)",
        ))

        # 4. Target Location (Red Dot instead of Cross)
        fig.add_trace(go.Scatter3d(
            x=[res["target"][0]], y=[res["target"][1]], z=[res["target"][2]],
            mode="markers+text",
            marker=dict(size=12, color="#dc2626", symbol="circle",
                        line=dict(color="#7f1d1d", width=2)),
            text=["Target"],
            textposition="top center",
            textfont=dict(color="#dc2626", size=13),
            name="Target Location",
        ))

        # 5. Clean White Background, Proportional Aspect Ratio & Centered Camera Fit
        fig.update_layout(
            scene=dict(
                xaxis=dict(
                    title=dict(text="Downrange (m)", font=dict(color="#111827", size=12)),
                    backgroundcolor="#ffffff",
                    gridcolor="#e5e7eb",
                    showbackground=True,
                    zerolinecolor="#9ca3af",
                    tickfont=dict(color="#374151"),
                ),
                yaxis=dict(
                    title=dict(text="Crossrange (m)", font=dict(color="#111827", size=12)),
                    backgroundcolor="#ffffff",
                    gridcolor="#e5e7eb",
                    showbackground=True,
                    zerolinecolor="#9ca3af",
                    tickfont=dict(color="#374151"),
                ),
                zaxis=dict(
                    title=dict(text="Altitude (m)", font=dict(color="#111827", size=12)),
                    backgroundcolor="#ffffff",
                    gridcolor="#e5e7eb",
                    showbackground=True,
                    zerolinecolor="#9ca3af",
                    tickfont=dict(color="#374151"),
                ),
                bgcolor="#ffffff",
                aspectmode="manual",
                aspectratio=dict(x=2.6, y=0.8, z=0.9),
                camera=dict(
                    eye=dict(x=0.0, y=-2.5, z=0.3),
                    center=dict(x=0.0, y=0.0, z=-0.05),
                    up=dict(x=0, y=0, z=1),
                ),
            ),
            paper_bgcolor="#ffffff",
            plot_bgcolor="#ffffff",
            font=dict(color="#111827"),
            margin=dict(l=0, r=0, b=0, t=40),
            height=600,
            legend=dict(
                x=0.02, y=0.96,
                bgcolor="rgba(255, 255, 255, 0.92)",
                bordercolor="#d1d5db",
                borderwidth=1,
                font=dict(color="#111827", size=11),
            ),
            updatemenus=[
                dict(
                    type="buttons",
                    direction="right",
                    x=0.02, y=1.04,
                    showactive=True,
                    bgcolor="#ffffff",
                    bordercolor="#d1d5db",
                    font=dict(color="#111827", size=11),
                    buttons=[
                        dict(
                            label="🔍 Seedha View (Full Arc Fit)",
                            method="relayout",
                            args=[{"scene.camera": dict(eye=dict(x=0.0, y=-2.5, z=0.3), center=dict(x=0.0, y=0.0, z=-0.05), up=dict(x=0, y=0, z=1))}]
                        ),
                        dict(
                            label="🧊 3D Isometric View",
                            method="relayout",
                            args=[{"scene.camera": dict(eye=dict(x=1.6, y=-1.8, z=1.0), center=dict(x=0.0, y=0.0, z=0.0), up=dict(x=0, y=0, z=1))}]
                        ),
                        dict(
                            label="🎯 Top-Down (Crossrange View)",
                            method="relayout",
                            args=[{"scene.camera": dict(eye=dict(x=0.0, y=0.01, z=2.5), center=dict(x=0.0, y=0.0, z=0.0), up=dict(x=0, y=1, z=0))}]
                        ),
                    ]
                )
            ]
        )
        st.plotly_chart(fig, use_container_width=True)

        # ── 2D Comparative Trajectory Views (Shows Dynamic Wind Bending) ────
        st.markdown("### 📊 Trajectory Breakdown: Wind Drift & Range Profile")
        col_drift, col_alt = st.columns(2)

        with col_drift:
            fig_drift = go.Figure()
            # Guided line
            fig_drift.add_trace(go.Scatter(
                x=pos[:, 0], y=pos[:, 1],
                mode="lines",
                line=dict(color="#2563eb", width=3),
                name="Guided PGK Path",
            ))
            # Unguided line
            if u_res is not None:
                fig_drift.add_trace(go.Scatter(
                    x=u_pos[:, 0], y=u_pos[:, 1],
                    mode="lines",
                    line=dict(color="#ef4444", width=2.5, dash="dot"),
                    name=f"Unguided Wind Drift (Miss: {u_res['miss_distance_m']:.0f}m)",
                ))
            # Target
            fig_drift.add_trace(go.Scatter(
                x=[res["target"][0]], y=[res["target"][1]],
                mode="markers+text",
                marker=dict(size=12, color="#dc2626", symbol="circle"),
                text=["Target"], textposition="top right",
                name="Target (24km, 0m)",
            ))
            fig_drift.update_layout(
                title="🎯 Top-Down View: Lateral Wind Drift (X vs Y)",
                xaxis_title="Downrange X (m)",
                yaxis_title="Crossrange Y (m) [Lateral Drift]",
                paper_bgcolor="#ffffff",
                plot_bgcolor="#ffffff",
                font=dict(color="#111827"),
                height=380,
                xaxis=dict(gridcolor="#f3f4f6", zerolinecolor="#d1d5db"),
                yaxis=dict(gridcolor="#f3f4f6", zerolinecolor="#d1d5db"),
                legend=dict(x=0.02, y=0.98, bgcolor="rgba(255,255,255,0.85)"),
                margin=dict(l=40, r=20, b=40, t=50),
            )
            st.plotly_chart(fig_drift, use_container_width=True)

        with col_alt:
            fig_alt = go.Figure()
            # Guided line
            fig_alt.add_trace(go.Scatter(
                x=pos[:, 0], y=pos[:, 2],
                mode="lines",
                line=dict(color="#2563eb", width=3),
                name="Guided Trajectory",
            ))
            # Unguided line
            if u_res is not None:
                fig_alt.add_trace(go.Scatter(
                    x=u_pos[:, 0], y=u_pos[:, 2],
                    mode="lines",
                    line=dict(color="#ef4444", width=2.5, dash="dot"),
                    name=f"Unguided Path (Impact: {u_pos[-1,0]/1000:.1f}km)",
                ))
            # Target
            fig_alt.add_trace(go.Scatter(
                x=[res["target"][0]], y=[res["target"][2]],
                mode="markers+text",
                marker=dict(size=12, color="#dc2626", symbol="circle"),
                text=["Target"], textposition="top right",
                name="Target Location",
            ))
            fig_alt.update_layout(
                title="📈 Side Profile: Altitude vs Downrange (X vs Z)",
                xaxis_title="Downrange X (m)",
                yaxis_title="Altitude Z (m)",
                paper_bgcolor="#ffffff",
                plot_bgcolor="#ffffff",
                font=dict(color="#111827"),
                height=380,
                xaxis=dict(gridcolor="#f3f4f6", zerolinecolor="#d1d5db"),
                yaxis=dict(gridcolor="#f3f4f6", zerolinecolor="#d1d5db"),
                legend=dict(x=0.02, y=0.98, bgcolor="rgba(255,255,255,0.85)"),
                margin=dict(l=40, r=20, b=40, t=50),
            )
            st.plotly_chart(fig_alt, use_container_width=True)
    else:
        st.info("👈 Configure parameters in the sidebar and click **Launch Simulation**")

# ── TAB 2: GNC & Sensor Fusion ────────────────────────────────────
with tab2:
    st.header("📡 Guidance, Navigation & Sensor Fusion")

    if st.session_state.sim_results is not None:
        res = st.session_state.sim_results
        t = res["time"]
        true_pos = res["true_position"]
        ekf_pos = res["ekf_position"]
        gps_pos = res["gps_position"]
        sigma = res["ekf_sigma"]

        # Position error
        err = ekf_pos - true_pos
        col1, col2 = st.columns(2)

        with col1:
            fig_err = go.Figure()
            labels = ["X (Range)", "Y (Cross)", "Z (Alt)"]
            colors = ["#1f77b4", "#ff7f0e", "#2ca02c"]
            for i, (lbl, clr) in enumerate(zip(labels, colors)):
                fig_err.add_trace(go.Scatter(x=t, y=err[:, i], name=f"{lbl} Error",
                                             line=dict(color=clr, width=1.5)))
                fig_err.add_trace(go.Scatter(x=t, y=3 * sigma[:, i], name=f"+3σ {lbl}",
                                             line=dict(color=clr, width=0.5, dash="dash"),
                                             showlegend=False))
                fig_err.add_trace(go.Scatter(x=t, y=-3 * sigma[:, i], name=f"-3σ {lbl}",
                                             line=dict(color=clr, width=0.5, dash="dash"),
                                             fill="tonexty", fillcolor=f"rgba({int(clr[1:3],16)},{int(clr[3:5],16)},{int(clr[5:7],16)},0.1)",
                                             showlegend=False))
            fig_err.update_layout(title="EKF Position Estimation Error", xaxis_title="Time (s)",
                                  yaxis_title="Error (m)", height=400)
            st.plotly_chart(fig_err, use_container_width=True)

        with col2:
            # Velocity / Mach profile
            v_mag = np.linalg.norm(res["true_velocity"], axis=1)
            mach = res.get("mach_number", v_mag / 340.0)
            fig_v = go.Figure()
            fig_v.add_trace(go.Scatter(x=t, y=v_mag, name="Speed (m/s)", line=dict(color="blue")))
            fig_v.add_trace(go.Scatter(x=t, y=mach, name="Mach", yaxis="y2", line=dict(color="red", dash="dash")))
            fig_v.update_layout(
                title="Velocity & Mach Profile",
                xaxis_title="Time (s)", yaxis_title="Speed (m/s)",
                yaxis2=dict(title="Mach", overlaying="y", side="right"),
                height=400,
            )
            st.plotly_chart(fig_v, use_container_width=True)

        # Canard deflections
        fig_can = go.Figure()
        fig_can.add_trace(go.Scatter(x=t, y=res["canard_pitch"], name="Pitch (°)", line=dict(color="blue")))
        fig_can.add_trace(go.Scatter(x=t, y=res["canard_yaw"], name="Yaw (°)", line=dict(color="orange")))
        fig_can.update_layout(title="Canard Deflection Commands", xaxis_title="Time (s)",
                              yaxis_title="Deflection (°)", height=350)
        st.plotly_chart(fig_can, use_container_width=True)
    else:
        st.info("Run simulation in Tab 1 to see GNC telemetry.")

# ── TAB 3: Multi-Mode Fuze ────────────────────────────────────────
with tab3:
    st.header("💣 Multi-Mode Electronic Fuze System")

    col1, col2 = st.columns([2, 1])
    with col1:
        st.subheader("Fuze State Machine (MIL-STD-1316)")
        st.graphviz_chart("""
        digraph {
            rankdir=LR;
            node [shape=box, style=filled, fontsize=12];
            SAFE [fillcolor="#90EE90"];
            ARMING [fillcolor="#FFFF99"];
            ARMED [fillcolor="#FFD700"];
            ACTIVE [fillcolor="#FFA500"];
            DETONATED [fillcolor="#FF6347"];

            SAFE -> ARMING [label="Setback >10,000g"];
            ARMING -> ARMED [label="Safe sep\\n>500m & >5s"];
            ARMED -> ACTIVE [label="Immediate"];
            ACTIVE -> DETONATED [label="Mode trigger"];
        }
        """)

    with col2:
        st.subheader("Operating Modes")
        st.markdown("""
        | Mode | Trigger |
        |------|---------|
        | **Proximity** | HOB < 7m (airburst) |
        | **Time** | Programmable timer |
        | **Impact** | Decel spike > 500g |
        """)

    if st.session_state.sim_results is not None:
        res = st.session_state.sim_results
        states = res["fuze_state"]
        state_map = {"SAFE": 0, "ARMING": 1, "ARMED": 2, "ACTIVE": 3, "DETONATED": 4}
        state_nums = [state_map.get(s, 0) for s in states]

        fig_fz = go.Figure()
        fig_fz.add_trace(go.Scatter(
            x=res["time"], y=state_nums, mode="lines", line_shape="hv",
            line=dict(color="#FF6347", width=3),
        ))
        fig_fz.update_layout(
            title="Fuze State Timeline",
            xaxis_title="Time (s)", yaxis_title="State",
            yaxis=dict(tickvals=[0, 1, 2, 3, 4],
                       ticktext=["SAFE", "ARMING", "ARMED", "ACTIVE", "DETONATED"]),
            height=300,
        )
        st.plotly_chart(fig_fz, use_container_width=True)

        telem = res.get("fuze_telemetry", {})
        if telem.get("event_log"):
            st.subheader("Event Log")
            for evt in telem["event_log"]:
                st.code(evt)

# ── TAB 4: Monte Carlo CEP ────────────────────────────────────────
with tab4:
    st.header("🎯 Monte Carlo CEP Analysis")

    mc_runs = st.slider("Number of Monte Carlo Runs", 10, 200, 50, step=10)

    if st.button("🔄 Run Monte Carlo Comparison", type="primary"):
        if MODULES_OK and CONFIG is not None:
            import copy
            from experiments.monte_carlo import MonteCarloRunner

            progress = st.progress(0.0, text="Initializing...")
            runner = MonteCarloRunner(CONFIG, n_runs=mc_runs)

            def cb(i, n, _):
                progress.progress((i + 1) / n, text=f"Run {i+1}/{n}")

            st.write("**Running unguided batch...**")
            unguided = runner.run_batch(guided=False, progress_callback=cb)
            st.write("**Running guided batch...**")
            guided_results = runner.run_batch(guided=True, progress_callback=cb)
            progress.empty()

            st.session_state.mc_guided = [(r["impact_point"][0], r["impact_point"][1]) for r in guided_results]
            st.session_state.mc_unguided = [(r["impact_point"][0], r["impact_point"][1]) for r in unguided]
        else:
            # Mock data
            rng = np.random.default_rng(42)
            target = CONFIG["target"] if CONFIG else {"x_m": 24000, "y_m": 0}
            tx, ty = target.get("x_m", 24000), target.get("y_m", 0)

            st.session_state.mc_guided = list(zip(
                rng.normal(tx, 12, mc_runs), rng.normal(ty, 12, mc_runs)))
            st.session_state.mc_unguided = list(zip(
                rng.normal(tx + 40, 80, mc_runs), rng.normal(ty + 15, 80, mc_runs)))

        st.success("Monte Carlo complete!")

    if st.session_state.mc_guided is not None:
        target_xy = [CONFIG["target"]["x_m"], CONFIG["target"]["y_m"]] if CONFIG else [24000, 0]

        g_arr = np.array(st.session_state.mc_guided)
        u_arr = np.array(st.session_state.mc_unguided)

        analyzer_g = CEPAnalyzer(g_arr, target_xy) if MODULES_OK else None
        analyzer_u = CEPAnalyzer(u_arr, target_xy) if MODULES_OK else None

        if analyzer_g and analyzer_u:
            rpt_g = analyzer_g.get_full_report()
            rpt_u = analyzer_u.get_full_report()
        else:
            g_miss = np.sqrt((g_arr[:, 0] - target_xy[0])**2 + (g_arr[:, 1] - target_xy[1])**2)
            u_miss = np.sqrt((u_arr[:, 0] - target_xy[0])**2 + (u_arr[:, 1] - target_xy[1])**2)
            rpt_g = {"CEP50_m": np.median(g_miss), "CEP90_m": np.percentile(g_miss, 90), "CEP95_m": np.percentile(g_miss, 95)}
            rpt_u = {"CEP50_m": np.median(u_miss), "CEP90_m": np.percentile(u_miss, 90), "CEP95_m": np.percentile(u_miss, 95)}

        # Metrics
        c1, c2, c3, c4 = st.columns(4)
        c1.metric("Guided CEP50", f"{rpt_g['CEP50_m']:.1f} m",
                   delta=f"{'✅ PASS' if rpt_g['CEP50_m'] < 30 else '❌ FAIL'}")
        c2.metric("Unguided CEP50", f"{rpt_u['CEP50_m']:.1f} m")
        c3.metric("Improvement", f"{rpt_u['CEP50_m']/max(rpt_g['CEP50_m'],0.1):.1f}x")
        c4.metric("Target CEP", "< 30 m", delta="Design Target")

        # Dispersion scatter plot
        g_off = g_arr - target_xy
        u_off = u_arr - target_xy

        fig_mc = go.Figure()
        fig_mc.add_trace(go.Scatter(x=u_off[:, 0], y=u_off[:, 1], mode="markers",
                                     name="Unguided", marker=dict(color="red", size=5, opacity=0.5)))
        fig_mc.add_trace(go.Scatter(x=g_off[:, 0], y=g_off[:, 1], mode="markers",
                                     name="Guided (PGK)", marker=dict(color="blue", size=5, opacity=0.6)))
        fig_mc.add_trace(go.Scatter(x=[0], y=[0], mode="markers",
                                     marker=dict(color="black", size=15, symbol="cross-thin"),
                                     name="Target"))

        # CEP circles
        for r, clr, lbl in [(30, "green", "30m Target"), (rpt_g["CEP50_m"], "blue", f"Guided CEP50"),
                             (rpt_u["CEP50_m"], "red", f"Unguided CEP50")]:
            theta_c = np.linspace(0, 2 * np.pi, 100)
            fig_mc.add_trace(go.Scatter(
                x=r * np.cos(theta_c), y=r * np.sin(theta_c), mode="lines",
                line=dict(color=clr, dash="dash"), name=f"{lbl} ({r:.0f}m)"))

        max_r = max(200, np.max(np.abs(u_off)) * 1.1)
        fig_mc.update_layout(
            title="Impact Dispersion — Guided vs Unguided",
            xaxis_title="Downrange Error (m)", yaxis_title="Crossrange Error (m)",
            xaxis=dict(range=[-max_r, max_r]), yaxis=dict(range=[-max_r, max_r], scaleanchor="x"),
            height=650,
        )
        st.plotly_chart(fig_mc, use_container_width=True)

        # CEP comparison table
        st.subheader("CEP Metrics Comparison")
        df = pd.DataFrame({
            "Metric": ["CEP50", "CEP90", "CEP95"],
            "Unguided (m)": [f"{rpt_u['CEP50_m']:.1f}", f"{rpt_u['CEP90_m']:.1f}", f"{rpt_u['CEP95_m']:.1f}"],
            "Guided PGK (m)": [f"{rpt_g['CEP50_m']:.1f}", f"{rpt_g['CEP90_m']:.1f}", f"{rpt_g['CEP95_m']:.1f}"],
            "Target": ["< 30 m", "—", "—"],
        })
        st.table(df)

# ── TAB 5: System Architecture ────────────────────────────────────
with tab5:
    st.header("⚡ System Architecture & SWaP-C")

    col1, col2 = st.columns(2)
    with col1:
        st.subheader("Component Breakdown")
        data = pd.DataFrame({
            "Component": ["STM32H7 MCU", "GNSS Receiver (u-blox)", "MEMS IMU (ADIS16490)",
                          "Baro Sensor (BMP390)", "4× Canard Servos", "Thermal Battery",
                          "Proximity Sensor (FMCW)", "Fuze Electronics PCB"],
            "Mass (g)": [5, 12, 15, 2, 80, 120, 25, 30],
            "Power (W)": [0.5, 0.8, 0.4, 0.01, 12.0, 0, 1.5, 0.5],
            "Est. Cost (USD)": [15, 25, 200, 5, 120, 80, 150, 40],
        })
        st.dataframe(data, use_container_width=True, hide_index=True)

        total_mass = data["Mass (g)"].sum()
        total_power = data["Power (W)"].sum()
        total_cost = data["Est. Cost (USD)"].sum()
        st.metric("Total PGK Mass", f"{total_mass} g")

    with col2:
        st.subheader("High-G Survival Requirements")
        st.markdown("""
        | Parameter | Value |
        |-----------|-------|
        | **Axial Setback** | > 15,000 g × 10 ms |
        | **Set-Forward** | ~3,000 g |
        | **Radial (Spin)** | > 20,000 g @ 250 rev/s |
        | **Temperature** | -40°C to +63°C |
        | **Vibration** | Per MIL-STD-810H |
        | **EMI/EMC** | Per MIL-STD-461G |
        """)

        st.subheader("Key Standards Compliance")
        st.markdown("""
        - **MIL-STD-1316** — Fuze Safety (S&A)
        - **STANAG 4187** — Fuze Safety Design
        - **STANAG 4369** — Inductive Fuze Setter
        - **AOP-21** — NATO Ammunition Safety
        """)

    st.subheader("⚡ Power Budget Calculator")
    flight_time = st.slider("Mission Flight Time (s)", 10, 150, 80)
    total_energy_j = total_power * flight_time
    battery_capacity_j = 5000  # Thermal battery

    c1, c2, c3 = st.columns(3)
    c1.metric("Total Power Draw", f"{total_power:.1f} W")
    c2.metric("Energy Required", f"{total_energy_j:.0f} J")
    c3.metric("Battery Capacity", f"{battery_capacity_j} J",
              delta=f"{'✅ Sufficient' if total_energy_j < battery_capacity_j else '⚠️ Marginal'}")

    margin = (battery_capacity_j - total_energy_j) / battery_capacity_j * 100
    st.progress(min(total_energy_j / battery_capacity_j, 1.0))
    st.caption(f"Power margin: {margin:.1f}% {'✅' if margin > 10 else '⚠️'}")
