import numpy as np
import matplotlib.pyplot as plt
from mpl_toolkits.mplot3d import Axes3D
import os

plt.style.use('seaborn-v0_8-whitegrid')

def plot_trajectory_3d(results, save_path=None):
    fig = plt.figure(figsize=(10, 8))
    ax = fig.add_subplot(111, projection='3d')
    
    t = results['time']
    x = results['state'][:, 0]
    y = results['state'][:, 1]
    z = results['state'][:, 2]
    
    # Plot trajectory colored by velocity
    vx = results['state'][:, 3]
    vy = results['state'][:, 4]
    vz = results['state'][:, 5]
    v = np.sqrt(vx**2 + vy**2 + vz**2)
    
    p = ax.scatter(x, y, z, c=v, cmap='jet', marker='.', s=1)
    fig.colorbar(p, label='Velocity (m/s)')
    
    # Mark launch point
    ax.scatter(x[0], y[0], z[0], c='g', marker='o', s=50, label='Launch')
    
    # Mark apogee
    apogee_idx = np.argmax(z)
    ax.scatter(x[apogee_idx], y[apogee_idx], z[apogee_idx], c='purple', marker='^', s=50, label='Apogee')
    
    # Mark target if available in results
    if 'target_pos' in results:
        tx, ty, tz = results['target_pos']
        ax.scatter(tx, ty, tz, c='r', marker='x', s=100, label='Target')
        
    ax.set_xlabel('Downrange (m)')
    ax.set_ylabel('Crossrange (m)')
    ax.set_zlabel('Altitude (m)')
    ax.set_title('3D Trajectory')
    ax.legend()
    
    if save_path:
        plt.savefig(save_path, bbox_inches='tight')
    plt.close()

def plot_trajectory_2d(results, save_path=None):
    fig, (ax1, ax2) = plt.subplots(2, 1, figsize=(10, 10))
    
    x = results['state'][:, 0]
    y = results['state'][:, 1]
    z = results['state'][:, 2]
    
    ax1.plot(x, z, 'b-')
    ax1.set_xlabel('Downrange (m)')
    ax1.set_ylabel('Altitude (m)')
    ax1.set_title('Side View')
    ax1.grid(True)
    
    ax2.plot(x, y, 'r-')
    ax2.set_xlabel('Downrange (m)')
    ax2.set_ylabel('Crossrange (m)')
    ax2.set_title('Top View')
    ax2.grid(True)
    
    plt.tight_layout()
    if save_path:
        plt.savefig(save_path, bbox_inches='tight')
    plt.close()

def plot_sensor_comparison(results, save_path=None):
    if 'ekf_state' not in results or 'gps_meas' not in results:
        return
        
    fig, axes = plt.subplots(3, 1, figsize=(12, 15), sharex=True)
    t = results['time']
    true_pos = results['state'][:, 0:3]
    
    ekf_pos = results['ekf_state'][:, 0:3]
    gps = results['gps_meas']
    
    labels = ['X (m)', 'Y (m)', 'Z (m)']
    
    for i in range(3):
        axes[i].plot(t, true_pos[:, i], 'k-', label='True')
        # GPS might have nan values where not updated, filter them out for plotting if needed
        # Assuming gps arrays match length of t
        axes[i].scatter(t, gps[:, i], c='r', s=5, alpha=0.5, label='GPS')
        axes[i].plot(t, ekf_pos[:, i], 'b--', label='EKF')
        axes[i].set_ylabel(labels[i])
        axes[i].grid(True)
        axes[i].legend()
        
    axes[-1].set_xlabel('Time (s)')
    axes[0].set_title('Sensor Fusion vs Truth')
    
    plt.tight_layout()
    if save_path:
        plt.savefig(save_path, bbox_inches='tight')
    plt.close()

def plot_ekf_errors(results, save_path=None):
    if 'ekf_state' not in results:
        return
        
    fig, axes = plt.subplots(3, 1, figsize=(10, 12), sharex=True)
    t = results['time']
    error = results['ekf_state'][:, 0:3] - results['state'][:, 0:3]
    
    labels = ['X Error (m)', 'Y Error (m)', 'Z Error (m)']
    for i in range(3):
        axes[i].plot(t, error[:, i], 'b-')
        if 'ekf_cov' in results:
            std = np.sqrt(results['ekf_cov'][:, i, i])
            axes[i].fill_between(t, 3*std, -3*std, color='r', alpha=0.2, label='±3σ')
            axes[i].legend()
        axes[i].set_ylabel(labels[i])
        axes[i].grid(True)
        
    axes[-1].set_xlabel('Time (s)')
    axes[0].set_title('EKF Position Estimation Error')
    
    plt.tight_layout()
    if save_path:
        plt.savefig(save_path, bbox_inches='tight')
    plt.close()

def plot_canard_deflections(results, save_path=None):
    if 'canard_commands' not in results:
        return
        
    fig, ax = plt.subplots(figsize=(10, 6))
    t = results['time']
    cmd = np.array(results['canard_commands'])
    
    if cmd.ndim == 2 and cmd.shape[1] >= 2:
        ax.plot(t, np.degrees(cmd[:, 0]), 'b-', label='Pitch Canard')
        ax.plot(t, np.degrees(cmd[:, 1]), 'r-', label='Yaw Canard')
    
    ax.set_xlabel('Time (s)')
    ax.set_ylabel('Deflection (deg)')
    ax.set_title('Canard Commands')
    ax.grid(True)
    ax.legend()
    
    plt.tight_layout()
    if save_path:
        plt.savefig(save_path, bbox_inches='tight')
    plt.close()

def plot_fuze_timeline(results, save_path=None):
    if 'fuze_states' not in results:
        return
        
    fig, ax = plt.subplots(figsize=(10, 4))
    t = results['time']
    states = results['fuze_states']
    
    # Map state strings to integers
    unique_states = list(set(states))
    state_map = {s: i for i, s in enumerate(unique_states)}
    y = [state_map[s] for s in states]
    
    ax.step(t, y, where='post')
    ax.set_yticks(range(len(unique_states)))
    ax.set_yticklabels(unique_states)
    
    ax.set_xlabel('Time (s)')
    ax.set_title('Fuze State Timeline')
    ax.grid(True)
    
    plt.tight_layout()
    if save_path:
        plt.savefig(save_path, bbox_inches='tight')
    plt.close()

def plot_velocity_profile(results, save_path=None):
    fig, ax1 = plt.subplots(figsize=(10, 6))
    
    t = results['time']
    vx = results['state'][:, 3]
    vy = results['state'][:, 4]
    vz = results['state'][:, 5]
    v = np.sqrt(vx**2 + vy**2 + vz**2)
    mach = v / 343.0  # Approx speed of sound
    
    ax1.plot(t, v, 'b-')
    ax1.set_xlabel('Time (s)')
    ax1.set_ylabel('Velocity (m/s)', color='b')
    ax1.tick_params(axis='y', labelcolor='b')
    
    ax2 = ax1.twinx()
    ax2.plot(t, mach, 'r--')
    ax2.set_ylabel('Mach Number', color='r')
    ax2.tick_params(axis='y', labelcolor='r')
    
    plt.title('Velocity and Mach Profile')
    ax1.grid(True)
    
    plt.tight_layout()
    if save_path:
        plt.savefig(save_path, bbox_inches='tight')
    plt.close()

def plot_altitude_profile(results, save_path=None):
    fig, (ax1, ax2) = plt.subplots(1, 2, figsize=(15, 6))
    
    t = results['time']
    x = results['state'][:, 0]
    z = results['state'][:, 2]
    
    ax1.plot(t, z, 'b-')
    ax1.set_xlabel('Time (s)')
    ax1.set_ylabel('Altitude (m)')
    ax1.set_title('Altitude vs Time')
    ax1.grid(True)
    
    ax2.plot(x, z, 'g-')
    ax2.set_xlabel('Downrange Distance (m)')
    ax2.set_ylabel('Altitude (m)')
    ax2.set_title('Altitude vs Downrange')
    ax2.grid(True)
    
    plt.tight_layout()
    if save_path:
        plt.savefig(save_path, bbox_inches='tight')
    plt.close()
