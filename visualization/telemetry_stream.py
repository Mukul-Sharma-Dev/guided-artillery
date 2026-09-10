import os
import json
import csv
import pandas as pd
import numpy as np

class TelemetryLogger:
    def __init__(self, log_dir='telemetry_logs'):
        self.log_dir = log_dir
        os.makedirs(self.log_dir, exist_ok=True)
        self.buffer = []
        
    def log_step(self, t, state, ekf_state, sensors, canard, fuze_state):
        entry = {
            'time': float(t),
            'x': float(state[0]),
            'y': float(state[1]),
            'z': float(state[2]),
            'vx': float(state[3]),
            'vy': float(state[4]),
            'vz': float(state[5]),
            'fuze_state': str(fuze_state)
        }
        
        if ekf_state is not None:
            entry.update({
                'ekf_x': float(ekf_state[0]),
                'ekf_y': float(ekf_state[1]),
                'ekf_z': float(ekf_state[2]),
                'ekf_vx': float(ekf_state[3]),
                'ekf_vy': float(ekf_state[4]),
                'ekf_vz': float(ekf_state[5])
            })
            
        if sensors is not None and 'gps' in sensors:
            entry.update({
                'gps_x': float(sensors['gps'][0]),
                'gps_y': float(sensors['gps'][1]),
                'gps_z': float(sensors['gps'][2])
            })
            
        if canard is not None:
            entry.update({
                'canard_pitch': float(canard[0]),
                'canard_yaw': float(canard[1])
            })
            
        self.buffer.append(entry)
        
    def save_csv(self, filename):
        if not self.buffer:
            return
        path = os.path.join(self.log_dir, filename)
        keys = self.buffer[0].keys()
        with open(path, 'w', newline='') as f:
            dict_writer = csv.DictWriter(f, fieldnames=keys)
            dict_writer.writeheader()
            dict_writer.writerows(self.buffer)
            
    def save_json(self, filename):
        if not self.buffer:
            return
        path = os.path.join(self.log_dir, filename)
        with open(path, 'w') as f:
            json.dump(self.buffer, f, indent=4)
            
    def get_dataframe(self):
        return pd.DataFrame(self.buffer)

def format_telemetry_display(t, state, ekf_state, fuze_state) -> str:
    vel = np.linalg.norm(state[3:6])
    mach = vel / 343.0
    alt = state[2]
    
    display = f"=== TELEMETRY @ T+{t:.2f}s ===\n"
    display += f"Alt: {alt:.1f} m | Vel: {vel:.1f} m/s (M{mach:.2f})\n"
    display += f"Pos: [{state[0]:.1f}, {state[1]:.1f}, {state[2]:.1f}] m\n"
    display += f"Fuze State: {fuze_state}\n"
    
    if ekf_state is not None:
        ekf_err = np.linalg.norm(state[0:3] - ekf_state[0:3])
        display += f"EKF Pos Err: {ekf_err:.2f} m\n"
        
    return display
