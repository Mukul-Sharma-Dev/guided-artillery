"""
serial_bridge.py -- Bidirectional Serial Bridge for PGK HIL Demonstrator
=========================================================================
Connects the Python Streamlit simulation to the STM32 Nucleo flight
computer over USB Serial for Hardware-in-the-Loop (HIL) operation.

Modes:
  - Mode 2 (Pure HIL): Sends simulation telemetry to STM32, STM32 drives
    servos and LEDs based on simulation data.
  - Mode 3 (Full Hybrid): Sends simulation trajectory waypoints AND
    receives real sensor data + computed guidance commands back from STM32.

Protocol:
  Outgoing (PC -> STM32):  $PGK,time,x,y,z,vx,vy,vz,ax,ay,az,alt*XX
  Incoming (STM32 -> PC):  $TLM,time,state,phase,ekf_x,...,fuze_state*XX

References:
  - embedded/README_HARDWARE.md for protocol specification
  - config/mission_config.yaml for simulation parameters
"""

try:
    import serial
    import serial.tools.list_ports
    HAS_SERIAL = True
except ImportError:
    serial = None
    HAS_SERIAL = False
import threading
import queue
import time
import numpy as np
from typing import Optional, Dict, List, Tuple


def compute_checksum(payload: str) -> str:
    """Compute XOR checksum of payload (between $ and *)."""
    chk = 0
    for char in payload:
        chk ^= ord(char)
    return f"{chk:02X}"


def verify_checksum(packet: str) -> bool:
    """Verify XOR checksum of a received packet."""
    try:
        dollar_idx = packet.index('$')
        star_idx = packet.index('*')
        payload = packet[dollar_idx + 1:star_idx]
        received_chk = packet[star_idx + 1:star_idx + 3].strip()
        computed_chk = compute_checksum(payload)
        return computed_chk.upper() == received_chk.upper()
    except (ValueError, IndexError):
        return False


class PGKSerialBridge:
    """Bidirectional serial bridge between Streamlit simulation and STM32.

    Parameters
    ----------
    port : str or None
        Serial port name (e.g., '/dev/tty.usbmodem...', 'COM3').
        If None, auto-detects STM32 Nucleo.
    baud : int
        Baud rate (default 115200).
    """

    def __init__(self, port: Optional[str] = None, baud: int = 115200):
        self.port = port
        self.baud = baud
        self.ser: Optional[serial.Serial] = None
        self.connected = False
        self.running = False

        # Receive buffer and parsed telemetry
        self._rx_thread: Optional[threading.Thread] = None
        self._rx_queue: queue.Queue = queue.Queue(maxsize=100)
        self._latest_telemetry: Dict = {}
        self._telemetry_lock = threading.Lock()

        # Transmit rate limiting
        self._last_tx_time = 0.0
        self._tx_min_interval = 0.01  # 100 Hz max transmit rate

        # Statistics
        self.tx_count = 0
        self.rx_count = 0
        self.rx_errors = 0

    @staticmethod
    def list_ports() -> List[Dict[str, str]]:
        """List available serial ports with descriptions."""
        if not HAS_SERIAL:
            return []
        ports = []
        for p in serial.tools.list_ports.comports():
            ports.append({
                "device": p.device,
                "description": p.description,
                "hwid": p.hwid,
            })
        return ports

    @staticmethod
    def auto_detect_stm32() -> Optional[str]:
        """Auto-detect STM32 Nucleo board on available serial ports."""
        if not HAS_SERIAL:
            return None
        for p in serial.tools.list_ports.comports():
            desc_lower = p.description.lower()
            hwid_lower = p.hwid.lower()
            # STM32 Nucleo boards typically show as STMicroelectronics
            if any(keyword in desc_lower or keyword in hwid_lower
                   for keyword in ["stm", "nucleo", "st-link", "stmicroelectronics"]):
                return p.device
            # Also check for common USB-serial chips used with STM32
            if any(keyword in desc_lower
                   for keyword in ["usbmodem", "cp210x", "ch340", "ftdi"]):
                return p.device
        return None

    def connect(self) -> bool:
        """Open serial connection to STM32."""
        if not HAS_SERIAL:
            return False
        if self.connected:
            return True

        port = self.port or self.auto_detect_stm32()
        if port is None:
            return False

        try:
            self.ser = serial.Serial(
                port=port,
                baudrate=self.baud,
                timeout=0.1,
                write_timeout=1.0,
            )
            self.connected = True
            self.port = port

            # Start receive thread
            self.running = True
            self._rx_thread = threading.Thread(
                target=self._receive_loop,
                daemon=True,
                name="PGK-SerialRX",
            )
            self._rx_thread.start()

            # Allow STM32 to reset after connection
            time.sleep(0.5)
            return True

        except (serial.SerialException, OSError):
            self.connected = False
            return False

    def disconnect(self):
        """Close serial connection."""
        self.running = False
        if self._rx_thread is not None:
            self._rx_thread.join(timeout=2.0)
            self._rx_thread = None

        if self.ser is not None and self.ser.is_open:
            try:
                self.ser.close()
            except Exception:
                pass
        self.connected = False

    def send_telemetry(
        self,
        t: float,
        pos: np.ndarray,
        vel: np.ndarray,
        accel: np.ndarray,
        alt_agl: float,
    ) -> bool:
        """Send simulation telemetry packet to STM32.

        Parameters
        ----------
        t : float
            Simulation time [s].
        pos : ndarray (3,)
            Position [x, y, z] in metres.
        vel : ndarray (3,)
            Velocity [vx, vy, vz] in m/s.
        accel : ndarray (3,)
            Acceleration [ax, ay, az] in m/s^2.
        alt_agl : float
            Altitude above ground level [m].

        Returns
        -------
        success : bool
        """
        if not self.connected or self.ser is None:
            return False

        # Rate limiting
        now = time.monotonic()
        if (now - self._last_tx_time) < self._tx_min_interval:
            return True  # Skip but not an error

        payload = (
            f"PGK,{t:.3f},"
            f"{pos[0]:.2f},{pos[1]:.2f},{pos[2]:.2f},"
            f"{vel[0]:.2f},{vel[1]:.2f},{vel[2]:.2f},"
            f"{accel[0]:.2f},{accel[1]:.2f},{accel[2]:.2f},"
            f"{alt_agl:.2f}"
        )
        chk = compute_checksum(payload)
        packet = f"${payload}*{chk}\n"

        try:
            self.ser.write(packet.encode("ascii"))
            self.tx_count += 1
            self._last_tx_time = now
            return True
        except (serial.SerialException, OSError):
            self.connected = False
            return False

    def send_simulation_frame(self, results: Dict, frame_idx: int) -> bool:
        """Send a single simulation frame to the STM32.

        Convenience wrapper that extracts the relevant arrays from a
        simulation results dictionary (as returned by FlightSimulator.run_single).

        Parameters
        ----------
        results : dict
            Simulation results from FlightSimulator.run_single().
        frame_idx : int
            Index into the time-series arrays.

        Returns
        -------
        success : bool
        """
        t = float(results["time"][frame_idx])
        pos = results["true_position"][frame_idx]
        vel = results["true_velocity"][frame_idx]

        # Compute acceleration from velocity differences
        if frame_idx > 0:
            dt = results["time"][frame_idx] - results["time"][frame_idx - 1]
            if dt > 0:
                accel = (vel - results["true_velocity"][frame_idx - 1]) / dt
            else:
                accel = np.zeros(3)
        else:
            accel = np.zeros(3)

        alt_agl = max(pos[2], 0.0)

        return self.send_telemetry(t, pos, vel, accel, alt_agl)

    def get_hardware_telemetry(self) -> Dict:
        """Get the latest telemetry received from STM32.

        Returns
        -------
        telemetry : dict
            Latest parsed telemetry, or empty dict if none available.
            Keys: time, state, phase, ekf_x, ekf_y, ekf_z,
                  ekf_vx, ekf_vy, ekf_vz, pitch_cmd, yaw_cmd,
                  miss_x, miss_y, fuze_state
        """
        with self._telemetry_lock:
            return dict(self._latest_telemetry)

    def get_status(self) -> Dict:
        """Get connection status and statistics."""
        return {
            "connected": self.connected,
            "port": self.port or "N/A",
            "tx_count": self.tx_count,
            "rx_count": self.rx_count,
            "rx_errors": self.rx_errors,
        }

    # ------------------------------------------------------------------ #
    # Internal receive loop                                                #
    # ------------------------------------------------------------------ #

    def _receive_loop(self):
        """Background thread: read and parse incoming telemetry."""
        buf = ""
        while self.running and self.connected:
            try:
                if self.ser is None or not self.ser.is_open:
                    break

                raw = self.ser.read(256)
                if not raw:
                    continue

                buf += raw.decode("ascii", errors="ignore")

                # Process complete lines
                while "\n" in buf:
                    line, buf = buf.split("\n", 1)
                    line = line.strip()
                    if line.startswith("$TLM"):
                        self._parse_tlm(line)

            except (serial.SerialException, OSError):
                self.connected = False
                break
            except Exception:
                continue

    def _parse_tlm(self, packet: str):
        """Parse incoming $TLM telemetry packet from STM32."""
        if not verify_checksum(packet):
            self.rx_errors += 1
            return

        try:
            # Strip $ prefix and *checksum suffix
            star_idx = packet.index("*")
            payload = packet[1:star_idx]
            fields = payload.split(",")

            if len(fields) < 15 or fields[0] != "TLM":
                self.rx_errors += 1
                return

            telemetry = {
                "time": float(fields[1]),
                "state": fields[2],
                "phase": fields[3],
                "ekf_x": float(fields[4]),
                "ekf_y": float(fields[5]),
                "ekf_z": float(fields[6]),
                "ekf_vx": float(fields[7]),
                "ekf_vy": float(fields[8]),
                "ekf_vz": float(fields[9]),
                "pitch_cmd": float(fields[10]),
                "yaw_cmd": float(fields[11]),
                "miss_x": float(fields[12]),
                "miss_y": float(fields[13]),
                "fuze_state": fields[14],
                "hw_timestamp": time.time(),
            }

            with self._telemetry_lock:
                self._latest_telemetry = telemetry

            self.rx_count += 1

        except (ValueError, IndexError):
            self.rx_errors += 1

    # ------------------------------------------------------------------ #
    # Context manager                                                      #
    # ------------------------------------------------------------------ #

    def __enter__(self):
        self.connect()
        return self

    def __exit__(self, exc_type, exc_val, exc_tb):
        self.disconnect()
        return False


# ---------------------------------------------------------------------- #
# Streamlit integration helper                                             #
# ---------------------------------------------------------------------- #

def stream_simulation_to_hardware(
    bridge: PGKSerialBridge,
    results: Dict,
    playback_speed: float = 1.0,
) -> None:
    """Stream complete simulation results to STM32 in real-time.

    This function blocks and sends frames at the simulation's time rate,
    scaled by playback_speed. Call from a separate thread or use
    send_simulation_frame() for frame-by-frame control.

    Parameters
    ----------
    bridge : PGKSerialBridge
        Connected serial bridge instance.
    results : dict
        Simulation results from FlightSimulator.run_single().
    playback_speed : float
        Time scaling factor (1.0 = real-time, 2.0 = 2x speed).
    """
    times = results["time"]
    n_frames = len(times)

    start_wall = time.monotonic()
    start_sim = times[0]

    for i in range(n_frames):
        if not bridge.connected:
            break

        # Wait until it's time to send this frame
        sim_elapsed = (times[i] - start_sim) / playback_speed
        wall_elapsed = time.monotonic() - start_wall
        wait = sim_elapsed - wall_elapsed
        if wait > 0:
            time.sleep(wait)

        bridge.send_simulation_frame(results, i)


# ---------------------------------------------------------------------- #
# CLI test                                                                 #
# ---------------------------------------------------------------------- #

if __name__ == "__main__":
    print("PGK Serial Bridge - Port Scanner")
    print("=" * 50)

    ports = PGKSerialBridge.list_ports()
    if not ports:
        print("No serial ports found.")
    else:
        for p in ports:
            print(f"  {p['device']:20s}  {p['description']}")

    auto = PGKSerialBridge.auto_detect_stm32()
    if auto:
        print(f"\nAuto-detected STM32: {auto}")
    else:
        print("\nNo STM32 Nucleo auto-detected.")

    print("\nTo test: python serial_bridge.py <port>")
    import sys
    if len(sys.argv) > 1:
        port = sys.argv[1]
        print(f"\nConnecting to {port}...")
        bridge = PGKSerialBridge(port=port)
        if bridge.connect():
            print("Connected. Sending test packets...")
            for i in range(50):
                t = i * 0.1
                pos = np.array([t * 500, 0.0, t * 100 * (1 - t / 5)])
                vel = np.array([500.0, 0.0, 100 * (1 - 2 * t / 5)])
                accel = np.array([0.0, 0.0, -9.81])
                alt = max(pos[2], 0.0)
                bridge.send_telemetry(t, pos, vel, accel, alt)
                time.sleep(0.1)

                hw = bridge.get_hardware_telemetry()
                if hw:
                    print(f"  t={t:.1f}s  HW_fuze={hw.get('fuze_state', '?')}")

            print(f"\nStats: {bridge.get_status()}")
            bridge.disconnect()
        else:
            print("Failed to connect.")
