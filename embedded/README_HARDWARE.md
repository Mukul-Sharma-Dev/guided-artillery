# PGK Demonstrator Hardware Guide

## 1. System Overview
This repository contains the embedded software and setup instructions for the Hardware-in-the-Loop (HIL) demonstrator of the Smart Multi-Mode Electronic Fuze and Precision Guidance Kit.

## 2. Bill of Materials (BOM)
| Component | Qty | Approx Cost (INR) | Purpose |
|---|---|---|---|
| Arduino Uno / ESP32 | 1 | ₹500 | Main Flight Computer Demo |
| Micro Servos (SG90) | 4 | ₹400 | Canard actuation simulation |
| RGB LED (Common Cathode)| 1 | ₹50 | Fuze state indicator |
| Standard LED (Red) | 1 | ₹10 | Armed status indicator |
| Breadboard & Jumpers | 1 | ₹200 | Interconnection |
| Total | | ₹1160 | |

## 3. Wiring Diagram (ASCII)
```
[ Arduino ]
   Pin 9  ----> Servo 1 (Pitch +) Signal
   Pin 10 ----> Servo 2 (Yaw +) Signal
   Pin 11 ----> Servo 3 (Pitch -) Signal
   Pin 12 ----> Servo 4 (Yaw -) Signal
   
   Pin 13 ----> Resistor (220) ----> Red LED (+) -> GND
   Pin 5  ----> Resistor (220) ----> RGB LED (R) -> GND
   Pin 6  ----> Resistor (220) ----> RGB LED (G) -> GND
   Pin 7  ----> Resistor (220) ----> RGB LED (B) -> GND
   
   5V     ----> All Servo VCC
   GND    ----> All Servo GND & LED GND
```

## 4. Setup Instructions
1. Install **Arduino IDE**.
2. Install the standard `Servo` library.
3. Open `PGK_Flight_Computer.ino`.
4. Select your board (e.g., Arduino Uno) and COM port.
5. Compile and upload.

## 5. Serial Protocol
The flight computer receives simulated telemetry via Serial (115200 baud).
Format:
`$PGK,time,x,y,z,vx,vy,vz,ax,ay,az,alt_agl*checksum\n`
- `time`: Simulation time (s)
- `x,y,z`: Position (m)
- `vx,vy,vz`: Velocity (m/s)
- `ax,ay,az`: Acceleration (m/s^2)
- `alt_agl`: Altitude Above Ground Level (m)
- `checksum`: XOR of all characters between `$` and `*` as HEX.

## 6. Python Companion Script
A python script can be used to send test data.
```python
import serial
import time

ser = serial.Serial('COM3', 115200) # Replace COM3

def checksum(data):
    chk = 0
    for char in data:
        chk ^= ord(char)
    return f"{chk:02X}"

def send_packet(t, x, y, z, vx, vy, vz, ax, ay, az, alt):
    payload = f"PGK,{t},{x},{y},{z},{vx},{vy},{vz},{ax},{ay},{az},{alt}"
    packet = f"${payload}*{checksum(payload)}\n"
    ser.write(packet.encode())

# Send safe data
send_packet(0, 0,0,0, 0,0,0, 0,0,0, 0)
```

## 7. Bench Test Procedure
1. Power the Arduino and connect via USB.
2. Observe RGB LED (Should be Blue - SAFE).
3. Send a packet with `t=1.0` and `ax=150.0`. Observe RGB LED turn Orange (ARMING).
4. Send `t=3.0`. Observe RGB LED turn Yellow (ARMED) and Pin 13 LED turn ON.
5. Send `vz=-10.0`. Observe RGB LED turn Green (ACTIVE). Servos will deflect based on target coordinates.
6. Send `alt=2.0` and `t=6.0`. Observe RGB LED turn Red (DETONATED).

## 8. Troubleshooting
- **Servos twitching:** Ensure adequate 5V power (use external supply if needed).
- **No LED change:** Check serial baud rate (115200) and checksum correctness.

## 9. SIH Booth Setup
Mount the 4 servos in a cross configuration mimicking the PGK canards. Secure the Arduino and LEDs in a visible enclosure. Run the Python simulator on a laptop connected to the Arduino to show live hardware response during the presentation.
