// PGK Flight Computer - Hardware-in-the-Loop Demonstrator
// For Arduino Uno/Nano, ESP32, or STM32 BluePill
// Serial protocol: receives flight telemetry, runs fuze state machine, drives servos

#include <Servo.h>

#if defined(ESP8266) || defined(ESP32)
#include <esp_task_wdt.h>
#define WDT_TIMEOUT 3
#elif defined(ARDUINO_AVR_UNO) || defined(ARDUINO_AVR_NANO)
#include <avr/wdt.h>
#endif

// --- Pin Definitions ---
const int PIN_SERVO_1 = 9;  // Pitch +
const int PIN_SERVO_2 = 10; // Yaw +
const int PIN_SERVO_3 = 11; // Pitch -
const int PIN_SERVO_4 = 12; // Yaw -
const int PIN_LED_ARMED = 13;
const int PIN_LED_R = 5;
const int PIN_LED_G = 6;
const int PIN_LED_B = 7;

// --- Servos ---
Servo servo1, servo2, servo3, servo4;

// --- Fuze State Machine ---
enum FuzeState {
    SAFE,
    ARMING,
    ARMED,
    ACTIVE,
    DETONATED
};
FuzeState currentState = SAFE;

// --- Telemetry Variables ---
float t_sim, p_x, p_y, p_z, v_x, v_y, v_z, a_x, a_y, a_z, alt_agl;

// --- Guidance ---
// Simple proportional navigation placeholders
float canard_pitch = 0.0;
float canard_yaw = 0.0;

// --- Buffer ---
const int BUF_SIZE = 128;
char serialBuf[BUF_SIZE];
int bufIndex = 0;

void setup() {
    Serial.begin(115200);
    
    // Attach Servos
    servo1.attach(PIN_SERVO_1);
    servo2.attach(PIN_SERVO_2);
    servo3.attach(PIN_SERVO_3);
    servo4.attach(PIN_SERVO_4);
    
    // Initialize Pins
    pinMode(PIN_LED_ARMED, OUTPUT);
    pinMode(PIN_LED_R, OUTPUT);
    pinMode(PIN_LED_G, OUTPUT);
    pinMode(PIN_LED_B, OUTPUT);
    
    // Watchdog
#if defined(ESP32)
    esp_task_wdt_init(WDT_TIMEOUT, true);
    esp_task_wdt_add(NULL);
#elif defined(ARDUINO_AVR_UNO)
    wdt_enable(WDTO_2S);
#endif

    setLED(0, 0, 255); // SAFE: Blue
    centerServos();
    
    Serial.println("PGK Flight Computer Initialized.");
}

void loop() {
    // Reset Watchdog
#if defined(ESP32)
    esp_task_wdt_reset();
#elif defined(ARDUINO_AVR_UNO)
    wdt_reset();
#endif

    // Read Serial Data
    while (Serial.available() > 0) {
        char c = Serial.read();
        if (c == '\n') {
            serialBuf[bufIndex] = '\0';
            parseTelemetry(serialBuf);
            bufIndex = 0;
        } else {
            if (bufIndex < BUF_SIZE - 1) {
                serialBuf[bufIndex++] = c;
            }
        }
    }
    
    // Run State Machine
    updateFuzeState();
    
    // Run Guidance if ACTIVE
    if (currentState == ACTIVE) {
        computeGuidance();
    }
    
    // Output Actuation
    actuateCanards();
}

void setLED(int r, int g, int b) {
    analogWrite(PIN_LED_R, r);
    analogWrite(PIN_LED_G, g);
    analogWrite(PIN_LED_B, b);
}

void centerServos() {
    servo1.write(90);
    servo2.write(90);
    servo3.write(90);
    servo4.write(90);
}

// Parses $PGK,time,x,y,z,vx,vy,vz,ax,ay,az,alt_agl*checksum
void parseTelemetry(char* packet) {
    if (packet[0] != '$') return;
    
    // Find checksum
    char* starPtr = strchr(packet, '*');
    if (!starPtr) return;
    
    // Verify checksum (XOR of characters between $ and *)
    byte calculatedChecksum = 0;
    for (char* p = packet + 1; p < starPtr; p++) {
        calculatedChecksum ^= *p;
    }
    
    int receivedChecksum = strtol(starPtr + 1, NULL, 16);
    if (calculatedChecksum != receivedChecksum) {
        return; // Checksum failed
    }
    
    // Extract variables
    char* token = strtok(packet, ",*"); // Skip $PGK
    if (!token) return;
    
    t_sim   = atof(strtok(NULL, ","));
    p_x     = atof(strtok(NULL, ","));
    p_y     = atof(strtok(NULL, ","));
    p_z     = atof(strtok(NULL, ","));
    v_x     = atof(strtok(NULL, ","));
    v_y     = atof(strtok(NULL, ","));
    v_z     = atof(strtok(NULL, ","));
    a_x     = atof(strtok(NULL, ","));
    a_y     = atof(strtok(NULL, ","));
    a_z     = atof(strtok(NULL, ","));
    alt_agl = atof(strtok(NULL, "*"));
}

void updateFuzeState() {
    switch (currentState) {
        case SAFE:
            setLED(0, 0, 255); // Blue
            // Check for setback acceleration > 10,000g (scaled for sim)
            // Simulating with > 100 m/s^2 for HIL demonstration
            if (a_x > 100.0 || t_sim > 0.5) {
                currentState = ARMING;
            }
            break;
            
        case ARMING:
            setLED(255, 165, 0); // Orange
            // Spin and time delay conditions (e.g., > 2.0 sec)
            if (t_sim > 2.0) {
                currentState = ARMED;
                digitalWrite(PIN_LED_ARMED, HIGH);
            }
            break;
            
        case ARMED:
            setLED(255, 255, 0); // Yellow
            // Apogee detection or guidance activation point
            if (v_z < 0) { // Passing apogee
                currentState = ACTIVE;
            }
            break;
            
        case ACTIVE:
            setLED(0, 255, 0); // Green
            // Height of burst (HOB) or point detonation
            if (alt_agl < 5.0 && t_sim > 5.0) { // 5m HOB
                currentState = DETONATED;
            }
            break;
            
        case DETONATED:
            setLED(255, 0, 0); // Red
            digitalWrite(PIN_LED_ARMED, LOW);
            canard_pitch = 0;
            canard_yaw = 0;
            break;
    }
}

void computeGuidance() {
    // Basic proportional guidance logic
    // Using reference area and deflection limits +/- 15 deg
    // In a real system, this would run the EKF and PN algorithms
    // Here we use a simplified mock response for hardware visualization
    
    // Fake target error based on cross-track drift
    float target_y = 0.0;
    float target_x = 10000.0; // dummy target
    
    float error_y = target_y - p_y;
    
    // Simple P-controller
    float k_p = 0.5;
    
    // Command deflections in degrees
    canard_yaw = constrain(error_y * k_p, -15.0, 15.0);
    canard_pitch = constrain((target_x - p_x) * 0.01, -15.0, 15.0);
}

void actuateCanards() {
    // Map +/- 15 deg to servo angles
    // Servos centered at 90 deg. 
    // Let's amplify visual effect for the demo by scaling to +/- 45 deg servo travel
    
    int pitch_servo_cmd = 90 + (int)(canard_pitch * 3);
    int yaw_servo_cmd   = 90 + (int)(canard_yaw * 3);
    
    // Differential deflection for roll control is ignored in this simple 2-axis model
    servo1.write(pitch_servo_cmd); // Pitch +
    servo3.write(180 - pitch_servo_cmd); // Pitch - (opposite)
    
    servo2.write(yaw_servo_cmd); // Yaw +
    servo4.write(180 - yaw_servo_cmd); // Yaw - (opposite)
}
