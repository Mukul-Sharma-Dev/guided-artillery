/*
 * main.cpp -- PGK Flight Computer Main Firmware
 * ================================================
 * Entry point for the STM32H743 Nucleo-based PGK demonstrator.
 * Runs the complete closed-loop flight computer:
 *   Sensors -> EKF -> Guidance -> Actuator -> Telemetry
 *
 * Modes of operation:
 *   Mode 2 (Pure HIL):   Simulation data via serial drives everything
 *   Mode 3 (Hybrid):     Real sensors + simulation data coexist
 *
 * Hardware:
 *   - STM32 Nucleo-H743ZI2 (Cortex-M7, 480 MHz)
 *   - MPU9250 IMU via SPI1
 *   - BMP280 Barometer via I2C1
 *   - NEO-6M GPS via UART2
 *   - 4x SG90 Servos via PWM (TIM1)
 *   - RGB LED + Armed LED via GPIO
 *
 * Build: PlatformIO with Arduino framework
 *   pio run -e nucleo_h743zi2 --target upload
 */

#ifdef ARDUINO
#include <Arduino.h>
#endif
#include "pgk_config.h"
#include "pgk_ekf.h"
#include "pgk_guidance.h"
#include "pgk_fuze.h"
#include "pgk_actuator.h"
#include "pgk_sensors.h"
#include "pgk_serial_bridge.h"

/* -------------------------------------------------------------------- */
/* Global Module Instances                                              */
/* -------------------------------------------------------------------- */
static PGK_EKF           ekf;
static PGK_Guidance       guidance;
static PGK_Fuze           fuze;
static PGK_Actuator       actuator;
static PGK_SerialBridge   serial_bridge;

/* Sensor data buffers */
static PGK_IMU_Data   imu_data;
static PGK_GPS_Data   gps_data;
static PGK_Baro_Data  baro_data;

/* Simulation data buffers (from serial) */
static PGK_IMU_Data   sim_imu;
static PGK_GPS_Data   sim_gps;
static PGK_Baro_Data  sim_baro;
static float           sim_time = 0.0f;
static float           sim_alt_agl = 0.0f;
static bool            sim_data_valid = false;

/* Timing */
static uint32_t last_loop_ms = 0;
static uint32_t loop_count = 0;
static float    flight_time = 0.0f;
static bool     flight_started = false;

/* Target coordinates (loaded from simulation or hardcoded) */
static float target[3] = {PGK_TARGET_X, PGK_TARGET_Y, PGK_TARGET_Z};

/* -------------------------------------------------------------------- */
/* LED Control                                                          */
/* -------------------------------------------------------------------- */
static void set_rgb_led(uint8_t r, uint8_t g, uint8_t b) {
    analogWrite(PGK_PIN_LED_R, r);
    analogWrite(PGK_PIN_LED_G, g);
    analogWrite(PGK_PIN_LED_B, b);
}

static void update_leds(PGK_FuzeState state) {
    switch (state) {
        case PGK_FUZE_SAFE:
            set_rgb_led(0, 0, 255);               /* Blue */
            digitalWrite(PGK_PIN_LED_ARMED, LOW);
            break;
        case PGK_FUZE_ARMING:
            set_rgb_led(255, 165, 0);              /* Orange */
            digitalWrite(PGK_PIN_LED_ARMED, LOW);
            break;
        case PGK_FUZE_ARMED:
            set_rgb_led(255, 255, 0);              /* Yellow */
            digitalWrite(PGK_PIN_LED_ARMED, HIGH);
            break;
        case PGK_FUZE_ACTIVE:
            set_rgb_led(0, 255, 0);                /* Green */
            digitalWrite(PGK_PIN_LED_ARMED, HIGH);
            break;
        case PGK_FUZE_DETONATED:
            set_rgb_led(255, 0, 0);                /* Red */
            digitalWrite(PGK_PIN_LED_ARMED, LOW);
            break;
        case PGK_FUZE_DUDE:
            set_rgb_led(128, 0, 128);              /* Purple (malfunction) */
            digitalWrite(PGK_PIN_LED_ARMED, LOW);
            break;
    }
}

/* -------------------------------------------------------------------- */
/* Fuze state name for telemetry                                        */
/* -------------------------------------------------------------------- */
static const char* fuze_state_name(PGK_FuzeState s) {
    switch (s) {
        case PGK_FUZE_SAFE:       return "SAFE";
        case PGK_FUZE_ARMING:     return "ARMING";
        case PGK_FUZE_ARMED:      return "ARMED";
        case PGK_FUZE_ACTIVE:     return "ACTIVE";
        case PGK_FUZE_DETONATED:  return "DETONATED";
        case PGK_FUZE_DUDE:       return "DUDE";
        default:                  return "UNKNOWN";
    }
}

static const char* phase_name(PGK_FlightPhase p) {
    switch (p) {
        case PGK_PHASE_BOOST:     return "BOOST";
        case PGK_PHASE_MIDCOURSE: return "MIDCOURSE";
        case PGK_PHASE_TERMINAL:  return "TERMINAL";
        case PGK_PHASE_POST:      return "POST_IMPACT";
        default:                  return "UNKNOWN";
    }
}

/* ================================================================== */
/* SETUP                                                              */
/* ================================================================== */
void setup() {
    /* 1. Serial (USB) for telemetry bridge */
    pgk_serial_init(&serial_bridge, PGK_SERIAL_BAUD);

    /* 2. LED GPIO */
    pinMode(PGK_PIN_LED_R, OUTPUT);
    pinMode(PGK_PIN_LED_G, OUTPUT);
    pinMode(PGK_PIN_LED_B, OUTPUT);
    pinMode(PGK_PIN_LED_ARMED, OUTPUT);
    set_rgb_led(0, 0, 255);  /* SAFE = Blue */

    /* 3. Sensor buses */
    pgk_sensors_init();

    /* 4. Initialize algorithm modules */
    float init_pos[3] = {0.0f, 0.0f, 0.0f};
    float init_vel[3] = {0.0f, 0.0f, 0.0f};
    pgk_ekf_init_default(&ekf, init_pos, init_vel);
    pgk_guidance_init(&guidance, target[0], target[1], target[2]);
    pgk_fuze_init_with_mode(&fuze, PGK_FUZE_MODE_IMPACT);
    pgk_actuator_init(&actuator);

    /* 5. Startup message */
    Serial.println("PGK Flight Computer v2.0 (STM32H743)");
    Serial.println("Mode 3: Hybrid (Real Sensors + Simulation)");
    Serial.println("Waiting for simulation data...");

    last_loop_ms = millis();
}

/* ================================================================== */
/* MAIN LOOP (100 Hz target)                                          */
/* ================================================================== */
void loop() {
    uint32_t now_ms = millis();

    /* Enforce 100 Hz loop rate (10 ms period) */
    if ((now_ms - last_loop_ms) < PGK_LOOP_PERIOD_MS) {
        return;
    }
    float dt = (float)(now_ms - last_loop_ms) / 1000.0f;
    if (dt > 0.1f) dt = 0.01f;  /* Cap dt to avoid EKF divergence */
    last_loop_ms = now_ms;
    loop_count++;

    /* ── 1. Check for simulation data from laptop ────────────── */
    if (pgk_serial_process_rx(&serial_bridge)) {
        const char *pkt = pgk_serial_get_packet(&serial_bridge);
        sim_data_valid = pgk_sensors_parse_serial(
            pkt, &sim_imu, &sim_gps, &sim_baro, &sim_time, &sim_alt_agl
        );
        if (sim_data_valid && !flight_started) {
            flight_started = true;
            /* Re-init EKF with simulation initial conditions */
            pgk_ekf_reset_state(&ekf, sim_gps.position, sim_gps.velocity);
        }
    }

    /* ── 2. Read real sensors (Mode 3) ───────────────────────── */
    bool real_imu_ok  = pgk_imu_read(&imu_data);
    bool real_gps_ok  = false;
    bool real_baro_ok = false;

    /* GPS: check every iteration (non-blocking, ~10 Hz from receiver) */
    real_gps_ok = pgk_gps_read(&gps_data);

    /* Baro: read at 20 Hz (every 5th iteration of 100 Hz loop) */
    if (loop_count % 5 == 0) {
        real_baro_ok = pgk_baro_read(&baro_data);
    }

    /* ── 3. Data Fusion: select best source ──────────────────── */
    /*
     * Mode 3 priority:
     *   - IMU: prefer real sensor (captures actual bench vibration/noise)
     *   - GPS: prefer simulation (real GPS on bench is stationary)
     *   - Baro: prefer real sensor (shows real pressure noise)
     *
     * If real sensor is unavailable, fall back to simulation data.
     */
    float accel[3], gps_pos[3], gps_vel[3], baro_alt;
    bool  gps_available = false;
    bool  baro_available = false;

    /* IMU: prefer real, fall back to sim */
    if (real_imu_ok) {
        accel[0] = imu_data.accel[0];
        accel[1] = imu_data.accel[1];
        accel[2] = imu_data.accel[2];
    } else if (sim_data_valid) {
        accel[0] = sim_imu.accel[0];
        accel[1] = sim_imu.accel[1];
        accel[2] = sim_imu.accel[2];
    } else {
        accel[0] = 0.0f;
        accel[1] = 0.0f;
        accel[2] = -9.81f;  /* Gravity only */
    }

    /* GPS: prefer simulation trajectory (bench GPS is stationary) */
    if (sim_data_valid && sim_gps.valid) {
        gps_pos[0] = sim_gps.position[0];
        gps_pos[1] = sim_gps.position[1];
        gps_pos[2] = sim_gps.position[2];
        gps_vel[0] = sim_gps.velocity[0];
        gps_vel[1] = sim_gps.velocity[1];
        gps_vel[2] = sim_gps.velocity[2];
        gps_available = true;
    } else if (real_gps_ok) {
        gps_pos[0] = gps_data.position[0];
        gps_pos[1] = gps_data.position[1];
        gps_pos[2] = gps_data.position[2];
        gps_vel[0] = gps_data.velocity[0];
        gps_vel[1] = gps_data.velocity[1];
        gps_vel[2] = gps_data.velocity[2];
        gps_available = true;
    }

    /* Baro: prefer real sensor */
    if (real_baro_ok) {
        baro_alt = baro_data.altitude_m;
        baro_available = true;
    } else if (sim_data_valid && sim_baro.valid) {
        baro_alt = sim_baro.altitude_m;
        baro_available = true;
    }

    /* Track flight time */
    if (sim_data_valid) {
        flight_time = sim_time;
    } else if (flight_started) {
        flight_time += dt;
    }

    /* ── 4. EKF Navigation ───────────────────────────────────── */

    /* Predict with IMU acceleration (subtract gravity estimate) */
    float nav_accel[3] = {
        accel[0],
        accel[1],
        accel[2] - 9.81f  /* Remove gravity for EKF model */
    };
    pgk_ekf_predict(&ekf, nav_accel, dt);

    /* GPS correction (when available) */
    if (gps_available) {
        pgk_ekf_update_gps(&ekf, gps_pos, gps_vel);
    }

    /* Baro correction (when available, at 20 Hz) */
    if (baro_available && (loop_count % 5 == 0)) {
        pgk_ekf_update_baro(&ekf, baro_alt);
    }

    /* Get EKF state estimate */
    float ekf_state[6], ekf_cov[6];
    pgk_ekf_get_state_and_cov(&ekf, ekf_state, ekf_cov);

    /* ── 5. Guidance & Control ───────────────────────────────── */
    float pitch_cmd = 0.0f, yaw_cmd = 0.0f;

    if (flight_started) {
        pgk_guidance_compute(&guidance, flight_time, ekf_state,
                             &pitch_cmd, &yaw_cmd);
    }

    /* ── 6. Actuator with dynamics ───────────────────────────── */
    float actual_pitch, actual_yaw;
    pgk_actuator_command(&actuator, pitch_cmd, yaw_cmd, dt,
                         &actual_pitch, &actual_yaw);
    pgk_actuator_write_pwm(&actuator);

    /* ── 7. Fuze State Machine ───────────────────────────────── */
    float accel_mag = sqrtf(accel[0]*accel[0] + accel[1]*accel[1] + accel[2]*accel[2]);
    float accel_g = accel_mag / 9.80665f;
    float altitude = ekf_state[2];
    float alt_agl = sim_data_valid ? sim_alt_agl : (altitude > 0 ? altitude : 0.0f);

    pgk_fuze_update(&fuze, flight_time, ekf_state, accel_g, alt_agl);
    PGK_FuzeState fstate = pgk_fuze_get_state(&fuze);

    /* Update LED indicators */
    update_leds(fstate);

    /* If detonated, center servos */
    if (fstate == PGK_FUZE_DETONATED) {
        pgk_actuator_reset(&actuator);
        pgk_actuator_write_pwm(&actuator);
    }

    /* ── 8. Send Telemetry to Laptop (10 Hz) ─────────────────── */
    if (loop_count % 10 == 0) {
        float miss_x = target[0] - ekf_state[0];
        float miss_y = target[1] - ekf_state[1];

        pgk_serial_send_telemetry(
            &serial_bridge,
            flight_time,
            "RUNNING",
            phase_name(pgk_guidance_get_phase(&guidance)),
            ekf_state,
            actual_pitch,
            actual_yaw,
            miss_x,
            miss_y,
            fuze_state_name(fstate)
        );
    }
}
