/*
 * pgk_actuator.h -- Canard Actuator Model with PWM Output
 * =========================================================
 * Port of simulation/actuator_model.py to embedded C.
 * First-order-lag actuator with rate limiting, deflection
 * saturation, and servo PWM output for SG90 micro servos.
 *
 * Reference: mission_config.yaml canard section
 */

#ifndef PGK_ACTUATOR_H
#define PGK_ACTUATOR_H

#ifdef __cplusplus
extern "C" {
#endif

/* Actuator state */
typedef struct {
    float pitch_deg;       /* Current pitch deflection [deg] */
    float yaw_deg;         /* Current yaw deflection [deg] */
    float max_defl;        /* Maximum deflection limit [deg] */
    float slew_rate;       /* Maximum slew rate [deg/s] */
    float tau;             /* First-order lag time constant [s] */
} PGK_Actuator;

/* Initialize actuator with default parameters from config */
void pgk_actuator_init(PGK_Actuator *act);

/* Process a canard deflection command through actuator dynamics.
 *   1. Clamp desired to +/-max_deflection
 *   2. Rate limit (slew rate)
 *   3. First-order lag
 *   4. Final saturation
 * Returns actual pitch and yaw deflections via pointers. */
void pgk_actuator_command(PGK_Actuator *act,
                          float desired_pitch_deg,
                          float desired_yaw_deg,
                          float dt,
                          float *actual_pitch_deg,
                          float *actual_yaw_deg);

/* Write current actuator state to servo PWM outputs.
 * Maps +/-15 deg to servo angles (90 = center).
 * Differential drive: Servo1/3 = pitch pair, Servo2/4 = yaw pair. */
void pgk_actuator_write_pwm(const PGK_Actuator *act);

/* Reset actuator to zero deflection */
void pgk_actuator_reset(PGK_Actuator *act);

/* Get current deflection state */
void pgk_actuator_get_state(const PGK_Actuator *act,
                            float *pitch_deg, float *yaw_deg);

#ifdef __cplusplus
}
#endif

#endif /* PGK_ACTUATOR_H */
