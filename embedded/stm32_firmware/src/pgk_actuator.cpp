/*
 * pgk_actuator.cpp -- Canard Actuator Model Implementation
 * ==========================================================
 * Port of simulation/actuator_model.py CanardActuator class.
 */

#include "pgk_actuator.h"
#include "pgk_config.h"
#include <math.h>

/* Arduino/STM32 Servo library */
#ifdef ARDUINO
#include <Servo.h>
static Servo servo_pitch_pos;   /* PE9  / Pin 9  -- Pitch + */
static Servo servo_yaw_pos;     /* PE11 / Pin 10 -- Yaw +   */
static Servo servo_pitch_neg;   /* PE13 / Pin 11 -- Pitch - */
static Servo servo_yaw_neg;     /* PE14 / Pin 12 -- Yaw -   */
static bool servos_attached = false;
#endif

/* Clamp value to [lo, hi] */
static float clampf(float val, float lo, float hi) {
    if (val < lo) return lo;
    if (val > hi) return hi;
    return val;
}

void pgk_actuator_init(PGK_Actuator *act) {
    act->pitch_deg = 0.0f;
    act->yaw_deg   = 0.0f;
    act->max_defl  = PGK_CANARD_MAX_DEFL_DEG;
    act->slew_rate = PGK_CANARD_SLEW_RATE_DEG_S;
    act->tau       = PGK_CANARD_LAG_TAU_S;

#ifdef ARDUINO
    if (!servos_attached) {
        servo_pitch_pos.attach(PGK_PIN_SERVO1);
        servo_yaw_pos.attach(PGK_PIN_SERVO2);
        servo_pitch_neg.attach(PGK_PIN_SERVO3);
        servo_yaw_neg.attach(PGK_PIN_SERVO4);
        servos_attached = true;
    }
    /* Center all servos */
    servo_pitch_pos.write(90);
    servo_yaw_pos.write(90);
    servo_pitch_neg.write(90);
    servo_yaw_neg.write(90);
#endif
}

void pgk_actuator_command(PGK_Actuator *act,
                          float desired_pitch_deg,
                          float desired_yaw_deg,
                          float dt,
                          float *actual_pitch_deg,
                          float *actual_yaw_deg) {
    /* Step 1: Clamp command to physical limits */
    float cmd_p = clampf(desired_pitch_deg, -act->max_defl, act->max_defl);
    float cmd_y = clampf(desired_yaw_deg,   -act->max_defl, act->max_defl);

    /* Step 2: Rate limiting (slew rate) */
    float max_delta = act->slew_rate * dt;

    float delta_p = clampf(cmd_p - act->pitch_deg, -max_delta, max_delta);
    float delta_y = clampf(cmd_y - act->yaw_deg,   -max_delta, max_delta);

    /* Step 3: First-order lag: x_new = x + delta * alpha */
    float alpha = dt / act->tau;
    if (alpha > 1.0f) alpha = 1.0f;

    act->pitch_deg += delta_p * alpha;
    act->yaw_deg   += delta_y * alpha;

    /* Step 4: Final saturation guard */
    act->pitch_deg = clampf(act->pitch_deg, -act->max_defl, act->max_defl);
    act->yaw_deg   = clampf(act->yaw_deg,   -act->max_defl, act->max_defl);

    /* Output */
    if (actual_pitch_deg) *actual_pitch_deg = act->pitch_deg;
    if (actual_yaw_deg)   *actual_yaw_deg   = act->yaw_deg;
}

void pgk_actuator_write_pwm(const PGK_Actuator *act) {
#ifdef ARDUINO
    /*
     * Map canard deflection (+/-15 deg) to servo angle (0-180 deg).
     * Center = 90 deg. Scale by 3x for visual amplification in demo.
     *
     * Pitch pair: Servo1 and Servo3 move in opposite directions.
     * Yaw pair:   Servo2 and Servo4 move in opposite directions.
     */
    int pitch_servo = 90 + (int)(act->pitch_deg * 3.0f);
    int yaw_servo   = 90 + (int)(act->yaw_deg   * 3.0f);

    /* Clamp to servo range */
    if (pitch_servo < 0)   pitch_servo = 0;
    if (pitch_servo > 180) pitch_servo = 180;
    if (yaw_servo < 0)     yaw_servo = 0;
    if (yaw_servo > 180)   yaw_servo = 180;

    /* Differential drive (opposite fins deflect oppositely) */
    servo_pitch_pos.write(pitch_servo);
    servo_pitch_neg.write(180 - pitch_servo);

    servo_yaw_pos.write(yaw_servo);
    servo_yaw_neg.write(180 - yaw_servo);
#endif
}

void pgk_actuator_reset(PGK_Actuator *act) {
    act->pitch_deg = 0.0f;
    act->yaw_deg   = 0.0f;

#ifdef ARDUINO
    servo_pitch_pos.write(90);
    servo_yaw_pos.write(90);
    servo_pitch_neg.write(90);
    servo_yaw_neg.write(90);
#endif
}

void pgk_actuator_get_state(const PGK_Actuator *act,
                            float *pitch_deg, float *yaw_deg) {
    if (pitch_deg) *pitch_deg = act->pitch_deg;
    if (yaw_deg)   *yaw_deg   = act->yaw_deg;
}
