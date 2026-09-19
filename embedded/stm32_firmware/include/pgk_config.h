#ifndef PGK_CONFIG_H
#define PGK_CONFIG_H

#ifdef ARDUINO
#include <Arduino.h>
#endif

/* Simulation parameters matching mission_config.yaml */

// Shell
#define PGK_SHELL_MASS_KG 43.2f
#define PGK_SHELL_REF_AREA_M2 0.018869f

// Target coordinates (m)
#define PGK_TARGET_X 24000.0f
#define PGK_TARGET_Y 0.0f
#define PGK_TARGET_Z 0.0f

// Canard
#define PGK_CANARD_AREA_M2 0.004f
#define PGK_CANARD_CL_DELTA 3.5f
#define PGK_CANARD_MAX_DEFLECTION_DEG 15.0f
#define PGK_CANARD_MAX_DEFL_DEG 15.0f
#define PGK_CANARD_SLEW_RATE_DEG_S 300.0f
#define PGK_CANARD_LAG_TAU_S 0.02f

// Rates & Timings
#define PGK_RATE_IMU_HZ 100
#define PGK_RATE_GPS_HZ 10
#define PGK_RATE_BARO_HZ 20
#define PGK_CONTROL_DT_S 0.01f
#define PGK_LOOP_PERIOD_MS 10

// Sensor noise defaults for EKF
#define PGK_EKF_Q_ACCEL 10.0f
#define PGK_EKF_GPS_POS_SIGMA 2.0f
#define PGK_EKF_GPS_VEL_SIGMA 0.1f
#define PGK_EKF_BARO_SIGMA 3.0f
#define PGK_EKF_GATE_THRESH 5.0f

// Guidance
#define PGK_GUIDANCE_NAV_GAIN 6.0f
#define PGK_GUIDANCE_TERM_GAIN_MULT 2.0f
#define PGK_GUIDANCE_YAW_KP 3.0f
#define PGK_GUIDANCE_YAW_KD 2.0f
#define PGK_GUIDANCE_CANARD_DEPLOY_TIME_S 2.0f
#define PGK_GUIDANCE_MIN_TGO_S 0.5f
#define PGK_GUIDANCE_TERMINAL_ALT_M 500.0f

// Fuze
#define PGK_FUZE_ARM_DISTANCE_M 500.0f
#define PGK_FUZE_ARM_TIME_S 5.0f
#define PGK_FUZE_SETBACK_THRESHOLD_G 10000.0f
#define PGK_FUZE_PROXIMITY_HOB_M 7.0f
#define PGK_FUZE_IMPACT_DECEL_G 500.0f

/* Hardware Pin Definitions (STM32 Nucleo-H743ZI2 pin numbers / identifiers) */

// SPI1 (IMU - MPU9250 / ADIS16490)
#ifndef PGK_PIN_IMU_CS
#if defined(PA4)
#define PGK_PIN_IMU_CS   PA4
#else
#define PGK_PIN_IMU_CS   4
#endif
#endif

// PWM TIM1 (Canard Servos 1 to 4)
#ifndef PGK_PIN_SERVO1
#if defined(PE9)
#define PGK_PIN_SERVO1   PE9
#define PGK_PIN_SERVO2   PE11
#define PGK_PIN_SERVO3   PE13
#define PGK_PIN_SERVO4   PE14
#else
#define PGK_PIN_SERVO1   9
#define PGK_PIN_SERVO2   10
#define PGK_PIN_SERVO3   11
#define PGK_PIN_SERVO4   12
#endif
#endif

// Aliases for PWM pins
#define PGK_PIN_PWM_CH1_PITCH_POS PGK_PIN_SERVO1
#define PGK_PIN_PWM_CH2_YAW_POS   PGK_PIN_SERVO2
#define PGK_PIN_PWM_CH3_PITCH_NEG PGK_PIN_SERVO3
#define PGK_PIN_PWM_CH4_YAW_NEG   PGK_PIN_SERVO4

// GPIO (Status LEDs)
#ifndef PGK_PIN_LED_R
#if defined(PB0)
#define PGK_PIN_LED_R     PB0
#define PGK_PIN_LED_G     PB1
#define PGK_PIN_LED_B     PB2
#define PGK_PIN_LED_ARMED PB5
#else
#define PGK_PIN_LED_R     5
#define PGK_PIN_LED_G     6
#define PGK_PIN_LED_B     7
#define PGK_PIN_LED_ARMED 13
#endif
#endif

/* Serial Protocol */
#define PGK_SERIAL_PROTOCOL_FORMAT "$PGK,time,x,y,z,vx,vy,vz,ax,ay,az,alt*checksum"
#define PGK_SERIAL_BAUD 115200
#define PGK_SERIAL_BAUD_RATE 115200

#endif // PGK_CONFIG_H
