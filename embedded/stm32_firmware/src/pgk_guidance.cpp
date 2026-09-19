#include "pgk_guidance.h"
#include "pgk_config.h"
#include <math.h>

#ifndef M_PI
#define M_PI 3.14159265358979323846f
#endif

void pgk_guidance_init(pgk_guidance_t *guid, float tx, float ty, float tz) {
    guid->phase = PGK_PHASE_BOOST_ASCENT;
    guid->target_x = tx;
    guid->target_y = ty;
    guid->target_z = tz;
    guid->time_launched = 0.0f; // Assumes time starts at 0 or initialized later
}

void pgk_guidance_compute(pgk_guidance_t *guid, float current_time_s, const float state[6], float pitch_cmd_out[1], float yaw_cmd_out[1]) {
    float px = state[0], py = state[1], pz = state[2];
    float vx = state[3], vy = state[4], vz = state[5];
    float alt = pz; // Assuming Z is up
    
    // State machine updates
    if (guid->phase == PGK_PHASE_BOOST_ASCENT && current_time_s >= PGK_GUIDANCE_CANARD_DEPLOY_TIME_S) {
        guid->phase = PGK_PHASE_MIDCOURSE_GUIDANCE;
    }
    if (guid->phase == PGK_PHASE_MIDCOURSE_GUIDANCE && alt < PGK_GUIDANCE_TERMINAL_ALT_M && vz < 0.0f) {
        guid->phase = PGK_PHASE_TERMINAL;
    }
    if (alt <= 0.0f) {
        guid->phase = PGK_PHASE_POST_IMPACT;
    }

    pitch_cmd_out[0] = 0.0f;
    yaw_cmd_out[0] = 0.0f;

    if (guid->phase == PGK_PHASE_BOOST_ASCENT || guid->phase == PGK_PHASE_POST_IMPACT) {
        return; // No guidance
    }

    // Kinematics and simplified TGO
    float V = sqrtf(vx*vx + vy*vy + vz*vz);
    if (V < 1.0f) V = 1.0f;

    float t_go = 0.0f;
    if (vx > 0.0f) {
        t_go = (guid->target_x - px) / vx;
    }
    if (t_go < PGK_GUIDANCE_MIN_TGO_S) t_go = PGK_GUIDANCE_MIN_TGO_S;

    // Pitch: ZEM-based proportional navigation
    float impact_x = px + vx * t_go; 
    float miss_x = guid->target_x - impact_x;
    
    float nav_gain = PGK_GUIDANCE_NAV_GAIN;
    if (guid->phase == PGK_PHASE_TERMINAL) nav_gain *= PGK_GUIDANCE_TERM_GAIN_MULT;

    float a_cmd_x = nav_gain * miss_x / (t_go * t_go);
    
    float rho = 1.225f; // Density
    float q_dyn = 0.5f * rho * V * V;
    float accel_per_deg = (q_dyn * PGK_CANARD_AREA_M2 * PGK_CANARD_CL_DELTA / PGK_SHELL_MASS_KG) * (M_PI / 180.0f);
    
    if (accel_per_deg < 0.001f) accel_per_deg = 0.001f;
    float pitch_cmd = a_cmd_x / accel_per_deg;

    // Yaw: PD controller on crossrange
    float kp = PGK_GUIDANCE_YAW_KP;
    float kd = PGK_GUIDANCE_YAW_KD;
    if (guid->phase == PGK_PHASE_TERMINAL) {
        kp *= PGK_GUIDANCE_TERM_GAIN_MULT;
        kd *= PGK_GUIDANCE_TERM_GAIN_MULT;
    }

    float crossrange_error = guid->target_y - py;
    float vy_required = crossrange_error / t_go;
    float vy_correction = vy_required - vy;
    
    float a_cmd_y = kp * crossrange_error / (t_go * t_go) + kd * vy_correction / t_go;
    float yaw_cmd = -a_cmd_y / accel_per_deg;

    // Ramp guidance
    float ramp = 1.0f;
    float t_active = current_time_s - PGK_GUIDANCE_CANARD_DEPLOY_TIME_S;
    if (t_active < 5.0f && t_active >= 0.0f) {
        ramp = t_active / 5.0f;
    }
    pitch_cmd *= ramp;
    yaw_cmd *= ramp;

    // Clamp
    if (pitch_cmd > PGK_CANARD_MAX_DEFLECTION_DEG) pitch_cmd = PGK_CANARD_MAX_DEFLECTION_DEG;
    if (pitch_cmd < -PGK_CANARD_MAX_DEFLECTION_DEG) pitch_cmd = -PGK_CANARD_MAX_DEFLECTION_DEG;
    if (yaw_cmd > PGK_CANARD_MAX_DEFLECTION_DEG) yaw_cmd = PGK_CANARD_MAX_DEFLECTION_DEG;
    if (yaw_cmd < -PGK_CANARD_MAX_DEFLECTION_DEG) yaw_cmd = -PGK_CANARD_MAX_DEFLECTION_DEG;

    pitch_cmd_out[0] = pitch_cmd;
    yaw_cmd_out[0] = yaw_cmd;
}

pgk_flight_phase_t pgk_guidance_get_phase(const pgk_guidance_t *guid) {
    return guid->phase;
}
