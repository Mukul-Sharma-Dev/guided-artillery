#ifndef PGK_GUIDANCE_H
#define PGK_GUIDANCE_H

#ifdef __cplusplus
extern "C" {
#endif

typedef enum {
    PGK_PHASE_BOOST_ASCENT = 0,
    PGK_PHASE_MIDCOURSE_GUIDANCE = 1,
    PGK_PHASE_TERMINAL = 2,
    PGK_PHASE_POST_IMPACT = 3,
    // Short aliases
    PGK_PHASE_BOOST = 0,
    PGK_PHASE_MIDCOURSE = 1,
    PGK_PHASE_POST = 3
} pgk_flight_phase_t;

typedef pgk_flight_phase_t PGK_FlightPhase;

typedef struct {
    pgk_flight_phase_t phase;
    float target_x;
    float target_y;
    float target_z;
    float time_launched;
} pgk_guidance_t;

typedef pgk_guidance_t PGK_Guidance;

void pgk_guidance_init(pgk_guidance_t *guid, float tx, float ty, float tz);
void pgk_guidance_compute(pgk_guidance_t *guid, float current_time_s, const float state[6], float *pitch_cmd_out, float *yaw_cmd_out);
pgk_flight_phase_t pgk_guidance_get_phase(const pgk_guidance_t *guid);

#ifdef __cplusplus
}
#endif

#endif // PGK_GUIDANCE_H
