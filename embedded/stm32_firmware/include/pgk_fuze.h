#ifndef PGK_FUZE_H
#define PGK_FUZE_H

#ifdef __cplusplus
extern "C" {
#endif

typedef enum {
    PGK_FUZE_STATE_SAFE = 0,
    PGK_FUZE_STATE_ARMING = 1,
    PGK_FUZE_STATE_ARMED = 2,
    PGK_FUZE_STATE_ACTIVE = 3,
    PGK_FUZE_STATE_DETONATED = 4,
    PGK_FUZE_STATE_DUDE = 5,
    // Short aliases
    PGK_FUZE_SAFE = 0,
    PGK_FUZE_ARMING = 1,
    PGK_FUZE_ARMED = 2,
    PGK_FUZE_ACTIVE = 3,
    PGK_FUZE_DETONATED = 4,
    PGK_FUZE_DUDE = 5
} pgk_fuze_state_t;

typedef pgk_fuze_state_t PGK_FuzeState;

typedef enum {
    PGK_FUZE_MODE_PROXIMITY = 0,
    PGK_FUZE_MODE_IMPACT = 1
} pgk_fuze_mode_t;

typedef pgk_fuze_mode_t PGK_FuzeMode;

typedef struct {
    pgk_fuze_state_t state;
    pgk_fuze_mode_t mode;
    float start_time;
    float start_x;
    float start_y;
} pgk_fuze_t;

typedef pgk_fuze_t PGK_Fuze;

void pgk_fuze_init(pgk_fuze_t *fuze);
void pgk_fuze_init_with_mode(pgk_fuze_t *fuze, pgk_fuze_mode_t mode);
void pgk_fuze_update(pgk_fuze_t *fuze, float current_time_s, const float state[6], float accel_g, float distance_to_ground);
pgk_fuze_state_t pgk_fuze_get_state(const pgk_fuze_t *fuze);
void pgk_fuze_set_mode(pgk_fuze_t *fuze, pgk_fuze_mode_t mode);

#ifdef __cplusplus
}
#endif

#endif // PGK_FUZE_H
