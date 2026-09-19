#include "pgk_fuze.h"
#include "pgk_config.h"
#include <math.h>

void pgk_fuze_init(pgk_fuze_t *fuze) {
    fuze->state = PGK_FUZE_STATE_SAFE;
    fuze->mode = PGK_FUZE_MODE_IMPACT;
    fuze->start_time = -1.0f;
    fuze->start_x = 0.0f;
    fuze->start_y = 0.0f;
}

void pgk_fuze_init_with_mode(pgk_fuze_t *fuze, pgk_fuze_mode_t mode) {
    pgk_fuze_init(fuze);
    fuze->mode = mode;
}

void pgk_fuze_set_mode(pgk_fuze_t *fuze, pgk_fuze_mode_t mode) {
    fuze->mode = mode;
}

pgk_fuze_state_t pgk_fuze_get_state(const pgk_fuze_t *fuze) {
    return fuze->state;
}

void pgk_fuze_update(pgk_fuze_t *fuze, float current_time_s, const float state[6], float accel_g, float distance_to_ground) {
    float x = state[0];
    float y = state[1];
    float vz = state[5];

    switch (fuze->state) {
        case PGK_FUZE_STATE_SAFE:
            if (accel_g >= PGK_FUZE_SETBACK_THRESHOLD_G) {
                fuze->state = PGK_FUZE_STATE_ARMING;
                fuze->start_time = current_time_s;
                fuze->start_x = x;
                fuze->start_y = y;
            }
            break;
            
        case PGK_FUZE_STATE_ARMING: {
            float dx = x - fuze->start_x;
            float dy = y - fuze->start_y;
            float downrange = sqrtf(dx*dx + dy*dy);
            float flight_time = current_time_s - fuze->start_time;
            
            if (downrange >= PGK_FUZE_ARM_DISTANCE_M && flight_time >= PGK_FUZE_ARM_TIME_S) {
                fuze->state = PGK_FUZE_STATE_ARMED;
            }
            break;
        }
            
        case PGK_FUZE_STATE_ARMED:
            fuze->state = PGK_FUZE_STATE_ACTIVE;
            break;
            
        case PGK_FUZE_STATE_ACTIVE:
            if (fuze->mode == PGK_FUZE_MODE_PROXIMITY) {
                if (distance_to_ground <= PGK_FUZE_PROXIMITY_HOB_M && vz < 0.0f) {
                    fuze->state = PGK_FUZE_STATE_DETONATED;
                }
            } else { // IMPACT
                if (accel_g >= PGK_FUZE_IMPACT_DECEL_G) {
                    fuze->state = PGK_FUZE_STATE_DETONATED;
                }
            }
            break;
            
        case PGK_FUZE_STATE_DETONATED:
        case PGK_FUZE_STATE_DUDE:
        default:
            break;
    }
}
