#ifndef PGK_EKF_H
#define PGK_EKF_H

#ifdef __cplusplus
extern "C" {
#endif

typedef struct {
    float x[6]; // [x, y, z, vx, vy, vz]
    float P[6][6];
    float q_accel;
    float R_gps[6][6];
    float R_baro[1][1];
    float gate;
} pgk_ekf_t;

typedef pgk_ekf_t PGK_EKF;

void pgk_ekf_init(pgk_ekf_t *ekf, float process_noise_accel, float gps_pos_noise, float gps_vel_noise, float baro_noise, float gate_threshold);
void pgk_ekf_init_default(pgk_ekf_t *ekf, const float init_pos[3], const float init_vel[3]);
void pgk_ekf_reset_state(pgk_ekf_t *ekf, const float init_pos[3], const float init_vel[3]);
void pgk_ekf_predict(pgk_ekf_t *ekf, const float accel[3], float dt);
void pgk_ekf_update_gps(pgk_ekf_t *ekf, const float gps_pos[3], const float gps_vel[3]);
void pgk_ekf_update_baro(pgk_ekf_t *ekf, float baro_alt);
void pgk_ekf_get_state(const pgk_ekf_t *ekf, float state_out[6]);
void pgk_ekf_get_state_and_cov(const pgk_ekf_t *ekf, float state_out[6], float cov_diag_out[6]);

#ifdef __cplusplus
}
#endif

#endif // PGK_EKF_H
