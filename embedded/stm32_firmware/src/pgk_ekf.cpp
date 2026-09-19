#include "pgk_ekf.h"
#include <string.h>
#include <math.h>

// Helper functions for 6x6 matrix math
static void mat_mul_6x6(const float A[6][6], const float B[6][6], float C[6][6]) {
    for (int i = 0; i < 6; i++) {
        for (int j = 0; j < 6; j++) {
            C[i][j] = 0.0f;
            for (int k = 0; k < 6; k++) {
                C[i][j] += A[i][k] * B[k][j];
            }
        }
    }
}

static void mat_transpose_6x6(const float A[6][6], float AT[6][6]) {
    for (int i = 0; i < 6; i++) {
        for (int j = 0; j < 6; j++) {
            AT[j][i] = A[i][j];
        }
    }
}

// In-place matrix inversion using Gauss-Jordan elimination
static void mat_inv_6x6(float A[6][6]) {
    float inv[6][6];
    for (int i = 0; i < 6; i++) {
        for (int j = 0; j < 6; j++) {
            inv[i][j] = (i == j) ? 1.0f : 0.0f;
        }
    }

    for (int i = 0; i < 6; i++) {
        float pivot = A[i][i];
        if (fabsf(pivot) < 1e-6f) pivot = 1e-6f; // Avoid division by zero
        for (int j = 0; j < 6; j++) {
            A[i][j] /= pivot;
            inv[i][j] /= pivot;
        }
        for (int k = 0; k < 6; k++) {
            if (k != i) {
                float factor = A[k][i];
                for (int j = 0; j < 6; j++) {
                    A[k][j] -= factor * A[i][j];
                    inv[k][j] -= factor * inv[i][j];
                }
            }
        }
    }

    for (int i = 0; i < 6; i++) {
        for (int j = 0; j < 6; j++) {
            A[i][j] = inv[i][j];
        }
    }
}

void pgk_ekf_init(pgk_ekf_t *ekf, float process_noise_accel, float gps_pos_noise, float gps_vel_noise, float baro_noise, float gate_threshold) {
    memset(ekf->x, 0, sizeof(ekf->x));
    memset(ekf->P, 0, sizeof(ekf->P));
    for (int i = 0; i < 3; i++) {
        ekf->P[i][i] = 100.0f;
        ekf->P[i+3][i+3] = 50.0f;
    }
    
    ekf->q_accel = process_noise_accel;
    
    memset(ekf->R_gps, 0, sizeof(ekf->R_gps));
    for (int i = 0; i < 3; i++) {
        ekf->R_gps[i][i] = gps_pos_noise * gps_pos_noise;
        ekf->R_gps[i+3][i+3] = gps_vel_noise * gps_vel_noise;
    }
    
    ekf->R_baro[0][0] = baro_noise * baro_noise;
    ekf->gate = gate_threshold;
}

void pgk_ekf_predict(pgk_ekf_t *ekf, const float accel[3], float dt) {
    float x_new[6];
    for (int i = 0; i < 6; i++) x_new[i] = ekf->x[i];
    
    x_new[0] += ekf->x[3]*dt + 0.5f*accel[0]*dt*dt;
    x_new[1] += ekf->x[4]*dt + 0.5f*accel[1]*dt*dt;
    x_new[2] += ekf->x[5]*dt + 0.5f*accel[2]*dt*dt;
    x_new[3] += accel[0]*dt;
    x_new[4] += accel[1]*dt;
    x_new[5] += accel[2]*dt;
    
    for (int i = 0; i < 6; i++) ekf->x[i] = x_new[i];
    
    float F[6][6] = {0};
    for (int i = 0; i < 6; i++) F[i][i] = 1.0f;
    F[0][3] = dt; F[1][4] = dt; F[2][5] = dt;
    
    float Q[6][6] = {0};
    float q2 = ekf->q_accel * ekf->q_accel;
    for (int i = 0; i < 3; i++) {
        Q[i][i] = q2 * dt*dt*dt / 3.0f;
        Q[i][i+3] = q2 * dt*dt / 2.0f;
        Q[i+3][i] = q2 * dt*dt / 2.0f;
        Q[i+3][i+3] = q2 * dt;
    }
    
    float FT[6][6];
    mat_transpose_6x6(F, FT);
    
    float PF_T[6][6];
    mat_mul_6x6(ekf->P, FT, PF_T);
    
    float FPF_T[6][6];
    mat_mul_6x6(F, PF_T, FPF_T);
    
    for (int i = 0; i < 6; i++) {
        for (int j = 0; j < 6; j++) {
            ekf->P[i][j] = FPF_T[i][j] + Q[i][j];
        }
    }
}

void pgk_ekf_update_gps(pgk_ekf_t *ekf, const float gps_pos[3], const float gps_vel[3]) {
    float H[6][6] = {0};
    for(int i = 0; i < 6; i++) H[i][i] = 1.0f;
    
    float z[6] = {gps_pos[0], gps_pos[1], gps_pos[2], gps_vel[0], gps_vel[1], gps_vel[2]};
    float innovation[6];
    for (int i = 0; i < 6; i++) {
        innovation[i] = z[i] - ekf->x[i];
    }
    
    float S[6][6];
    for (int i = 0; i < 6; i++) {
        for (int j = 0; j < 6; j++) {
            S[i][j] = ekf->P[i][j] + ekf->R_gps[i][j]; // P + R since H = I
        }
    }
    
    mat_inv_6x6(S);
    
    float K[6][6];
    mat_mul_6x6(ekf->P, S, K); // P * inv(S) since H = I
    
    for (int i = 0; i < 6; i++) {
        float update = 0.0f;
        for (int j = 0; j < 6; j++) {
            update += K[i][j] * innovation[j];
        }
        ekf->x[i] += update;
    }
    
    float I_KH[6][6] = {0};
    for(int i = 0; i < 6; i++) I_KH[i][i] = 1.0f;
    for(int i = 0; i < 6; i++) {
        for(int j = 0; j < 6; j++) {
            I_KH[i][j] -= K[i][j]; // K*H since H=I
        }
    }
    
    float I_KH_T[6][6];
    mat_transpose_6x6(I_KH, I_KH_T);
    
    float temp1[6][6], temp2[6][6];
    mat_mul_6x6(I_KH, ekf->P, temp1);
    mat_mul_6x6(temp1, I_KH_T, temp2);
    
    float KT[6][6], KR[6][6], KRK[6][6];
    mat_transpose_6x6(K, KT);
    mat_mul_6x6(K, ekf->R_gps, KR);
    mat_mul_6x6(KR, KT, KRK);
    
    for (int i = 0; i < 6; i++) {
        for (int j = 0; j < 6; j++) {
            ekf->P[i][j] = temp2[i][j] + KRK[i][j];
        }
    }
}

void pgk_ekf_update_baro(pgk_ekf_t *ekf, float baro_alt) {
    float innovation = baro_alt - ekf->x[2];
    
    float S = ekf->P[2][2] + ekf->R_baro[0][0];
    float S_inv = 1.0f / S;
    
    float K[6];
    for (int i = 0; i < 6; i++) {
        K[i] = ekf->P[i][2] * S_inv;
    }
    
    for (int i = 0; i < 6; i++) {
        ekf->x[i] += K[i] * innovation;
    }
    
    float I_KH[6][6];
    for (int i = 0; i < 6; i++) {
        for (int j = 0; j < 6; j++) {
            I_KH[i][j] = (i == j ? 1.0f : 0.0f);
            if (j == 2) I_KH[i][j] -= K[i];
        }
    }
    
    float I_KH_T[6][6];
    mat_transpose_6x6(I_KH, I_KH_T);
    
    float temp1[6][6], temp2[6][6];
    mat_mul_6x6(I_KH, ekf->P, temp1);
    mat_mul_6x6(temp1, I_KH_T, temp2);
    
    for (int i = 0; i < 6; i++) {
        for (int j = 0; j < 6; j++) {
            ekf->P[i][j] = temp2[i][j] + K[i] * ekf->R_baro[0][0] * K[j];
        }
    }
}

void pgk_ekf_get_state(const pgk_ekf_t *ekf, float state_out[6]) {
    for (int i = 0; i < 6; i++) {
        state_out[i] = ekf->x[i];
    }
}

void pgk_ekf_get_state_and_cov(const pgk_ekf_t *ekf, float state_out[6], float cov_diag_out[6]) {
    for (int i = 0; i < 6; i++) {
        if (state_out) state_out[i] = ekf->x[i];
        if (cov_diag_out) cov_diag_out[i] = ekf->P[i][i];
    }
}

void pgk_ekf_init_default(pgk_ekf_t *ekf, const float init_pos[3], const float init_vel[3]) {
    pgk_ekf_init(ekf, 10.0f, 2.0f, 0.1f, 3.0f, 5.0f);
    pgk_ekf_reset_state(ekf, init_pos, init_vel);
}

void pgk_ekf_reset_state(pgk_ekf_t *ekf, const float init_pos[3], const float init_vel[3]) {
    if (init_pos) {
        ekf->x[0] = init_pos[0];
        ekf->x[1] = init_pos[1];
        ekf->x[2] = init_pos[2];
    }
    if (init_vel) {
        ekf->x[3] = init_vel[0];
        ekf->x[4] = init_vel[1];
        ekf->x[5] = init_vel[2];
    }
    for (int i = 0; i < 6; i++) {
        for (int j = 0; j < 6; j++) {
            ekf->P[i][j] = (i == j) ? 1.0f : 0.0f;
        }
    }
}

