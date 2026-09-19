/*
 * pgk_sensors.h -- Sensor Driver Abstraction Layer
 * ===================================================
 * Provides a unified interface for reading IMU (MPU9250),
 * GPS (NEO-6M), and Barometer (BMP280) sensors, plus
 * parsing simulation data from the serial bridge.
 *
 * Mode 3 (Hybrid): Real sensors + simulation data coexist.
 * The main loop selects the best available data source.
 */

#ifndef PGK_SENSORS_H
#define PGK_SENSORS_H

#include <stdint.h>
#include <stdbool.h>

#ifdef __cplusplus
extern "C" {
#endif

/* --- Data Structures --- */

typedef struct {
    float accel[3];          /* Acceleration [m/s^2] (x, y, z) */
    float gyro[3];           /* Angular rate [rad/s] (p, q, r) */
    bool valid;
    uint32_t timestamp_ms;
} PGK_IMU_Data;

typedef struct {
    float position[3];       /* Position [m] (x, y, z) */
    float velocity[3];       /* Velocity [m/s] (vx, vy, vz) */
    bool valid;
    uint32_t timestamp_ms;
} PGK_GPS_Data;

typedef struct {
    float altitude_m;        /* Altitude [m] */
    float pressure_pa;       /* Raw pressure [Pa] */
    float temperature_c;     /* Temperature [C] */
    bool valid;
    uint32_t timestamp_ms;
} PGK_Baro_Data;

/* Combined sensor snapshot */
typedef struct {
    PGK_IMU_Data imu;
    PGK_GPS_Data gps;
    PGK_Baro_Data baro;
    float sim_time;          /* Simulation time from serial bridge [s] */
    float sim_alt_agl;       /* AGL altitude from simulation [m] */
    bool from_simulation;    /* True if data came from serial bridge */
} PGK_Sensor_Snapshot;

/* --- Initialization --- */

/* Initialize all sensor buses (SPI1, I2C1, UART2) */
void pgk_sensors_init(void);

/* --- Real Sensor Reads --- */

/* Read MPU9250 IMU via SPI1.
 * Returns true if valid data was read. */
bool pgk_imu_read(PGK_IMU_Data *data);

/* Read NEO-6M GPS via UART2.
 * Returns true if a new valid fix is available (non-blocking). */
bool pgk_gps_read(PGK_GPS_Data *data);

/* Read BMP280 barometer via I2C1.
 * Returns true if valid data was read. */
bool pgk_baro_read(PGK_Baro_Data *data);

/* --- Simulation Data Parsing --- */

/* Parse a $PGK serial packet from the Python simulation.
 * Format: $PGK,time,x,y,z,vx,vy,vz,ax,ay,az,alt*XX
 * Returns true if parsing succeeded and checksum matched. */
bool pgk_sensors_parse_serial(const char *packet,
                              PGK_IMU_Data *imu,
                              PGK_GPS_Data *gps,
                              PGK_Baro_Data *baro,
                              float *sim_time,
                              float *alt_agl);

#ifdef __cplusplus
}
#endif

#endif /* PGK_SENSORS_H */
