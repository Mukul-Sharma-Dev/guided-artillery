/*
 * pgk_sensors.cpp -- Sensor Driver Implementations
 * ===================================================
 * Contains real sensor read functions (MPU9250, BMP280, NEO-6M)
 * and simulation packet parser for Mode 3 hybrid operation.
 */

#include "pgk_sensors.h"
#include "pgk_config.h"
#include <string.h>
#include <stdlib.h>
#include <math.h>

#ifdef ARDUINO
#include <SPI.h>
#include <Wire.h>
#endif

/* -------------------------------------------------------------------- */
/* MPU9250 Register Definitions                                         */
/* -------------------------------------------------------------------- */
#define MPU9250_ACCEL_XOUT_H  0x3B
#define MPU9250_GYRO_XOUT_H   0x43
#define MPU9250_WHO_AM_I       0x75
#define MPU9250_PWR_MGMT_1     0x6B
#define MPU9250_ACCEL_CONFIG   0x1C
#define MPU9250_GYRO_CONFIG    0x1B

/* Scale factors (16g accel range, 2000 deg/s gyro range) */
#define ACCEL_SCALE  (16.0f * 9.80665f / 32768.0f)   /* LSB -> m/s^2 */
#define GYRO_SCALE   (2000.0f * 3.14159265f / 180.0f / 32768.0f) /* LSB -> rad/s */

/* -------------------------------------------------------------------- */
/* BMP280 Register Definitions                                          */
/* -------------------------------------------------------------------- */
#define BMP280_ADDR           0x76
#define BMP280_REG_ID         0xD0
#define BMP280_REG_CTRL_MEAS  0xF4
#define BMP280_REG_CONFIG     0xF5
#define BMP280_REG_PRESS_MSB  0xF7
#define BMP280_REG_TEMP_MSB   0xFA
#define BMP280_REG_CALIB_START 0x88

/* Calibration data (populated during init) */
static struct {
    uint16_t dig_T1;
    int16_t  dig_T2, dig_T3;
    uint16_t dig_P1;
    int16_t  dig_P2, dig_P3, dig_P4, dig_P5;
    int16_t  dig_P6, dig_P7, dig_P8, dig_P9;
    bool loaded;
} bmp_cal = {0};

/* -------------------------------------------------------------------- */
/* NMEA Parser State                                                    */
/* -------------------------------------------------------------------- */
#define NMEA_BUF_SIZE 128
static char nmea_buf[NMEA_BUF_SIZE];
static int  nmea_idx = 0;
static PGK_GPS_Data last_gps_fix = {0};
static bool new_gps_available = false;

/* -------------------------------------------------------------------- */
/* Initialization                                                       */
/* -------------------------------------------------------------------- */
void pgk_sensors_init(void) {
#ifdef ARDUINO
    /* SPI1 for MPU9250 */
    SPI.begin();
    pinMode(PGK_PIN_IMU_CS, OUTPUT);
    digitalWrite(PGK_PIN_IMU_CS, HIGH);

    /* Wake up MPU9250 */
    digitalWrite(PGK_PIN_IMU_CS, LOW);
    SPI.beginTransaction(SPISettings(1000000, MSBFIRST, SPI_MODE0));
    SPI.transfer(MPU9250_PWR_MGMT_1);  /* Write register */
    SPI.transfer(0x00);                 /* Clear sleep bit */
    SPI.endTransaction();
    digitalWrite(PGK_PIN_IMU_CS, HIGH);
    delay(10);

    /* Set accel range to +/-16g */
    digitalWrite(PGK_PIN_IMU_CS, LOW);
    SPI.beginTransaction(SPISettings(1000000, MSBFIRST, SPI_MODE0));
    SPI.transfer(MPU9250_ACCEL_CONFIG);
    SPI.transfer(0x18);  /* AFS_SEL = 3 (+/-16g) */
    SPI.endTransaction();
    digitalWrite(PGK_PIN_IMU_CS, HIGH);

    /* Set gyro range to +/-2000 deg/s */
    digitalWrite(PGK_PIN_IMU_CS, LOW);
    SPI.beginTransaction(SPISettings(1000000, MSBFIRST, SPI_MODE0));
    SPI.transfer(MPU9250_GYRO_CONFIG);
    SPI.transfer(0x18);  /* FS_SEL = 3 (+/-2000 dps) */
    SPI.endTransaction();
    digitalWrite(PGK_PIN_IMU_CS, HIGH);

    /* I2C1 for BMP280 */
    Wire.begin();

    /* Configure BMP280: oversampling x16 for pressure, x2 for temp, normal mode */
    Wire.beginTransmission(BMP280_ADDR);
    Wire.write(BMP280_REG_CTRL_MEAS);
    Wire.write(0x57);  /* osrs_t=010, osrs_p=101, mode=11 (normal) */
    Wire.endTransmission();

    Wire.beginTransmission(BMP280_ADDR);
    Wire.write(BMP280_REG_CONFIG);
    Wire.write(0x10);  /* t_sb=000, filter=100, spi3w_en=0 */
    Wire.endTransmission();

    /* Read BMP280 calibration data */
    Wire.beginTransmission(BMP280_ADDR);
    Wire.write(BMP280_REG_CALIB_START);
    Wire.endTransmission();
    Wire.requestFrom((int)BMP280_ADDR, 26);
    if (Wire.available() >= 26) {
        uint8_t calib[26];
        for (int i = 0; i < 26; i++) calib[i] = Wire.read();
        bmp_cal.dig_T1 = (uint16_t)(calib[1] << 8 | calib[0]);
        bmp_cal.dig_T2 = (int16_t)(calib[3] << 8 | calib[2]);
        bmp_cal.dig_T3 = (int16_t)(calib[5] << 8 | calib[4]);
        bmp_cal.dig_P1 = (uint16_t)(calib[7] << 8 | calib[6]);
        bmp_cal.dig_P2 = (int16_t)(calib[9] << 8 | calib[8]);
        bmp_cal.dig_P3 = (int16_t)(calib[11] << 8 | calib[10]);
        bmp_cal.dig_P4 = (int16_t)(calib[13] << 8 | calib[12]);
        bmp_cal.dig_P5 = (int16_t)(calib[15] << 8 | calib[14]);
        bmp_cal.dig_P6 = (int16_t)(calib[17] << 8 | calib[16]);
        bmp_cal.dig_P7 = (int16_t)(calib[19] << 8 | calib[18]);
        bmp_cal.dig_P8 = (int16_t)(calib[21] << 8 | calib[20]);
        bmp_cal.dig_P9 = (int16_t)(calib[23] << 8 | calib[22]);
        bmp_cal.loaded = true;
    }

    /* UART2 for GPS (NEO-6M at 9600 baud default) */
    Serial2.begin(9600);
#endif
}

/* -------------------------------------------------------------------- */
/* IMU Read (MPU9250 via SPI)                                           */
/* -------------------------------------------------------------------- */
bool pgk_imu_read(PGK_IMU_Data *data) {
    if (!data) return false;
    data->valid = false;

#ifdef ARDUINO
    uint8_t raw[14]; /* 6 accel + 2 temp + 6 gyro */

    digitalWrite(PGK_PIN_IMU_CS, LOW);
    SPI.beginTransaction(SPISettings(4000000, MSBFIRST, SPI_MODE0));
    SPI.transfer(MPU9250_ACCEL_XOUT_H | 0x80); /* Read flag */
    for (int i = 0; i < 14; i++) {
        raw[i] = SPI.transfer(0x00);
    }
    SPI.endTransaction();
    digitalWrite(PGK_PIN_IMU_CS, HIGH);

    /* Parse accelerometer (big-endian, 16-bit signed) */
    int16_t ax_raw = (int16_t)((raw[0] << 8) | raw[1]);
    int16_t ay_raw = (int16_t)((raw[2] << 8) | raw[3]);
    int16_t az_raw = (int16_t)((raw[4] << 8) | raw[5]);

    /* Parse gyroscope (skip temp bytes 6,7) */
    int16_t gx_raw = (int16_t)((raw[8]  << 8) | raw[9]);
    int16_t gy_raw = (int16_t)((raw[10] << 8) | raw[11]);
    int16_t gz_raw = (int16_t)((raw[12] << 8) | raw[13]);

    /* Convert to engineering units */
    data->accel[0] = ax_raw * ACCEL_SCALE;
    data->accel[1] = ay_raw * ACCEL_SCALE;
    data->accel[2] = az_raw * ACCEL_SCALE;

    data->gyro[0] = gx_raw * GYRO_SCALE;
    data->gyro[1] = gy_raw * GYRO_SCALE;
    data->gyro[2] = gz_raw * GYRO_SCALE;

    data->valid = true;
    data->timestamp_ms = millis();
#endif

    return data->valid;
}

/* -------------------------------------------------------------------- */
/* GPS Read (NEO-6M via UART2, NMEA parsing)                            */
/* -------------------------------------------------------------------- */

/* Parse degrees-minutes to decimal degrees */
static float nmea_to_decimal(const char *field, const char *dir) {
    if (!field || !dir || field[0] == '\0') return 0.0f;
    float raw = atof(field);
    int degrees = (int)(raw / 100.0f);
    float minutes = raw - degrees * 100.0f;
    float decimal = degrees + minutes / 60.0f;
    if (dir[0] == 'S' || dir[0] == 'W') decimal = -decimal;
    return decimal;
}

/* Process a complete NMEA sentence (called when newline is received) */
static void process_nmea(const char *sentence) {
    /* Only process $GPGGA (Global Positioning System Fix Data) */
    if (strncmp(sentence, "$GPGGA", 6) != 0 &&
        strncmp(sentence, "$GNGGA", 6) != 0) return;

    /* Parse fields: $GPGGA,time,lat,N/S,lon,E/W,fix,sats,hdop,alt,M,... */
    char buf[NMEA_BUF_SIZE];
    strncpy(buf, sentence, NMEA_BUF_SIZE - 1);
    buf[NMEA_BUF_SIZE - 1] = '\0';

    char *fields[15] = {0};
    int field_count = 0;
    char *tok = strtok(buf, ",");
    while (tok && field_count < 15) {
        fields[field_count++] = tok;
        tok = strtok(NULL, ",*");
    }

    if (field_count < 10) return;

    /* Check fix quality (field 6, must be >= 1) */
    int fix_quality = atoi(fields[6]);
    if (fix_quality < 1) return;

    float lat = nmea_to_decimal(fields[2], fields[3]);
    float lon = nmea_to_decimal(fields[4], fields[5]);
    float alt = atof(fields[9]);

    /*
     * Convert lat/lon/alt to local ENU (East-North-Up) frame.
     * For the SIH demo, we use a simplified flat-Earth approximation
     * relative to a reference point (launch location).
     * Real system would use WGS84 ECEF -> ENU transform.
     */
    static float ref_lat = 0.0f, ref_lon = 0.0f;
    static bool ref_set = false;

    if (!ref_set && fix_quality >= 1) {
        ref_lat = lat;
        ref_lon = lon;
        ref_set = true;
    }

    /* Approximate meters per degree at reference latitude */
    float cos_lat = cosf(ref_lat * 3.14159265f / 180.0f);
    float m_per_deg_lat = 111132.0f;
    float m_per_deg_lon = 111132.0f * cos_lat;

    last_gps_fix.position[0] = (lon - ref_lon) * m_per_deg_lon;  /* East = X */
    last_gps_fix.position[1] = (lat - ref_lat) * m_per_deg_lat;  /* North = Y */
    last_gps_fix.position[2] = alt;

    /* Velocity not available from GGA; set to zero (use RMC for velocity) */
    last_gps_fix.velocity[0] = 0.0f;
    last_gps_fix.velocity[1] = 0.0f;
    last_gps_fix.velocity[2] = 0.0f;

    last_gps_fix.valid = true;
#ifdef ARDUINO
    last_gps_fix.timestamp_ms = millis();
#endif
    new_gps_available = true;
}

bool pgk_gps_read(PGK_GPS_Data *data) {
    if (!data) return false;
    data->valid = false;

#ifdef ARDUINO
    /* Non-blocking: read available bytes from GPS UART */
    while (Serial2.available() > 0) {
        char c = Serial2.read();
        if (c == '\n' || c == '\r') {
            if (nmea_idx > 0) {
                nmea_buf[nmea_idx] = '\0';
                process_nmea(nmea_buf);
                nmea_idx = 0;
            }
        } else {
            if (nmea_idx < NMEA_BUF_SIZE - 1) {
                nmea_buf[nmea_idx++] = c;
            }
        }
    }
#endif

    if (new_gps_available) {
        *data = last_gps_fix;
        new_gps_available = false;
        return true;
    }
    return false;
}

/* -------------------------------------------------------------------- */
/* Barometer Read (BMP280 via I2C)                                      */
/* -------------------------------------------------------------------- */
bool pgk_baro_read(PGK_Baro_Data *data) {
    if (!data) return false;
    data->valid = false;

#ifdef ARDUINO
    if (!bmp_cal.loaded) return false;

    /* Read raw pressure and temperature (3 bytes each) */
    Wire.beginTransmission(BMP280_ADDR);
    Wire.write(BMP280_REG_PRESS_MSB);
    Wire.endTransmission();
    Wire.requestFrom((int)BMP280_ADDR, 6);

    if (Wire.available() < 6) return false;

    uint8_t raw[6];
    for (int i = 0; i < 6; i++) raw[i] = Wire.read();

    int32_t adc_P = ((int32_t)raw[0] << 12) | ((int32_t)raw[1] << 4) | (raw[2] >> 4);
    int32_t adc_T = ((int32_t)raw[3] << 12) | ((int32_t)raw[4] << 4) | (raw[5] >> 4);

    /* Temperature compensation (from BMP280 datasheet) */
    int32_t var1 = ((((adc_T >> 3) - ((int32_t)bmp_cal.dig_T1 << 1))) *
                    ((int32_t)bmp_cal.dig_T2)) >> 11;
    int32_t var2 = (((((adc_T >> 4) - ((int32_t)bmp_cal.dig_T1)) *
                      ((adc_T >> 4) - ((int32_t)bmp_cal.dig_T1))) >> 12) *
                    ((int32_t)bmp_cal.dig_T3)) >> 14;
    int32_t t_fine = var1 + var2;

    float temp_c = (float)((t_fine * 5 + 128) >> 8) / 100.0f;

    /* Pressure compensation (from BMP280 datasheet) */
    int64_t p_var1 = ((int64_t)t_fine) - 128000;
    int64_t p_var2 = p_var1 * p_var1 * (int64_t)bmp_cal.dig_P6;
    p_var2 = p_var2 + ((p_var1 * (int64_t)bmp_cal.dig_P5) << 17);
    p_var2 = p_var2 + (((int64_t)bmp_cal.dig_P4) << 35);
    p_var1 = ((p_var1 * p_var1 * (int64_t)bmp_cal.dig_P3) >> 8) +
             ((p_var1 * (int64_t)bmp_cal.dig_P2) << 12);
    p_var1 = (((((int64_t)1) << 47) + p_var1)) * ((int64_t)bmp_cal.dig_P1) >> 33;

    if (p_var1 == 0) return false;

    int64_t p = 1048576 - adc_P;
    p = (((p << 31) - p_var2) * 3125) / p_var1;
    p_var1 = (((int64_t)bmp_cal.dig_P9) * (p >> 13) * (p >> 13)) >> 25;
    p_var2 = (((int64_t)bmp_cal.dig_P8) * p) >> 19;
    p = ((p + p_var1 + p_var2) >> 8) + (((int64_t)bmp_cal.dig_P7) << 4);

    float pressure_pa = (float)p / 256.0f;

    /* Convert pressure to altitude using barometric formula
     * h = 44330 * (1 - (P/P0)^0.190284) */
    float altitude = 44330.0f * (1.0f - powf(pressure_pa / 101325.0f, 0.190284f));

    data->altitude_m = altitude;
    data->pressure_pa = pressure_pa;
    data->temperature_c = temp_c;
    data->valid = true;
    data->timestamp_ms = millis();
#endif

    return data->valid;
}

/* -------------------------------------------------------------------- */
/* Simulation Packet Parser                                             */
/* -------------------------------------------------------------------- */
static uint8_t compute_xor_checksum(const char *data, int len) {
    uint8_t chk = 0;
    for (int i = 0; i < len; i++) {
        chk ^= (uint8_t)data[i];
    }
    return chk;
}

bool pgk_sensors_parse_serial(const char *packet,
                              PGK_IMU_Data *imu,
                              PGK_GPS_Data *gps,
                              PGK_Baro_Data *baro,
                              float *sim_time,
                              float *alt_agl) {
    if (!packet || packet[0] != '$') return false;

    /* Find checksum delimiter */
    const char *star = strchr(packet, '*');
    if (!star) return false;

    /* Verify XOR checksum */
    int payload_len = (int)(star - packet - 1); /* Between $ and * */
    uint8_t calc_chk = compute_xor_checksum(packet + 1, payload_len);
    uint8_t recv_chk = (uint8_t)strtol(star + 1, NULL, 16);
    if (calc_chk != recv_chk) return false;

    /* Copy payload for tokenization */
    char buf[256];
    int copy_len = (int)(star - packet - 1);
    if (copy_len >= 255) copy_len = 254;
    memcpy(buf, packet + 1, copy_len);
    buf[copy_len] = '\0';

    /* Tokenize: PGK,time,x,y,z,vx,vy,vz,ax,ay,az,alt */
    char *fields[12] = {0};
    int count = 0;
    char *tok = strtok(buf, ",");
    while (tok && count < 12) {
        fields[count++] = tok;
        tok = strtok(NULL, ",");
    }

    if (count < 12) return false;
    if (strcmp(fields[0], "PGK") != 0) return false;

    float t   = atof(fields[1]);
    float x   = atof(fields[2]);
    float y   = atof(fields[3]);
    float z   = atof(fields[4]);
    float vx  = atof(fields[5]);
    float vy  = atof(fields[6]);
    float vz  = atof(fields[7]);
    float ax  = atof(fields[8]);
    float ay  = atof(fields[9]);
    float az  = atof(fields[10]);
    float alt = atof(fields[11]);

    /* Fill IMU data (acceleration from simulation) */
    if (imu) {
        imu->accel[0] = ax;
        imu->accel[1] = ay;
        imu->accel[2] = az;
        imu->gyro[0] = 0.0f;
        imu->gyro[1] = 0.0f;
        imu->gyro[2] = 0.0f;
        imu->valid = true;
        imu->timestamp_ms = (uint32_t)(t * 1000.0f);
    }

    /* Fill GPS data (position and velocity from simulation) */
    if (gps) {
        gps->position[0] = x;
        gps->position[1] = y;
        gps->position[2] = z;
        gps->velocity[0] = vx;
        gps->velocity[1] = vy;
        gps->velocity[2] = vz;
        gps->valid = true;
        gps->timestamp_ms = (uint32_t)(t * 1000.0f);
    }

    /* Fill baro data (altitude from simulation) */
    if (baro) {
        baro->altitude_m = z;
        baro->pressure_pa = 101325.0f * powf(1.0f - z / 44330.0f, 5.255f);
        baro->temperature_c = 15.0f - 0.0065f * z;
        baro->valid = true;
        baro->timestamp_ms = (uint32_t)(t * 1000.0f);
    }

    if (sim_time) *sim_time = t;
    if (alt_agl)  *alt_agl  = alt;

    return true;
}
