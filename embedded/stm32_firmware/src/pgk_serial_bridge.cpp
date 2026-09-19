/*
 * pgk_serial_bridge.cpp -- Serial Bridge Implementation
 * =======================================================
 */

#include "pgk_serial_bridge.h"
#include <string.h>
#include <stdio.h>

#ifdef ARDUINO
#include <Arduino.h>
#endif

/* -------------------------------------------------------------------- */
/* Checksum                                                             */
/* -------------------------------------------------------------------- */
uint8_t pgk_serial_checksum(const char *data, int len) {
    uint8_t chk = 0;
    for (int i = 0; i < len; i++) {
        chk ^= (uint8_t)data[i];
    }
    return chk;
}

/* -------------------------------------------------------------------- */
/* Initialization                                                       */
/* -------------------------------------------------------------------- */
void pgk_serial_init(PGK_SerialBridge *bridge, uint32_t baud) {
    memset(bridge, 0, sizeof(PGK_SerialBridge));

#ifdef ARDUINO
    Serial.begin(baud);
    while (!Serial && millis() < 3000) { /* Wait up to 3s for USB */ }
#endif
}

/* -------------------------------------------------------------------- */
/* Receive Processing (non-blocking)                                    */
/* -------------------------------------------------------------------- */
bool pgk_serial_process_rx(PGK_SerialBridge *bridge) {
    bridge->packet_ready = false;

#ifdef ARDUINO
    while (Serial.available() > 0) {
        char c = Serial.read();

        if (c == '\n' || c == '\r') {
            if (bridge->rx_idx > 0) {
                bridge->rx_buf[bridge->rx_idx] = '\0';

                /* Validate: must start with $ and contain * */
                if (bridge->rx_buf[0] == '$' &&
                    strchr(bridge->rx_buf, '*') != NULL) {
                    bridge->packet_ready = true;
                    bridge->rx_count++;
                } else {
                    bridge->rx_errors++;
                }
                bridge->rx_idx = 0;
                return bridge->packet_ready;
            }
        } else {
            if (bridge->rx_idx < PGK_SERIAL_BUF_SIZE - 1) {
                bridge->rx_buf[bridge->rx_idx++] = c;
            } else {
                /* Buffer overflow -- reset */
                bridge->rx_idx = 0;
                bridge->rx_errors++;
            }
        }
    }
#endif

    return false;
}

const char* pgk_serial_get_packet(const PGK_SerialBridge *bridge) {
    return bridge->rx_buf;
}

/* -------------------------------------------------------------------- */
/* Transmit Telemetry                                                   */
/* -------------------------------------------------------------------- */
void pgk_serial_send_telemetry(PGK_SerialBridge *bridge,
                               float t,
                               const char *state_str,
                               const char *phase_str,
                               const float *ekf_state,
                               float pitch_cmd,
                               float yaw_cmd,
                               float miss_x,
                               float miss_y,
                               const char *fuze_str) {
#ifdef ARDUINO
    /* Build payload */
    char payload[PGK_SERIAL_BUF_SIZE];
    snprintf(payload, sizeof(payload),
             "TLM,%.3f,%s,%s,"
             "%.2f,%.2f,%.2f,"
             "%.2f,%.2f,%.2f,"
             "%.2f,%.2f,"
             "%.2f,%.2f,%s",
             t,
             state_str ? state_str : "UNKNOWN",
             phase_str ? phase_str : "UNKNOWN",
             ekf_state[0], ekf_state[1], ekf_state[2],
             ekf_state[3], ekf_state[4], ekf_state[5],
             pitch_cmd, yaw_cmd,
             miss_x, miss_y,
             fuze_str ? fuze_str : "UNKNOWN");

    /* Compute checksum */
    int plen = strlen(payload);
    uint8_t chk = pgk_serial_checksum(payload, plen);

    /* Send formatted packet */
    char packet[PGK_SERIAL_BUF_SIZE];
    snprintf(packet, sizeof(packet), "$%s*%02X\n", payload, chk);
    Serial.print(packet);

    bridge->tx_count++;
#else
    (void)bridge; (void)t; (void)state_str; (void)phase_str;
    (void)ekf_state; (void)pitch_cmd; (void)yaw_cmd;
    (void)miss_x; (void)miss_y; (void)fuze_str;
#endif
}
