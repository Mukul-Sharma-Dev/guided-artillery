/*
 * pgk_serial_bridge.h -- Bidirectional Serial Communication
 * ===========================================================
 * Handles the USB serial link between the STM32 flight computer
 * and the laptop running the Streamlit dashboard.
 */

#ifndef PGK_SERIAL_BRIDGE_H
#define PGK_SERIAL_BRIDGE_H

#include <stdint.h>
#include <stdbool.h>

#ifdef __cplusplus
extern "C" {
#endif

/* Maximum packet size */
#define PGK_SERIAL_BUF_SIZE 256

/* Serial bridge state */
typedef struct {
    char rx_buf[PGK_SERIAL_BUF_SIZE];
    int  rx_idx;
    bool packet_ready;
    uint32_t tx_count;
    uint32_t rx_count;
    uint32_t rx_errors;
} PGK_SerialBridge;

/* Initialize serial communication at given baud rate */
void pgk_serial_init(PGK_SerialBridge *bridge, uint32_t baud);

/* Non-blocking: check for and parse incoming packets.
 * Call this every loop iteration.
 * Returns true if a complete packet is available in bridge->rx_buf. */
bool pgk_serial_process_rx(PGK_SerialBridge *bridge);

/* Get the last received packet (null-terminated string).
 * Only valid after pgk_serial_process_rx returns true. */
const char* pgk_serial_get_packet(const PGK_SerialBridge *bridge);

/* Send telemetry packet to laptop.
 * Format: $TLM,time,state,phase,ekf_x,ekf_y,ekf_z,
 *          ekf_vx,ekf_vy,ekf_vz,pitch_cmd,yaw_cmd,
 *          miss_x,miss_y,fuze_state*XX
 */
void pgk_serial_send_telemetry(PGK_SerialBridge *bridge,
                               float t,
                               const char *state_str,
                               const char *phase_str,
                               const float *ekf_state,
                               float pitch_cmd,
                               float yaw_cmd,
                               float miss_x,
                               float miss_y,
                               const char *fuze_str);

/* Compute XOR checksum of a data string */
uint8_t pgk_serial_checksum(const char *data, int len);

#ifdef __cplusplus
}
#endif

#endif /* PGK_SERIAL_BRIDGE_H */
