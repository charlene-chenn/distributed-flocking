#ifndef RADIO_H
#define RADIO_H

#include <stdint.h>
#include <stddef.h>
#include "kinematics.h"
#include "sntp_time.h"

#ifdef __cplusplus
extern "C" {
#endif

// Forward declarations for C++ types
#ifdef __cplusplus
class SX1276;
class EspHal;
#else
typedef struct SX1276 SX1276;
typedef struct EspHal EspHal;
#endif 

// 3. Radio I/O task
// Radio packet format (COMP0221 standard)
// This goes over LoRa. Units are fixed to match the spec:
//  - position in millimetres
//  - velocity in millimetres per second
//  - yaw in centi-degrees (deg * 100)
//  - timestamps are Unix seconds + millisecond part
//
// IMPORTANT: __attribute__((packed)) to avoid padding that breaks radio packets because transmitted byte offsets will be wrong.

// LoRA packet structure
typedef struct __attribute__((packed)) {
    uint8_t  version;
    uint8_t  team_id;
    uint8_t  node_id[6];

    uint16_t seq_number;

    // Unix seconds
    uint32_t ts_s;

    // Milliseconds part of the last seen time
    uint16_t ts_ms;

    // Position (mm)
    int32_t x_mm;
    int32_t y_mm;
    int32_t z_mm;

    // Velocity (mm/s)
    int32_t  vx_mm_s;
    int32_t  vy_mm_s;
    int32_t  vz_mm_s;

    // Yaw (centi-degrees)
    uint16_t yaw_cd;

    // Truncated MAC tag (4 bytes)
    uint8_t  mac_tag[4];

} radio_packet_t;

int compute_cmac(const uint8_t *message, size_t message_len, uint8_t *tag_out);
void state_to_radio_packet(radio_packet_t *pkt, const drone_state_t *state, uint32_t sequence, const uint8_t* node_id);
void radio_packet_to_state(const radio_packet_t *pkt, drone_state_t *state);
bool verify_packet(const radio_packet_t *pkt);

#ifdef __cplusplus
}
#endif

#endif // RADIO_H

