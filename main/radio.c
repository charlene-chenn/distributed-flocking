#include "config.h"
#include "kinematics.h"
#include "radio.h"
#include "esp_mac.h"
#include "esp_system.h"
#include "esp_log.h"
#include "esp_timer.h"
#include "mbedtls/cipher.h"
#include "mbedtls/cmac.h"
#include <string.h>
#include "sntp_time.h"

static const char *TAG = "RADIO"; 

// const uint8_t secret_key[16] = {
//     0x00, 0x01, 0x02, 0x03,
//     0x04, 0x05, 0x06, 0x07,
//     0x08, 0x09, 0x0A, 0x0B,
//     0x0C, 0x0D, 0x0E, 0x0F,
// };

const uint8_t secret_key[16] = {
    0x2B, 0x7E, 0x15, 0x16, 
    0x22, 0xA0, 0xD2, 0xA6,
    0xAC, 0xF7, 0x19, 0x88, 
    0x09, 0xCF, 0x4F, 0x3C,
};

int compute_cmac(const uint8_t *message, size_t message_len, uint8_t *tag_out) {
    const mbedtls_cipher_info_t *cipher_info = mbedtls_cipher_info_from_type(MBEDTLS_CIPHER_AES_128_ECB);
    if (cipher_info == NULL) {
        ESP_LOGE(TAG, "Failed to get cipher info for AES-128-ECB");
        return -1;
    }

    // Use the simpler, one-shot mbedtls_cipher_cmac function to ensure consistency
    int ret = mbedtls_cipher_cmac(
        cipher_info,     // Cipher to use
        secret_key,      // Authentication key
        128,             // Key length in bits
        message,         // Data to authenticate
        message_len,     // Length of data
        tag_out          // Output buffer for 16-byte tag
    );

    return ret;
}

// Convert drone state and radio identity into packet for transmission.
void state_to_radio_packet(radio_packet_t *pkt, const drone_state_t *state, uint32_t sequence, const uint8_t* node_id) {
    pkt->version = (uint8_t)PROTOCOL_VERSION;
    pkt->team_id = TEAM_ID;
    memcpy(pkt->node_id, node_id, 6);
    pkt->seq_number = (uint16_t)sequence;

    // Get current time in microseconds and convert to seconds + milliseconds
    uint32_t temp_s;
    uint16_t temp_ms;
    get_current_unix_time(&temp_s, &temp_ms);
    pkt->ts_s = temp_s;
    pkt->ts_ms = temp_ms;

    // Positions: Cast signed internal values to uint32_t
    pkt->x_mm = (uint32_t)state->x;
    pkt->y_mm = (uint32_t)state->y;
    pkt->z_mm = (uint32_t)state->z;

    pkt->vx_mm_s = state->vx;
    pkt->vy_mm_s = state->vy;
    pkt->vz_mm_s = state->vz;
    pkt->yaw_cd = state->yaw_heading;

    // Compute AES-128-CMAC over the packet (excluding mac_tag field)
    const uint8_t *m = (const uint8_t *)pkt;
    size_t n = offsetof(radio_packet_t, mac_tag);

    uint8_t tag16[16]; // Full 16-byte CMAC tag

    if (compute_cmac(m, n, tag16) != 0) {
        ESP_LOGE(TAG, "CMAC computation failed");
        memset(pkt->mac_tag, 0, sizeof(pkt->mac_tag));
        return;
    }

    // Copy last 4 bytes of the 16-byte tag to packet
    memcpy(pkt->mac_tag, tag16 + 12, 4);
}

// Convert received radio packet into internal state (mm, mm/s, rad).
void radio_packet_to_state(const radio_packet_t *pkt, drone_state_t *state) {
    // Packet positions are unsigned; clamp to INT32_MAX if too large
    if (pkt->x_mm > (uint32_t)INT32_MAX) state->x = INT32_MAX; else state->x = (int32_t)pkt->x_mm;
    if (pkt->y_mm > (uint32_t)INT32_MAX) state->y = INT32_MAX; else state->y = (int32_t)pkt->y_mm;
    if (pkt->z_mm > (uint32_t)INT32_MAX) state->z = INT32_MAX; else state->z = (int32_t)pkt->z_mm;
    state->vx = pkt->vx_mm_s;
    state->vy = pkt->vy_mm_s;
    state->vz = pkt->vz_mm_s;
    state->yaw_heading = (uint16_t)(pkt->yaw_cd & 0xFFFF);
}

bool verify_packet(const radio_packet_t *pkt) {
    // Check protocol version (explicit field)
    // Accept configured protocol or legacy 0
    uint8_t version = pkt->version;
    if (version != PROTOCOL_VERSION && version != 0) {
        ESP_LOGW(TAG, "Invalid packet version: %u (expected %d or 0) (protocol confusion)", version, PROTOCOL_VERSION);
        return false;
    }

    // Check team ID
    if (pkt->team_id != TEAM_ID) {
        ESP_LOGW(TAG, "Invalid team packet: %d (team spoofing)", pkt->team_id);
        return false;
    }

    // Check position bounds (detect overflow/out-of-bounds attack packets)
    // Check for negative positions (invalid in world coordinates)
    if (pkt->x_mm < 0 || pkt->y_mm < 0 || pkt->z_mm < 0) {
        ESP_LOGW(TAG, "Invalid position (negative values) x=%ld y=%ld z=%ld (invalid)",
                 (long)pkt->x_mm, (long)pkt->y_mm, (long)pkt->z_mm);
        return false;
    }
    
    if (pkt->x_mm > WORLD_LIMIT_MM || pkt->y_mm > WORLD_LIMIT_MM || pkt->z_mm > WORLD_LIMIT_MM) {
        ESP_LOGW(TAG, "Invalid position (exceeds world limits) x=%ld y=%ld z=%ld (max=%d mm)",
                 (long)pkt->x_mm, (long)pkt->y_mm, (long)pkt->z_mm, WORLD_LIMIT_MM);
        return false;
    }

    // Check velocity bounds (detect unrealistic velocity attacks)
    int32_t vx_abs = (pkt->vx_mm_s < 0) ? -pkt->vx_mm_s : pkt->vx_mm_s;
    int32_t vy_abs = (pkt->vy_mm_s < 0) ? -pkt->vy_mm_s : pkt->vy_mm_s;
    int32_t vz_abs = (pkt->vz_mm_s < 0) ? -pkt->vz_mm_s : pkt->vz_mm_s;
    
    if (vx_abs > MAX_VELOCITY_MMS || vy_abs > MAX_VELOCITY_MMS || vz_abs > MAX_VELOCITY_MMS) {
        ESP_LOGW(TAG, "Invalid velocity (exceed velocity limits): vx=%ld vy=%ld vz=%ld (max=%d mm/s)",
                 (long)pkt->vx_mm_s, (long)pkt->vy_mm_s, (long)pkt->vz_mm_s, MAX_VELOCITY_MMS);
        return false;
    }

    // Verify AES-128-CMAC tag (mac_tag)
    const uint8_t *m = (const uint8_t *)pkt;
    size_t n = offsetof(radio_packet_t, mac_tag);
    uint8_t computed_tag16[16]; // Full 16-byte CMAC tag

    if (compute_cmac(m, n, computed_tag16) != 0) {
        ESP_LOGE(TAG, "CMAC verification computation failed");
        return false;
    }

    if (memcmp(computed_tag16 + 12, pkt->mac_tag, 4) != 0) {
        ESP_LOGW(TAG, "CMAC verification failed (packet spoofing)");
        return false;
    }
    
    return true;
}