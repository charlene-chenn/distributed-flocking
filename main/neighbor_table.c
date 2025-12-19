#include "neighbor_table.h"
#include "esp_mac.h"
#include "esp_timer.h"
#include "esp_log.h"
#include <string.h>
#include <stdio.h>

static const char *TAG = "DRONE";

void neighbor_table_init(neighbor_t *table, int max_size)
{
    // Zero out the entire array
    memset(table, 0, sizeof(neighbor_t) * max_size);
}

static uint32_t get_time_ms(void)
{
    return (uint32_t)(esp_timer_get_time() / 1000);
}

bool is_sequence_number_valid(neighbor_t *table, const uint8_t* node_id, uint16_t new_seq, int *neighbor_count) {
    for (int i = 0; i < *neighbor_count; i++) {
        if (memcmp(table[i].node_id, node_id, 6) == 0) {
            if (!table[i].has_seq_number) {
                // First packet from this neighbor - accept any sequence
                return true;
            }
            
            // Handle sequence number wraparound (uint16_t: 0-65535)
            uint16_t last_seq = table[i].last_seq_number;
            
            // Simple check: new sequence should be greater than last
            // This handles normal increment and reasonable wraparound: detects replay
            if (new_seq > last_seq || (last_seq > 60000 && new_seq < 5000)) {
                return true; // Valid sequence progression
            }

            ESP_LOGW(TAG, "RX: Sequence reject from %02x:%02x - got seq=%u, last=%u, loss=%u", 
                     node_id[4], node_id[5], new_seq, last_seq);
            return false; // Duplicate or replay packet
        }
    }
    // New neighbor - accept first packet
    return true;
}

// Insert or overwrite neighbor entry
void neighbor_table_update(neighbor_t *table, const uint8_t node_id[6], const drone_state_t *state, uint16_t seq_number, int *neighbor_count) {
    int64_t now = get_time_ms(); // ms

    // Try to find an existing entry
    for (int i = 0; i < *neighbor_count; i++) {
        if (memcmp(table[i].node_id, node_id, 6) == 0) {
            table[i].state = *state;
            table[i].last_seen_ms = now;
            table[i].last_seq_number = seq_number;
            table[i].has_seq_number = true;
            ESP_LOGI("METRIC_NEIGHBORS", "NEIGHBOR_UPDATED,ID,%02x:%02x:%02x:%02x:%02x:%02x",
                     node_id[0], node_id[1], node_id[2], node_id[3], node_id[4], node_id[5]);
            return;
        }
    }

    // If not in neighbor table, add it in
    if (*neighbor_count < MAX_NEIGHBORS) {
        memcpy(table[*neighbor_count].node_id, node_id, 6);
        table[*neighbor_count].state = *state;
        table[*neighbor_count].last_seen_ms = now;
        table[*neighbor_count].last_seq_number = seq_number;
        table[*neighbor_count].has_seq_number = true;
        (*neighbor_count)++;
        ESP_LOGI("METRIC_NEIGHBORS", "NEIGHBOR_ADDED,ID, %02x:%02x:%02x:%02x:%02x:%02x,TOTAL, %d",
                 node_id[0], node_id[1], node_id[2], node_id[3], node_id[4], node_id[5], *neighbor_count);
    } else {
        // Could implement replacement policy; for now drop if full
        ESP_LOGW(TAG, "Neighbor table full, cannot add new neighbor");
    }
}

void neighbor_table_expire(neighbor_t *table, int *neighbor_count)
{
    int64_t now = get_time_ms(); // Convert to milliseconds
    int write_idx = 0;
    
    for (int read_idx = 0; read_idx < *neighbor_count; read_idx++) {
        if ((now - table[read_idx].last_seen_ms) < NEIGHBOR_TIMEOUT_MS) {
            // Keep this neighbor
            if (write_idx != read_idx) {
                table[write_idx] = table[read_idx];
            }
            write_idx++;
        } else {
            uint8_t* node_id = table[read_idx].node_id;
            ESP_LOGI("METRIC_NEIGHBORS", "NEIGHBOR_REMOVED,ID,%02x:%02x:%02x:%02x:%02x:%02x",
                     node_id[0], node_id[1], node_id[2], node_id[3], node_id[4], node_id[5]);
        }
    }
    
    *neighbor_count = write_idx;
}