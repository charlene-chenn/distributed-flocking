#ifndef NEIGHBOR_TABLE_H
#define NEIGHBOR_TABLE_H

#include "kinematics.h"
#include "config.h"
#include <stdint.h>
#include <stdbool.h>

#ifdef __cplusplus
extern "C" {
#endif

// Global neighbor count (defined in main.cpp)
extern int global_neighbor_count;

// Neighbor table entry

typedef struct {
    uint8_t node_id[6]; // [cite: 54]
    drone_state_t state;
    int64_t last_seen_ms; // For culling stale entries
    uint16_t last_seq_number; // Last received sequence number
    bool has_seq_number; // Whether we have received any packets from this neighbor
} neighbor_t;

void neighbor_table_init(neighbor_t *table, int max_size);
bool is_sequence_number_valid(neighbor_t *table, const uint8_t* node_id, uint16_t new_seq, int *neighbor_count);
void neighbor_table_update(neighbor_t *table, const uint8_t node_id[6], const drone_state_t* state, uint16_t seq_number, int *neighbor_count);
void neighbor_table_expire(neighbor_t *table, int *neighbor_count);

#ifdef __cplusplus
}
#endif

#endif // NEIGHBOR_TABLE_H
