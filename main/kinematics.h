#ifndef KINEMATICS_H
#define KINEMATICS_H

#include <stdint.h>
#include <stdbool.h>
#include "config.h"

#ifdef __cplusplus
extern "C" {
#endif

// 1. Physics integration task
// Drone state (relative to world frame)
typedef struct {
    // Position (mm)
    int32_t x;
    int32_t y;
    int32_t z;

    // Velocity (mm/s)
    int32_t vx;
    int32_t vy;
    int32_t vz;

    // Orientation (rad)
    uint16_t yaw_heading;
} drone_state_t;

// 2. Flocking control task (Reyholds cohesion-alignment-separation, could add for obstacle avoidance)
// Target velocity in world frame
typedef struct {
    int32_t target_vx;        // Target velocity in x (mm/s)
    int32_t target_vy;        // Target velocity in y (mm/s)
    int32_t target_vz;        // Target velocity in z (mm/s)
    int16_t target_yaw_rate;  // Target yaw rate (rad/s)
} flock_command_t;

// Initialise a drone state.
void drone_state_init(drone_state_t *s);
void flock_command_init(flock_command_t *c);

// Update drone state: integrates position from velocity, clamps boundaries, updates heading.
void drone_state_update(drone_state_t *s, int32_t vx_mms, int32_t vy_mms, int32_t vz_mms,
                        int32_t yaw_rate_cds, float dt);

#ifdef __cplusplus
}
#endif

#endif // KINEMATICS_H