#include "config.h"
#include "kinematics.h"
// #include "radio.h"
#include <math.h>
#include <string.h>

void drone_state_init(drone_state_t *s)
{
    // Initialize at origin to world frame, zero velocity, zero orientation
    memset(s, 0, sizeof(*s));
}

void flock_command_init(flock_command_t *c)
{
    memset(c, 0, sizeof(*c));
}

void drone_state_update(drone_state_t *s, int32_t vx_mms, int32_t vy_mms, int32_t vz_mms,
                        int32_t yaw_rate_cds, float dt)
{
    // Integrate position using current velocity
    // Formula: d = v * dt, where v (mm/s), dt (seconds)
    float pos_x_mm = (float)s->x + (float)vx_mms * dt;
    float pos_y_mm = (float)s->y + (float)vy_mms * dt;
    float pos_z_mm = (float)s->z + (float)vz_mms * dt;

    // Clamp positions to stay within 100x100x100 meter world boundaries
    if (pos_x_mm < 0.0f){
        pos_x_mm = 0.0f;
    }
    if (pos_x_mm > WORLD_LIMIT_MM){
        pos_x_mm = WORLD_LIMIT_MM;
    }
    if (pos_y_mm < 0.0f){
        pos_y_mm = 0.0f;
    } 
    if (pos_y_mm > WORLD_LIMIT_MM){
        pos_y_mm = WORLD_LIMIT_MM;
    }
    if (pos_z_mm < 0.0f){
        pos_z_mm = 0.0f;
    }
    if (pos_z_mm > WORLD_LIMIT_MM){
        pos_z_mm = WORLD_LIMIT_MM;
    }

    // Update position
    s->x = (int32_t)roundf(pos_x_mm);
    s->y = (int32_t)roundf(pos_y_mm);
    s->z = (int32_t)roundf(pos_z_mm);

    // Update velocity
    s->vx = vx_mms;
    s->vy = vy_mms;
    s->vz = vz_mms;

    // Update heading
    float heading_cd_f = (float)s->yaw_heading + (float)yaw_rate_cds * dt;

    // Normalize heading to centidegrees
    int32_t heading_i = (int32_t)roundf(heading_cd_f);
    heading_i %= 36000;
    if (heading_i < 0) heading_i += 36000;
    s->yaw_heading = (uint16_t)heading_i;
}