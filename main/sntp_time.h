#ifndef SNTP_TIME_H
#define SNTP_TIME_H

#include <stdint.h>
#include "esp_err.h"

#ifdef __cplusplus
extern "C" {
#endif

/**
 * @brief Initializes SNTP and synchronizes the system time.
 * 
 * This function blocks until the time is synchronized or retries have been exhausted.
 *
 * @return ESP_OK on success, or an error code on failure.
 */
esp_err_t sync_time(void);

/**
 * @brief Gets the current Unix time.
 *
 * Populates the given pointers with the number of seconds and milliseconds
 * since the Unix epoch.
 *
 * @param ts_s Pointer to store the seconds part of the timestamp.
 * @param ts_ms Pointer to store the milliseconds part of the timestamp.
 */
void get_current_unix_time(uint32_t* ts_s, uint16_t* ts_ms);

#ifdef __cplusplus
}
#endif

#endif // SNTP_TIME_H