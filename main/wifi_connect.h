#ifndef WIFI_CONNECT_H
#define WIFI_CONNECT_H

#include "esp_err.h"

#ifdef __cplusplus
extern "C" {
#endif

/**
 * @brief Connects to the configured Wi-Fi network.
 * 
 * This function blocks until a connection is established or a timeout occurs.
 *
 * @return ESP_OK on success, or an error code on failure.
 */
esp_err_t wifi_connect(void);

#ifdef __cplusplus
}
#endif

#endif // WIFI_CONNECT_H