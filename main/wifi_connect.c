// wifi_connect.c
#include "wifi_connect.h"

// ------------------------------------------------------------ WiFi
// 0 - Home WiFi
// 1 - Eduroam
#define USE_EDUROAM 0

#if USE_EDUROAM
#define WIFI_SSID "eduroam"
#define EDUROAM_IDENTITY "identity"
#define EDUROAM_USERNAME "username"
#define EDUROAM_PASSWORD "password"
#else
#define WIFI_SSID "wifi"
#define WIFI_PASSWORD "password"
#endif

#define WIFI_CONNECT_TIMEOUT_MS 30000

#include "esp_event.h"
#include "esp_netif.h"
#include "esp_wifi.h"
#include "freertos/FreeRTOS.h"
#include "freertos/event_groups.h"
#include "nvs_flash.h"
#include <string.h>
#include "esp_log.h" // Add esp_log.h for ESP_LOG macros

static const char *TAG = "WIFI_CONNECT";

#if USE_EDUROAM
#include "esp_eap_client.h"
#endif

#define CHECK_ERR(x)                                              \
    do {                                                          \
        esp_err_t err = (x);                                      \
        if (err != ESP_OK) {                                      \
            ESP_LOGE(TAG, "%s failed with err %d", #x, err); \
            vTaskDelay(portMAX_DELAY);                            \
        }                                                         \
    } while (0)

// Event group to signal when we are connected
static EventGroupHandle_t s_wifi_event_group;
static const int WIFI_CONNECTED_BIT = BIT0;

/**
 * @brief Event handler for Wi-Fi and IP events.
 *
 * This function handles events like starting the station, disconnection,
 * and obtaining an IP address.
 *
 * @param arg           User data passed to the event handler (not used).
 * @param base          The event base.
 * @param id            The event ID.
 * @param data          The event data.
 */
static void wifi_event_handler(void* arg, esp_event_base_t base, int32_t id, void* data)
{
    if (base == WIFI_EVENT && id == WIFI_EVENT_STA_START) {
        // This event fires when Wi-Fi is started, now we can connect.
        esp_wifi_connect();
    } else if (base == WIFI_EVENT && id == WIFI_EVENT_STA_DISCONNECTED) {
        // Simple retry-forever logic.
        ESP_LOGW(TAG, "Disconnected, retrying...");
        esp_wifi_connect();
    } else if (base == IP_EVENT && id == IP_EVENT_STA_GOT_IP) {
        // We got an IP address!
        const ip_event_got_ip_t* e = (const ip_event_got_ip_t*)data;
        ESP_LOGI(TAG, "Got IP: %d.%d.%d.%d", IP2STR(&e->ip_info.ip));
        // Set the bit to unblock the wifi_connect_init function
        xEventGroupSetBits(s_wifi_event_group, WIFI_CONNECTED_BIT);
    }
}

#if USE_EDUROAM
/**
 * @brief Connects to a WPA2-Enterprise network (eduroam).
 * @return ESP_OK on success, or an error code on failure.
 */
static esp_err_t wifi_connect_eduroam(void)
{
    // 1. Initialize NVS (Required for Wi-Fi)
    esp_err_t ret = nvs_flash_init();
    if (ret == ESP_ERR_NVS_NO_FREE_PAGES || ret == ESP_ERR_NVS_NEW_VERSION_FOUND) {
        CHECK_ERR(nvs_flash_erase());
        CHECK_ERR(nvs_flash_init());
    } else {
        CHECK_ERR(ret);
    }

    // 2. Create the event group
    s_wifi_event_group = xEventGroupCreate();

    // 3. Initialize Network Stack (TCP/IP) and Event Loop
    CHECK_ERR(esp_netif_init());
    CHECK_ERR(esp_event_loop_create_default());
    esp_netif_create_default_wifi_sta();

    // 4. Initialize Wi-Fi Driver
    wifi_init_config_t wcfg = WIFI_INIT_CONFIG_DEFAULT();
    CHECK_ERR(esp_wifi_init(&wcfg));

    // 5. Register Event Handlers
    CHECK_ERR(esp_event_handler_register(WIFI_EVENT, ESP_EVENT_ANY_ID, &wifi_event_handler, NULL));
    CHECK_ERR(esp_event_handler_register(IP_EVENT, IP_EVENT_STA_GOT_IP, &wifi_event_handler, NULL));

    // 6. Configure Wi-Fi with credentials from conf.h
    wifi_config_t cfg = { 0 };
    strncpy((char*)cfg.sta.ssid, WIFI_SSID, sizeof(cfg.sta.ssid));
    cfg.sta.threshold.authmode = WIFI_AUTH_WPA2_ENTERPRISE;

    CHECK_ERR(esp_wifi_set_mode(WIFI_MODE_STA));
    CHECK_ERR(esp_wifi_set_config(WIFI_IF_STA, &cfg));

    // PEAP / MSCHAPv2 credentials
    CHECK_ERR(esp_eap_client_set_identity((const uint8_t*)EDUROAM_IDENTITY, strlen(EDUROAM_IDENTITY)));
    CHECK_ERR(esp_eap_client_set_username((const uint8_t*)EDUROAM_USERNAME, strlen(EDUROAM_USERNAME)));
    CHECK_ERR(esp_eap_client_set_password((const uint8_t*)EDUROAM_PASSWORD, strlen(EDUROAM_PASSWORD)));

    CHECK_ERR(esp_wifi_sta_enterprise_enable());

    // 7. CRITICAL: Disable Wi-Fi power save for low-latency (from professor's code)
    CHECK_ERR(esp_wifi_set_ps(WIFI_PS_NONE));

    // 8. Start Wi-Fi
    CHECK_ERR(esp_wifi_start());

    // 9. Block and Wait for Connection (or Timeout)
    ESP_LOGI(TAG, "Connecting to SSID: %s...", WIFI_SSID);
    EventBits_t bits = xEventGroupWaitBits(
        s_wifi_event_group,
        WIFI_CONNECTED_BIT,
        pdFALSE, // Don't clear the bit on exit
        pdFALSE, // Wait for *any* bit (we only have one)
        pdMS_TO_TICKS(WIFI_CONNECT_TIMEOUT_MS));

    if (bits & WIFI_CONNECTED_BIT) {
        ESP_LOGI(TAG, "Wi-Fi Connected.");
        return ESP_OK;
    } else {
        ESP_LOGE(TAG, "Timed out waiting for Wi-Fi connection.");
        return ESP_ERR_TIMEOUT;
    }
}
#else
/**
 * @brief Connects to a standard WPA2-PSK network.
 * @return ESP_OK on success, or an error code on failure.
 */
static esp_err_t wifi_connect_home(void)
{
    // 1. Initialize NVS (Required for Wi-Fi)
    esp_err_t ret = nvs_flash_init();
    if (ret == ESP_ERR_NVS_NO_FREE_PAGES || ret == ESP_ERR_NVS_NEW_VERSION_FOUND) {
        CHECK_ERR(nvs_flash_erase());
        CHECK_ERR(nvs_flash_init());
    } else {
        CHECK_ERR(ret);
    }

    // 2. Create the event group
    s_wifi_event_group = xEventGroupCreate();

    // 3. Initialize Network Stack (TCP/IP) and Event Loop
    CHECK_ERR(esp_netif_init());
    CHECK_ERR(esp_event_loop_create_default());
    esp_netif_create_default_wifi_sta();

    // 4. Initialize Wi-Fi Driver
    wifi_init_config_t wcfg = WIFI_INIT_CONFIG_DEFAULT();
    CHECK_ERR(esp_wifi_init(&wcfg));

    // 5. Register Event Handlers
    CHECK_ERR(esp_event_handler_register(WIFI_EVENT, ESP_EVENT_ANY_ID, &wifi_event_handler, NULL));
    CHECK_ERR(esp_event_handler_register(IP_EVENT, IP_EVENT_STA_GOT_IP, &wifi_event_handler, NULL));

    // 6. Configure Wi-Fi with credentials from conf.h
    wifi_config_t cfg = { 0 };
    cfg.sta.threshold.authmode = WIFI_AUTH_WPA2_PSK;
    // Use safe bounded copy for credentials
    strncpy((char*)cfg.sta.ssid, WIFI_SSID, sizeof(cfg.sta.ssid));
    strncpy((char*)cfg.sta.password, WIFI_PASSWORD, sizeof(cfg.sta.password));

    CHECK_ERR(esp_wifi_set_mode(WIFI_MODE_STA));
    CHECK_ERR(esp_wifi_set_config(WIFI_IF_STA, &cfg));

    // 7. CRITICAL: Disable Wi-Fi power save for low-latency (from professor's code)
    CHECK_ERR(esp_wifi_set_ps(WIFI_PS_NONE));

    // 8. Start Wi-Fi
    CHECK_ERR(esp_wifi_start());

    // 9. Block and Wait for Connection (or Timeout)
    ESP_LOGI(TAG, "Connecting to SSID: %s...", WIFI_SSID);
    EventBits_t bits = xEventGroupWaitBits(
        s_wifi_event_group,
        WIFI_CONNECTED_BIT,
        pdFALSE, // Don't clear the bit on exit
        pdFALSE, // Wait for *any* bit (we only have one)
        pdMS_TO_TICKS(WIFI_CONNECT_TIMEOUT_MS));

    if (bits & WIFI_CONNECTED_BIT) {
        ESP_LOGI(TAG, "Wi-Fi Connected.");
        return ESP_OK;
    } else {
        ESP_LOGE(TAG, "Timed out waiting for Wi-Fi connection.");
        return ESP_ERR_TIMEOUT;
    }
}
#endif

esp_err_t wifi_connect(void)
{
#if USE_EDUROAM
    return wifi_connect_eduroam();
#else
    return wifi_connect_home();
#endif
}
