#include "esp_err.h"
#include "esp_sntp.h"
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "sys/time.h"
#include "esp_log.h"

static const char *TAG = "SNTP_TIME";

esp_err_t sync_time()
{
    ESP_LOGI(TAG, "Initializing SNTP...");
    esp_sntp_setoperatingmode(SNTP_OPMODE_POLL);
    esp_sntp_setservername(0, "pool.ntp.org");
    esp_sntp_init();

    // Set timezone to GMT+0
    setenv("TZ", "GMT0", 1);
    tzset();

    // Wait for time to be set
    sntp_sync_status_t sync_status = SNTP_SYNC_STATUS_RESET;
    int retries = 0;
    while (sync_status != SNTP_SYNC_STATUS_COMPLETED && retries < 10) {
        sync_status = sntp_get_sync_status();
        vTaskDelay(pdMS_TO_TICKS(2000));
        retries++;
    }

    if (sync_status == SNTP_SYNC_STATUS_COMPLETED) {
        ESP_LOGI(TAG, "Time successfully synchronized");
        return ESP_OK;
    } else {
        ESP_LOGE(TAG, "Failed to synchronize time");
        return ESP_FAIL;
    }
}

void get_current_unix_time(uint32_t* ts_s, uint16_t* ts_ms)
{
    struct timeval tv;

    if (gettimeofday(&tv, NULL) == 0) {
        *ts_s = tv.tv_sec;
        *ts_ms = tv.tv_usec / 1000;
    } else {
        // Handle error
        ESP_LOGE(TAG, "Failed to get time of day");
        *ts_s = 0;
        *ts_ms = 0;
    }
}
