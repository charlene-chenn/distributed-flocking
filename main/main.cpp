#include <stdio.h>
#include <math.h>
#include "esp_log.h"
#include "esp_mac.h"
#include "esp_timer.h"
#include "EspHal.h"
#include "driver/gpio.h"
#include "RadioLib.h"

#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "freertos/queue.h"     // for neighbor table
#include "freertos/semphr.h"    // for concurrency
#include "config.h"

#include "kinematics.h"
#include "neighbor_table.h"
#include "radio.h"

#include "nvs_flash.h"
#include "esp_netif.h"
#include "esp_event.h"
#include "esp_wifi.h"
#include "freertos/event_groups.h"
#include "mqtt_client.h"
#include <sys/time.h>
#include "sntp_time.h"
#include "wifi_connect.h"

static const char *TAG = "DRONE";
static uint8_t drone_node[6] = {0}; // Drone MAC Address

drone_state_t drone_state;
neighbor_t neighbor_table[MAX_NEIGHBORS];
QueueHandle_t flock_command;

SemaphoreHandle_t drone_state_mutex;
SemaphoreHandle_t neighbor_table_mutex;

// Radio objects
static SX1276* lora = nullptr;
static EspHal* hal = nullptr;
static uint32_t tx_sequence = 0;
int global_neighbor_count = 0;

// Telemetry
static bool mqtt_connected = false;
static esp_mqtt_client_handle_t mqtt_client = NULL;

// Attacks
#ifdef ENABLE_REPLAY_ATTACK
static radio_packet_t replay_packet;
static bool has_replay_packet = false;
#endif

void capture_replay_packet(const radio_packet_t* rx_packet) {
#ifdef ENABLE_REPLAY_ATTACK
    if (!has_replay_packet) {
        memcpy(&replay_packet, rx_packet, sizeof(radio_packet_t));
        has_replay_packet = true;
        int64_t timestamp_ms = esp_timer_get_time() / 1000;
        ESP_LOGW(TAG, "ATTACK: Replay Packet Captured, timestamp_ms=%lld", (long long)timestamp_ms);
    }
#endif
}

void run_replay_injection(SX1276* lora_dev) {
#ifdef ENABLE_REPLAY_ATTACK
    static int replay_counter = 0;
    // Run every ~1 second (100 * 10ms loop)
    if (has_replay_packet && ++replay_counter >= 100) { 
        replay_counter = 0;
        int64_t timestamp_ms = esp_timer_get_time() / 1000;
        ESP_LOGW(TAG, "ATTACK: Replaying packet seq=%u, timestamp_ms=%lld", replay_packet.seq_number, (long long)timestamp_ms);
        lora_dev->transmit((uint8_t*)&replay_packet, sizeof(replay_packet));
        lora_dev->startReceive(); // Resume RX immediately
    }
#endif
}

void run_int_overflow_attack(drone_state_t* state, uint32_t seq) {
#ifdef ENABLE_INT_OVERFLOW_ATTACK
    if (seq > 0 && seq % 3 == 0) { // For every 20 packets sent from me
        int64_t timestamp_ms = esp_timer_get_time() / 1000;
        ESP_LOGW(TAG, "ATTACK: Integer Overflow (Very large number), timestamp_ms=%lld", (long long)timestamp_ms);
        state->x = INT32_MAX;
        state->y = INT32_MAX;
    }
#endif
}

void run_table_overflow_attack(drone_state_t* state, uint8_t* node_id_out, uint32_t seq) {
#ifdef ENABLE_TABLE_OVERFLOW_ATTACK
    memcpy(node_id_out, drone_node, 6);
    static uint8_t fake_mac_counter = 0;
    node_id_out[5] = fake_mac_counter++;  // At each increment, new spoofed identity is created in each packet
    int64_t timestamp_ms = esp_timer_get_time() / 1000;
    ESP_LOGW(TAG, "ATTACK: Table Overflow (Fake ID), timestamp_ms=%lld", (long long)timestamp_ms);
#endif
}

void run_packet_spoofing(radio_packet_t* packet, uint32_t seq) {
#ifdef ENABLE_PACKET_SPOOFING_ATTACK
    if (seq > 0 && seq % 10 == 0) {
        int64_t timestamp_ms = esp_timer_get_time() / 1000;
        ESP_LOGW(TAG, "ATTACK: Packet Spoofing (Invalid CMAC), timestamp_ms=%lld", (long long)timestamp_ms);
        packet->x_mm += 50000; // Alter payload to break signature
    }
#endif
}

void run_flooding_attack(SX1276* lora_dev, drone_state_t* state, uint32_t* sequence, const uint8_t* node_id) {
#ifdef ENABLE_FLOODING_ATTACK
    static int flood_counter = 0;
    static int64_t last_flood_log = 0;
    
    // Send 10 packets rapidly every loop iteration (floods the network)
    for (int i = 0; i < 10; i++) {
        radio_packet_t flood_packet;
        state_to_radio_packet(&flood_packet, state, (*sequence)++, node_id);
        lora_dev->standby();
        lora_dev->transmit((uint8_t*)&flood_packet, sizeof(flood_packet));
        flood_counter++;
    }
    lora_dev->startReceive();
    
    // Log flooding stats every second
    int64_t now = esp_timer_get_time() / 1000;
    if (now - last_flood_log >= 1000) {
        ESP_LOGW(TAG, "ATTACK: Flooding - sent %d packets in last second, timestamp_ms=%lld", flood_counter, (long long)now);
        flood_counter = 0;
        last_flood_log = now;
    }
#endif
}

// Calculate periods in FreeRTOS ticks
// Hz is how many times / second, so each period would be (1 s or 1000 ms) / (times per s)
// How often each task runs
const TickType_t xPhysicsPeriodTicks = pdMS_TO_TICKS(1000 / PHYSICS_TASK_FREQ_HZ);      // 20 ms period
const TickType_t xFlockingPeriodTicks = pdMS_TO_TICKS(1000 / FLOCKING_TASK_FREQ_HZ);    // 100 ms period
const TickType_t xRadioPeriodTicks = pdMS_TO_TICKS(1000 * 6);                           // 6000 ms period (prevent lossy division)
const TickType_t xTelemetryPeriodTicks = pdMS_TO_TICKS(1000 / TELEMETRY_TASK_FREQ_HZ);  // 500 ms period
     
// Helper functions
float calculate_energy(float current_ma, int64_t period_us) {
    // Energy (uJ) = Power (mW) * Time (us) / 1000
    // Power (mW) = Voltage (V) * Current (mA)
    float power_mw = SUPPLY_VOLTAGE_V * current_ma;
    return power_mw * (float)period_us / 1000.0f;
}

static void log_energy_use(const char* component, float energy_uj, int64_t duration_us, float avg_power_mw) {
    ESP_LOGI("METRIC_ENERGY", "%s,ENERGY_UJ,%.2f,DURATION_US,%lld,AVG_POWER_MW,%.2f", 
             component, energy_uj, (long long)duration_us, avg_power_mw);
}

void set_metric_logging(bool enabled) {
    const char* metric_tags[] = {
        "METRIC_LATENCY",
        "METRIC_JITTER",
        "METRIC_NEIGHBORS",
        "METRIC_FLOCKING",
        "METRIC_ENERGY"
    };
    esp_log_level_t level = enabled ? ESP_LOG_INFO : ESP_LOG_NONE;
    for (int i = 0; i < 5; i++) {
        esp_log_level_set(metric_tags[i], level);
    }
}

bool radio_init(SX1276** lora_ptr, EspHal** hal_ptr) {
    ESP_LOGI(TAG, "Initializing RadioLib (SX1276)...");
    
    // Create HAL instance
    *hal_ptr = new EspHal(PIN_SPI_SCK, PIN_SPI_MISO, PIN_SPI_MOSI);
    if (!*hal_ptr) {
        ESP_LOGE(TAG, "Failed to create EspHal instance");
        return false;
    }
    
    // Create radio module instance
    Module* module = new Module(*hal_ptr, PIN_LORA_CS, PIN_LORA_DIO0, PIN_LORA_RST, PIN_LORA_DIO1);
    if (!module) {
        ESP_LOGE(TAG, "Failed to create Module instance");
        return false;
    }
    
    // Create SX1276 radio instance
    *lora_ptr = new SX1276(module);
    if (!*lora_ptr) {
        ESP_LOGE(TAG, "Failed to create SX1276 instance");
        return false;
    }
    
    // Initialize radio
    int state = (*lora_ptr)->begin(LORA_FREQ_MHZ, LORA_BW_KHZ, LORA_SF, LORA_CR, 
                                    LORA_SYNCWORD, LORA_PREAMBLE, LORA_POWER_DBM, LORA_CRC_ON);
    
    if (state != RADIOLIB_ERR_NONE) {
        ESP_LOGE(TAG, "Lora initialization failed, code %d", state);
        return false;
    }
    
    (*lora_ptr)->setOutputPower(LORA_POWER_DBM);
    
    ESP_LOGI(TAG, "Radio ready: %.1f MHz BW=%.0f kHz SF=%u CR=%u",
             LORA_FREQ_MHZ, LORA_BW_KHZ, LORA_SF, LORA_CR);
    return true;
}

static void mqtt_event_handler(void *handler_args, esp_event_base_t base, int32_t event_id, void *event_data)
{
    esp_mqtt_event_handle_t event = (esp_mqtt_event_handle_t)event_data;
    
    switch ((esp_mqtt_event_id_t)event_id) {
        case MQTT_EVENT_CONNECTED:
            ESP_LOGI(TAG, "MQTT Connected");
            mqtt_connected = true;
            break;
        case MQTT_EVENT_DISCONNECTED:
            ESP_LOGI(TAG, "MQTT Disconnected");
            mqtt_connected = false;
            break;
        case MQTT_EVENT_PUBLISHED:
            ESP_LOGD(TAG, "MQTT Published, msg_id=%d", event->msg_id);
            break;
        case MQTT_EVENT_ERROR:
            ESP_LOGE(TAG, "MQTT Error");
            mqtt_connected = false;
            break;
        default:
            break;
    }
}

static void init_mqtt(void)
{
    if (mqtt_client != NULL) {
        return; // Already initialized
    }
    
    esp_mqtt_client_config_t mqtt_cfg = {};
    mqtt_cfg.broker.address.uri = MQTT_BROKER_URI;
    
    mqtt_client = esp_mqtt_client_init(&mqtt_cfg);
    if (mqtt_client == NULL) {
        ESP_LOGE(TAG, "Failed to initialize MQTT client");
        return;
    }
    
    esp_mqtt_client_register_event(mqtt_client, (esp_mqtt_event_id_t)ESP_EVENT_ANY_ID, mqtt_event_handler, NULL);
    esp_mqtt_client_start(mqtt_client);
}
void physics_task(void *pvParameters) {
    TickType_t lastWakeTime = xTaskGetTickCount();
    flock_command_t cmd;
    
    // Initialize state
    drone_state_init(&drone_state);

    // Localize drone at middle of field
    if (xSemaphoreTake(drone_state_mutex, portMAX_DELAY) == pdTRUE) {
        drone_state.x = 50000;  // 50m in mm
        drone_state.y = 50000;  // 50m in mm
        drone_state.z = 1000;   // 1m in mm
        drone_state.yaw_heading = 0;
        xSemaphoreGive(drone_state_mutex);
    }

    // Initialize flocking velocity
    int32_t curr_vx = 100, curr_vy = 0, curr_vz = 0, curr_yaw_rate = 0;

    ESP_LOGI(TAG, "Physics Task started");

    for (;;) {
        TickType_t expectedWakeTimeTicks = xTaskGetTickCount();
        int32_t jitter_ticks = expectedWakeTimeTicks - lastWakeTime;
        int64_t task_start_us = esp_timer_get_time();

        // Check if there is new command for physics task to consume
        // If yes > update velocity target / If no > continue with previous targets
        if (xQueueReceive(flock_command, &cmd, 0) == pdTRUE) {
            ESP_LOGD(TAG, "Physics: Received new command");
            curr_vx = cmd.target_vx;
            curr_vy = cmd.target_vy;
            curr_vz = cmd.target_vz;
            curr_yaw_rate = cmd.target_yaw_rate;            
        }

        // Calculate dt
        const float dt = 1.0f / (float)PHYSICS_TASK_FREQ_HZ;

        // Update drone state with integrated physics
        // NOTE: Only take semaphore when necessary
        if (xSemaphoreTake(drone_state_mutex, pdMS_TO_TICKS(10)) == pdTRUE) {
            drone_state_update(&drone_state, curr_vx, curr_vy, curr_vz, curr_yaw_rate, dt);
            xSemaphoreGive(drone_state_mutex);
        } else {
            ESP_LOGE(TAG, "Physics task failed to get state mutex");
        }
        
        // Log data
        static int physics_log_counter = 0;
        if (++physics_log_counter >= 50) { // 50Hz * 1s = 50
            physics_log_counter = 0;
            int64_t task_duration_us = esp_timer_get_time() - task_start_us;
            ESP_LOGI("METRIC_LATENCY", "PHYSICS,EXEC_TIME_US,%lld", (long long)task_duration_us);
        
            ESP_LOGI("METRIC_JITTER", "PHYSICS,JITTER_TICKS,%ld", (long)jitter_ticks);
            
            // System energy consumption over 10 seconds
            static int64_t last_system_energy_log = 0;
            int64_t current_time_us = esp_timer_get_time();
            if (last_system_energy_log == 0) last_system_energy_log = current_time_us;
            
            int64_t period_us = current_time_us - last_system_energy_log;
            float system_current = ESP32_ACTIVE_CURRENT_MA + SX1276_STANDBY_MA; // Base system current
            float system_energy_uj = calculate_energy(system_current, period_us);
            float avg_system_power_mw = SUPPLY_VOLTAGE_V * system_current;
            
            ESP_LOGI("METRIC_ENERGY", "SYSTEM_BASE,ENERGY_UJ,%.2f,PERIOD_US,%lld,AVG_POWER_MW,%.2f", 
                     system_energy_uj, (long long)period_us, avg_system_power_mw);
            
            last_system_energy_log = current_time_us;
            
            // Print current position every 10 seconds
            if (xSemaphoreTake(drone_state_mutex, pdMS_TO_TICKS(10)) == pdTRUE) {
                ESP_LOGI(TAG, "PHYSICS: Position=[%ld, %ld, %ld] mm, Velocity=[%ld, %ld, %ld] mm/s, Yaw=%u cdeg",
                         (long)drone_state.x, (long)drone_state.y, (long)drone_state.z,
                         (long)drone_state.vx, (long)drone_state.vy, (long)drone_state.vz,
                         (unsigned)drone_state.yaw_heading);
                xSemaphoreGive(drone_state_mutex);
            }
        }

        // Delay until next cycle, period completed
        // Use delay until instead of delay to ensure execution time is not affecting timing
        vTaskDelayUntil(&lastWakeTime, xPhysicsPeriodTicks);
    }
}

void flocking_task(void *pvParameters) {
    TickType_t lastWakeTime = xTaskGetTickCount();
    neighbor_t local_table[MAX_NEIGHBORS]; // Create a copy for any changes made, keep global table unchanged
    int neighbor_count = 0;
    
    ESP_LOGI(TAG, "Flocking Task started");

    for (;;) {
        int64_t task_start_us = esp_timer_get_time();

        // Read my drone state
        drone_state_t curr_state;
        if (xSemaphoreTake(drone_state_mutex, pdMS_TO_TICKS(10)) == pdTRUE) {
            curr_state = drone_state;
            xSemaphoreGive(drone_state_mutex);
        } else {
            ESP_LOGE(TAG, "Flocking task failed to get drone state mutex");
            vTaskDelayUntil(&lastWakeTime, xFlockingPeriodTicks);
            continue; // Skip this cycle
        }

        // Copy global neighbor table to local table
        if (xSemaphoreTake(neighbor_table_mutex, pdMS_TO_TICKS(100)) == pdTRUE) {
            memcpy(local_table, neighbor_table, sizeof(neighbor_table));
            neighbor_count = global_neighbor_count;
            xSemaphoreGive(neighbor_table_mutex);
        } else {
            ESP_LOGE(TAG, "Flocking task failed to get neighbour mutex");
        }

        // Implement Reynold's flocking rules
        flock_command_t update_cmd;
        memset(&update_cmd, 0, sizeof(flock_command_t));
        int64_t min_separation_sq = -1; // Placeholder value if no neighbors exist

        // Separation param
        for (int i = 0; i < neighbor_count; i++) {
            // Check offset in each dimension offset_x = neighbor - self
            int64_t offset_x = local_table[i].state.x - curr_state.x;
            int64_t offset_y = local_table[i].state.y - curr_state.y;
            int64_t offset_z = local_table[i].state.z - curr_state.z;
            int64_t dist_sq = offset_x*offset_x + offset_y*offset_y + offset_z*offset_z;
            
            // Update separation metric
            if (min_separation_sq < 0 || dist_sq < min_separation_sq) {
                min_separation_sq = dist_sq;
            }
            
            if (dist_sq > 0 && dist_sq < MIN_SEPARATION_MM * MIN_SEPARATION_MM) {
                // Normalize by distance so there is a stronger push when they are very close
                float dist_norm = sqrtf((float)dist_sq);
                float scale = SEPARATION_WEIGHT * (MIN_SEPARATION_MM - dist_norm) / dist_norm;
                update_cmd.target_vx -= (int32_t)((float)offset_x * scale);
                update_cmd.target_vy -= (int32_t)((float)offset_y * scale);
                update_cmd.target_vz -= (int32_t)((float)offset_z * scale);
            }
        }
        
        // Calculate averages used for alignment and cohesion
        int32_t avg_x = 0, avg_y = 0, avg_z = 0;
        int32_t avg_vx = 0, avg_vy = 0, avg_vz = 0;

        if (neighbor_count > 0) {
            int64_t sum_x = 0, sum_y = 0, sum_z = 0;
            int64_t sum_vx = 0, sum_vy = 0, sum_vz = 0;

            for (int i = 0; i < neighbor_count; i++) {
                sum_x += local_table[i].state.x;
                sum_y += local_table[i].state.y;
                sum_z += local_table[i].state.z;
                sum_vx += local_table[i].state.vx;
                sum_vy += local_table[i].state.vy;
                sum_vz += local_table[i].state.vz;
            }
            
            // Cohesion param
            avg_vx = sum_vx / neighbor_count;
            avg_vy = sum_vy / neighbor_count;
            avg_vz = sum_vz / neighbor_count;
            
            avg_x = sum_x / neighbor_count;
            avg_y = sum_y / neighbor_count;
            avg_z = sum_z / neighbor_count;

            update_cmd.target_vx += (int32_t)((float)(avg_x - curr_state.x) * COHESION_WEIGHT);
            update_cmd.target_vy += (int32_t)((float)(avg_y - curr_state.y) * COHESION_WEIGHT);
            update_cmd.target_vz += (int32_t)((float)(avg_z - curr_state.z) * COHESION_WEIGHT);

            // Alignment param
            update_cmd.target_vx += (int32_t)((float)(avg_vx - curr_state.vx) * ALIGNMENT_WEIGHT);
            update_cmd.target_vy += (int32_t)((float)(avg_vy - curr_state.vy) * ALIGNMENT_WEIGHT);
            update_cmd.target_vz += (int32_t)((float)(avg_vz - curr_state.vz) * ALIGNMENT_WEIGHT);
            
            // Calculate desired heading based on velocity direction
            if (avg_vx != 0 || avg_vy != 0) {
                float desired_heading_rad = atan2f((float)avg_vy, (float)avg_vx);
                float desired_heading_deg = desired_heading_rad * 180.0f / M_PI;
                if (desired_heading_deg < 0) desired_heading_deg += 360.0f;
                
                // Convert current heading from centi-degrees to degrees
                float current_heading_deg = (float)curr_state.yaw_heading / 100.0f;
                
                // Calculate shortest angular difference
                float heading_diff = desired_heading_deg - current_heading_deg;
                if (heading_diff > 180.0f) heading_diff -= 360.0f;
                if (heading_diff < -180.0f) heading_diff += 360.0f;
                
                // Apply proportional yaw control (convert back to centi-degrees per second)
                update_cmd.target_yaw_rate = (int16_t)(heading_diff * 200.0f);
            }
        }
        
        // Log: Stability of Flocking (only when neighbors exist)
        // Distance to centroid - if high distance, drone is drifting away (cohesion)
        // Heading alignment - keeps consistent direction when value is high
        // Minimum separation - avoid collision when separation is too small

        if (neighbor_count > 0) {

            // Distance to centroid
            float dist_to_centroid = sqrtf(
                (float)(avg_x - curr_state.x) * (avg_x - curr_state.x) +
                (float)(avg_y - curr_state.y) * (avg_y - curr_state.y) +
                (float)(avg_z - curr_state.z) * (avg_z - curr_state.z)
            );

            // Heading alignment
            float dot_product = (float)curr_state.vx * avg_vx + (float)curr_state.vy * avg_vy + (float)curr_state.vz * avg_vz;
            float mag_my_v = sqrtf((float)curr_state.vx * curr_state.vx + (float)curr_state.vy * curr_state.vy + (float)curr_state.vz * curr_state.vz);
            float mag_align_v = sqrtf((float)avg_vx * avg_vx + (float)avg_vy * avg_vy + (float)avg_vz * avg_vz);
            float heading_alignment = 0.0f;
            if (mag_my_v > 0.0f && mag_align_v > 0.0f) {
                heading_alignment = dot_product / (mag_my_v * mag_align_v);
            }
            
            // Minimum separation
            float min_sep_dist = (min_separation_sq >= 0) ? sqrtf(min_separation_sq) : -1.0f;

            static int flocking_log_counter = 0;
            if (++flocking_log_counter >= 10) { // 10Hz * 1s = 10
                flocking_log_counter = 0;
                
                // Log Jitter/Latency
                int64_t task_duration_us = esp_timer_get_time() - task_start_us;
                ESP_LOGI("METRIC_LATENCY", "FLOCKING,EXEC_TIME_US,%lld", (long long)task_duration_us);
                
                TickType_t expectedWakeTimeTicks = xTaskGetTickCount();
                int32_t jitter_ticks = expectedWakeTimeTicks - lastWakeTime;
                ESP_LOGI("METRIC_JITTER", "FLOCKING,JITTER_TICKS,%ld", (long)jitter_ticks);

                // Log stability
                if (neighbor_count > 0) {
                    ESP_LOGI("METRIC_FLOCKING", "FLOCKING,MIN_SEP_MM,%.2f,DIST_TO_CENTROID_MM,%.2f,HEADING_ALIGNMENT,%.4f",
                            min_sep_dist, dist_to_centroid, heading_alignment);
                }
}
        }

        // Only send a command if we have neighbors to react to
        if (neighbor_count > 0) {
            // Clamp total velocity to MAX_VELOCITY_MMS
            float current_speed_sq = (float)update_cmd.target_vx * update_cmd.target_vx +
                                     (float)update_cmd.target_vy * update_cmd.target_vy +
                                     (float)update_cmd.target_vz * update_cmd.target_vz;
            float max_speed_sq = (float)MAX_VELOCITY_MMS * MAX_VELOCITY_MMS;

            if (current_speed_sq > max_speed_sq) {
                float ratio = sqrtf(max_speed_sq / current_speed_sq);
                update_cmd.target_vx = (int32_t)((float)update_cmd.target_vx * ratio);
                update_cmd.target_vy = (int32_t)((float)update_cmd.target_vy * ratio);
                update_cmd.target_vz = (int32_t)((float)update_cmd.target_vz * ratio);
            }

            // Send command to Physics task
            ESP_LOGD(TAG, "Flocking cmd: v=(%ld,%ld,%ld) yaw_rate=%d", 
                     (long)update_cmd.target_vx, (long)update_cmd.target_vy, 
                     (long)update_cmd.target_vz, update_cmd.target_yaw_rate);
            
            // Producer that writes to queue, doesn't write to physics task directly
            // Don't use shared variables that causes race conditions and lost updates, gives atomic, FIFO, thread-safe communication
            if (xQueueSend(flock_command, &update_cmd, 0) != pdTRUE) {
                ESP_LOGW(TAG, "Flocking: Velocity queue full!");
            }
        }

        // Delay until next cycle
        vTaskDelayUntil(&lastWakeTime, xFlockingPeriodTicks);
    }
}

extern "C" void radio_task(void *pvParameters) {
    drone_state_t local_state_copy;
    radio_packet_t rx_packet;
    radio_packet_t tx_packet;
    uint8_t rx_buffer[sizeof(radio_packet_t)];
    
    // Buffer for Node ID (Real or Fake)
    uint8_t tx_node_id[6]; 
    (void)rx_packet;
    (void)rx_buffer;
    (void)tx_node_id;

    static int64_t last_tx_time_ms = -6000;

    if (!radio_init(&lora, &hal)) { vTaskDelete(NULL); return; }
    
    ESP_LOGI(TAG, "Radio Task started");
    lora->standby();
    lora->startReceive();

    for (;;) {
        int64_t task_start_us = esp_timer_get_time();
        int64_t current_time_ms = task_start_us / 1000;

        // Transmit phase
        if (current_time_ms - last_tx_time_ms >= 6000) {
            last_tx_time_ms = current_time_ms;
            lora->standby();

            // Get State
            if (xSemaphoreTake(drone_state_mutex, pdMS_TO_TICKS(10)) == pdTRUE) {
                local_state_copy = drone_state;
                xSemaphoreGive(drone_state_mutex);
            }

            // Apply Attacks to State or ID (if flag enabled)
            run_int_overflow_attack(&local_state_copy, tx_sequence);
            run_table_overflow_attack(&local_state_copy, tx_node_id, tx_sequence);

            // Generate packet
            state_to_radio_packet(&tx_packet, &local_state_copy, tx_sequence, drone_node);

            // Apply Attacks to alter CMAC
            run_packet_spoofing(&tx_packet, tx_sequence);
            
            tx_sequence++;

            // Transmit
            int tx_state = lora->transmit((uint8_t*)&tx_packet, sizeof(tx_packet));
            if (tx_state == RADIOLIB_ERR_NONE) {
                int64_t tx_dur = esp_timer_get_time() - task_start_us;
                ESP_LOGI(TAG, "TX: Sent packet seq=%u pos=[%lu, %lu, %lu]", 
                         (unsigned)tx_packet.seq_number,
                         (unsigned long)tx_packet.x_mm,
                         (unsigned long)tx_packet.y_mm,
                         (unsigned long)tx_packet.z_mm);
                
                // Calculate TX energy consumption
                float tx_current_total = ESP32_ACTIVE_CURRENT_MA + SX1276_TX_CURRENT_MA; // ESP32 + LoRa TX
                float tx_energy_uj = calculate_energy(tx_current_total, tx_dur);
                float tx_power_mw = SUPPLY_VOLTAGE_V * tx_current_total;
                
                // Only log energy metrics every 10 seconds
                static int64_t last_tx_energy_log = 0;
                static float cumulative_tx_energy_uj = 0.0f;
                static int tx_count = 0;
                
                cumulative_tx_energy_uj += tx_energy_uj;
                tx_count++;
                
                int64_t current_time_ms = esp_timer_get_time() / 1000;
                if (current_time_ms - last_tx_energy_log >= 10000) {
                    log_energy_use("TX_TOTAL", cumulative_tx_energy_uj, tx_dur, tx_power_mw);
                    ESP_LOGI("METRIC_ENERGY", "TX_COUNT,%d,AVG_ENERGY_PER_TX_UJ,%.2f", 
                             tx_count, cumulative_tx_energy_uj / tx_count);
                    last_tx_energy_log = current_time_ms;
                    cumulative_tx_energy_uj = 0.0f;
                    tx_count = 0;
                }
            } else {
                // ESP_LOGW(TAG, "TX Fail: %d (%s)", tx_state, radiolib_error_name(tx_state));
                ESP_LOGW(TAG, "TX Fail: %d (%s)", tx_state);
            }
            lora->startReceive();
        }

        // Receive phase
        bool packet_received = false;
        if (gpio_get_level((gpio_num_t)PIN_LORA_DIO0) == 1) {
            packet_received = true;
        } else {
            vTaskDelay(pdMS_TO_TICKS(10));
        }

        if (packet_received) {
            int64_t rx_start = esp_timer_get_time();
            memset(rx_buffer, 0, sizeof(rx_buffer));
            
            int rx_state = lora->readData(rx_buffer, sizeof(rx_buffer));

            if (rx_state == RADIOLIB_ERR_NONE) {
                if (lora->getPacketLength() == sizeof(radio_packet_t)) {
                    memcpy(&rx_packet, rx_buffer, sizeof(radio_packet_t));
                    
                    // Filter out own packets first (before expensive CMAC verification)
                    if (memcmp(rx_packet.node_id, drone_node, 6) != 0) {
                        if (verify_packet(&rx_packet)) {
                            // Check sequence number validity before processing (prevent protocol confusion and team spoofing)
                            // Checking CMAC ensures data integrity, was not modified by the user in the process
                            if (is_sequence_number_valid(neighbor_table, rx_packet.node_id, rx_packet.seq_number, &global_neighbor_count)) {
                                ESP_LOGI(TAG, "RX: Valid packet from %02x:%02x seq=%u pos=[%lu, %lu, %lu]",
                                         rx_packet.node_id[4], rx_packet.node_id[5], 
                                         (unsigned)rx_packet.seq_number,
                                         (unsigned long)rx_packet.x_mm,
                                         (unsigned long)rx_packet.y_mm,
                                         (unsigned long)rx_packet.z_mm);
                                
                                // Capture for Replay Attack
                                capture_replay_packet(&rx_packet);

                                // Update State
                                drone_state_t n_state;
                                radio_packet_to_state(&rx_packet, &n_state);
                                if (xSemaphoreTake(neighbor_table_mutex, pdMS_TO_TICKS(10)) == pdTRUE) {
                                    neighbor_table_update(neighbor_table, rx_packet.node_id, &n_state, rx_packet.seq_number, &global_neighbor_count);
                                    neighbor_table_expire(neighbor_table, &global_neighbor_count);
                                    xSemaphoreGive(neighbor_table_mutex);

                                }
                            } else {
                                ESP_LOGW(TAG, "Blocked: Rejected duplicate/replay packet from %02x:%02x seq=%u",
                                         rx_packet.node_id[4], rx_packet.node_id[5], (unsigned)rx_packet.seq_number);
                            }
                        } else {
                            // Failed if protocol version, team ID, or cmac/secret key incompatible
                            ESP_LOGW(TAG, "Blocked: Fail from %02x:%02x:%02x:%02x:%02x:%02x",
                                     rx_packet.node_id[0], rx_packet.node_id[1], rx_packet.node_id[2],
                                     rx_packet.node_id[3], rx_packet.node_id[4], rx_packet.node_id[5]);
                        }
                    }else{
                        ESP_LOGW(TAG, "Blocked: Rejected own packet, seq=%u", (unsigned)rx_packet.seq_number);
                    }
                }
                int64_t rx_dur = esp_timer_get_time() - rx_start;
                
                // Calculate RX energy consumption
                float rx_current_total = ESP32_ACTIVE_CURRENT_MA + SX1276_RX_CURRENT_MA; // ESP32 + LoRa RX
                float rx_energy_uj = calculate_energy(rx_current_total, rx_dur);
                float rx_power_mw = SUPPLY_VOLTAGE_V * rx_current_total;
                
                // Only log energy metrics every 10 seconds
                static int64_t last_rx_energy_log = 0;
                static float cumulative_rx_energy_uj = 0.0f;
                static int rx_count = 0;
                
                cumulative_rx_energy_uj += rx_energy_uj;
                rx_count++;
                
                int64_t current_time_ms = esp_timer_get_time() / 1000;
                if (current_time_ms - last_rx_energy_log >= 10000) {
                    log_energy_use("RX_TOTAL", cumulative_rx_energy_uj, rx_dur, rx_power_mw);
                    ESP_LOGI("METRIC_ENERGY", "RX_COUNT,%d,AVG_ENERGY_PER_RX_UJ,%.2f", 
                             rx_count, cumulative_rx_energy_uj / rx_count);
                    last_rx_energy_log = current_time_ms;
                    cumulative_rx_energy_uj = 0.0f;
                    rx_count = 0;
                }
            }
            lora->startReceive();
        }

        run_replay_injection(lora);
        run_flooding_attack(lora, &local_state_copy, &tx_sequence, drone_node);
    }
}

void telemetry_task(void *pvParameters) {
    TickType_t lastWakeTime = xTaskGetTickCount();
    drone_state_t local_state_copy;
    char payload[512];
    char mqtt_topic[64];
    
    // Construct MQTT topic for visualiser
    snprintf(mqtt_topic, sizeof(mqtt_topic), "flocksim");
    
    ESP_LOGI(TAG, "Telemetry Task started");
    ESP_LOGI(TAG, "Will publish to topic: %s", mqtt_topic);

    for (;;) {
        int64_t task_start_us = esp_timer_get_time();

        // Get a copy of the current state for publishing
        if (xSemaphoreTake(drone_state_mutex, pdMS_TO_TICKS(10)) == pdTRUE) {
            local_state_copy = drone_state;
            xSemaphoreGive(drone_state_mutex);
        } else {
            ESP_LOGE(TAG, "Telemetry task failed to get state mutex");
            vTaskDelayUntil(&lastWakeTime, xTelemetryPeriodTicks);
            continue; // Skip this cycle
        }

        // Format local_state_copy into JSON format for visualiser
        // Get current time for timestamp
        struct timeval tv;
        gettimeofday(&tv, NULL);
        
        // Create a temporary LoRa packet to get the MAC tag
        radio_packet_t temp_packet;
        uint32_t temp_seq = 0; // Use 0 for telemetry or increment a counter
        state_to_radio_packet(&temp_packet, &local_state_copy, temp_seq, drone_node);
        
        snprintf(payload, sizeof(payload),
                 "{"
                 "\"version\":%d,"
                 "\"team_id\":%d,"
                 "\"node_id\":\"%02x%02x%02x%02x%02x%02x\","
                 "\"seq_number\":%lu,"
                 "\"ts_s\":%lld,"
                 "\"ts_ms\":%ld,"
                 "\"x_mm\":%ld,"
                 "\"y_mm\":%ld,"
                 "\"z_mm\":%ld,"
                 "\"vx_mm_s\":%ld,"
                 "\"vy_mm_s\":%ld,"
                 "\"vz_mm_s\":%ld,"
                 "\"yaw_cd\":%u,"
                 "\"mac_tag\":\"%02x%02x%02x%02x\""
                 "}",
                 PROTOCOL_VERSION,
                 TEAM_ID,
                 drone_node[0], drone_node[1], drone_node[2],
                 drone_node[3], drone_node[4], drone_node[5],
                 temp_seq,
                 tv.tv_sec,
                 tv.tv_usec / 1000,
                 (long)local_state_copy.x,
                 (long)local_state_copy.y,
                 (long)local_state_copy.z,
                 (long)local_state_copy.vx,
                 (long)local_state_copy.vy,
                 (long)local_state_copy.vz,
                 (unsigned)local_state_copy.yaw_heading,
                 temp_packet.mac_tag[0], temp_packet.mac_tag[1], 
                 temp_packet.mac_tag[2], temp_packet.mac_tag[3]);

        // Publish JSON to MQTT topic "flocksim" for visualiser
        if (mqtt_connected && mqtt_client != NULL) {
            int msg_id = esp_mqtt_client_publish(mqtt_client, mqtt_topic, payload, 0, 1, 0);
            ESP_LOGD(TAG, "Published to MQTT, msg_id=%d: %s", msg_id, payload);
        } else {
            ESP_LOGW(TAG, "MQTT not connected");
            ESP_LOGD(TAG, "Publish msg: %s", payload);
        }

        static int telemetry_log_counter = 0;
        if (++telemetry_log_counter >= 2) { // 2Hz * 1s = 2
            telemetry_log_counter = 0;
            
            TickType_t expectedWakeTimeTicks = xTaskGetTickCount();
            int32_t jitter_ticks = expectedWakeTimeTicks - lastWakeTime;
            ESP_LOGI("METRIC_JITTER", "TELEMETRY,JITTER_TICKS,%ld", (long)jitter_ticks);
            
            int64_t task_duration_us = esp_timer_get_time() - task_start_us;
            ESP_LOGI("METRIC_LATENCY", "TELEMETRY,EXEC_TIME_US,%lld", (long long)task_duration_us);
        }

        // Delay until next cycle
        vTaskDelayUntil(&lastWakeTime, xTelemetryPeriodTicks);
    }
}

extern "C" void app_main(void)
{
    ESP_LOGI(TAG, "Starting swarm simulation...");
    
    set_metric_logging(true);

    gpio_reset_pin((gpio_num_t)BLINK_GPIO);
    gpio_set_direction((gpio_num_t)BLINK_GPIO, GPIO_MODE_OUTPUT);

    // Initialize drone state
    drone_state_mutex = xSemaphoreCreateMutex();
    if(drone_state_mutex == NULL) {
        ESP_LOGE(TAG, "Failed to create state mutex");
    }

    // Initialize flocking command queue
    flock_command = xQueueCreate(5, sizeof(flock_command_t));
    if(flock_command == NULL) {
        ESP_LOGE(TAG, "Failed to create flock command queue");
    }

    // Initialize neighbor table
    neighbor_table_mutex = xSemaphoreCreateMutex();
    if(neighbor_table_mutex == NULL) {
        ESP_LOGE(TAG, "Failed to create neighbour mutex");
    }

    // Initialize node ID with device MAC address
    esp_read_mac(drone_node, ESP_MAC_WIFI_STA);
    ESP_LOGI(TAG, "Node ID (MAC): %02x:%02x:%02x:%02x:%02x:%02x",
             drone_node[0], drone_node[1], drone_node[2],
             drone_node[3], drone_node[4], drone_node[5]);

    // Connect to Wi-Fi and sync time
    wifi_connect();
    if (sync_time() != ESP_OK) {
        ESP_LOGE(TAG, "Time sync failed, continuing with monotonic time.");
    } else {
        // Print current timestamp after successful sync
        uint32_t ts_s = 0;
        uint16_t ts_ms = 0;
        get_current_unix_time(&ts_s, &ts_ms);
        struct timeval tv;
        gettimeofday(&tv, NULL);
        ESP_LOGI(TAG, "Time synced successfully. Unix time: %u.%03u s, gettimeofday: %ld.%06ld", 
                 ts_s, ts_ms, (long)tv.tv_sec, (long)tv.tv_usec);
    }

    // Initialize MQTT client for visualiser
    ESP_LOGI(TAG, "Initializing MQTT client for visualizer...");
    init_mqtt();

    // FRAMEWORK TASK 2: Set up task priority
    xTaskCreate(physics_task, "Physics Task", 8192, NULL, 10, NULL);      // Priority 10
    xTaskCreate(flocking_task, "Flocking Task", 8192, NULL, 8, NULL);     // Priority 8
    xTaskCreate(radio_task, "Radio Task", 8192, NULL, 6, NULL);           // Priority 6
    xTaskCreate(telemetry_task, "Telemetry Task", 8192, NULL, 4, NULL);   // Priority 4

    ESP_LOGI(TAG, "All tasks created.");
};
