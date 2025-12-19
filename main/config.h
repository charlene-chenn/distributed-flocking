#ifndef CONFIG_H
#define CONFIG_H

// Hardware Pins (LoRA SPI)
#define PIN_SPI_SCK             5       // SPI Clock
#define PIN_SPI_MISO            19      // SPI MISO
#define PIN_SPI_MOSI            27      // SPI MOSI
#define PIN_LORA_CS             18      // LoRa Chip Select (NSS)
#define PIN_LORA_RST            23      // LoRa Reset
#define PIN_LORA_DIO0           26      // LoRa DIO0 (IRQ)
#define PIN_LORA_DIO1           33      // LoRa DIO1 (optional)
#define BLINK_GPIO              2       // LED GPIO pin for status indication

// Given variables
#define PHYSICS_TASK_FREQ_HZ    50   // Physics update rate
#define FLOCKING_TASK_FREQ_HZ   10   // Flocking algorithm rate
#define RADIO_TASK_FREQ_HZ      1/6  // LoRa packet transmission rate
#define TELEMETRY_TASK_FREQ_HZ  2    // MQTT telemetry rate
#define WORLD_LIMIT_MM          100000  // 100 meters (100x100x100m simulation box)
#define PROTOCOL_VERSION        1       // Protocol version
#define TEAM_ID                 0       // Team identifier

// Kinematics
#define DRONE_VEL_TAU_S         0.5f    // Time constant for velocity response (s)
#define DRONE_YAW_TAU_S         0.4f    // Time constant for yaw rate response (s)

// Flocking
#define COHESION_WEIGHT         0.10f   // Weight for moving toward center of mass
#define ALIGNMENT_WEIGHT        0.08f   // Weight for matching neighbor velocity
#define SEPARATION_WEIGHT       1.0f   // Weight for avoiding crowding
#define MIN_SEPARATION_MM       5000    // 5 meters minimum separation distance
#define PERCEPTION_RADIUS_MM    30000    // 3 meters perception radius
#define MAX_VELOCITY_MMS        500     // 0.5 m/s maximum velocity

// Neighbor table
#define MAX_NEIGHBORS           10      // Maximum neighbors to track
#define NEIGHBOR_TIMEOUT_MS     20000    // Neighbor timeout in milliseconds, 10 seconds here (5 seconds is too short)

// LoRA Radio
#define LORA_FREQ_MHZ  868.1f  // MHz (UK/EU)
#define LORA_BW_KHZ    250.0f  // kHz - bandwidth (MUST match other drones!)
#define LORA_SF        9       // spreading factor (SF9=~200ms TX)
#define LORA_CR        7       // coding rate denominator 5..8
#define LORA_SYNCWORD  0x12    // LoRaWAN public sync word
#define LORA_PREAMBLE  10      // symbols
#define LORA_POWER_DBM 14      // TX power in dBm
#define LORA_CRC_ON    true
#define SEQ_HALF_RANGE        32768u
#define MAX_FORWARD_JUMP      2000u

// Energy Calculation
#define SUPPLY_VOLTAGE_V        3.3f    // 3.3V supply
#define ESP32_ACTIVE_CURRENT_MA 80.0f   // ESP32 active mode (WiFi + CPU)
#define SX1276_TX_CURRENT_MA    120.0f  // SX1276 TX at 14dBm (868MHz)
#define SX1276_RX_CURRENT_MA    12.0f   // SX1276 RX mode
#define SX1276_STANDBY_MA       1.5f    // SX1276 standby mode

#define MQTT_BROKER_URI "mqtt://broker.hivemq.com:1883"

// Adversarial attack settings
//#define ENABLE_REPLAY_ATTACK            // Comment out this line to disable the attack
//#define ENABLE_INT_OVERFLOW_ATTACK        // Comment out this line to disable the attack
//#define ENABLE_TABLE_OVERFLOW_ATTACK  // Comment out this line to disable the attack
//#define ENABLE_PACKET_SPOOFING_ATTACK // Comment out this line to disable the attack
//#define ENABLE_FLOODING_ATTACK        // Comment out this line to disable the attack

#endif // CONFIG_H