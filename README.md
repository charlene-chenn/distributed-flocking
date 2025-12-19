# Distributed Flocking - ESP32 Drone Swarm

A real-time distributed flocking system implemented on ESP32 microcontrollers using FreeRTOS, featuring Reynolds-style behavioral algorithms, LoRa radio communication, and cryptographic security measures with comprehensive adversarial testing.

## Overview

This project implements a simulated drone swarm where each ESP32 board represents an autonomous agent that:
- Maintains virtual 3D position and velocity through physics simulation
- Communicates with neighbors via LoRa radio at 915MHz
- Publishes telemetry over Wi-Fi using MQTT
- Executes Reynolds flocking algorithms (cohesion, separation, alignment)
- Authenticates messages using AES-CMAC cryptographic tags
- Logs performance metrics for timing, stability, and security analysis

## System Architecture

### FreeRTOS Task Design

| Task | Frequency | Priority | Function |
|------|-----------|----------|----------|
| **Physics Task** | 50 Hz | High | Integrates position/velocity, applies commanded acceleration, enforces boundary conditions |
| **Flocking Task** | 10 Hz | Medium | Computes Reynolds rules from neighbor states, generates velocity/yaw commands |
| **Radio Task** | 1/6 Hz | Medium-Low | Broadcasts state via LoRa, receives and validates neighbor packets |
| **Telemetry Task** | 2-5 Hz | Low | Publishes JSON state via MQTT over Wi-Fi |

### Key Data Structures

```c
typedef struct {
    int32_t x, y, z;        // Position in mm
    int32_t vx, vy, vz;     // Velocity in mm/s
    int32_t yaw_cdeg;       // Yaw angle in centi-degrees
    uint32_t timestamp_ms;  // System uptime
    uint8_t node_id;        // Unique identifier
    uint16_t sequence_num;  // Packet counter
} drone_state_t;

typedef struct {
    drone_state_t state;
    uint32_t last_rx_ms;    // Last received timestamp
    bool valid;             // Entry validity flag
} neighbour_t;
```

## Hardware Requirements

- **ESP32 Development Board** (ESP32-WROOM-32)
- **SX1276/RFM95W LoRa Module** (915MHz)
- **USB-Serial Adapter** (for programming/debugging)
- **Power Supply** (5V USB or 3.7V LiPo)

### Pin Configuration

| Function | GPIO Pin |
|----------|----------|
| LoRa NSS | GPIO 5 |
| LoRa RST | GPIO 26 |
| LoRa DIO0 | GPIO 25 |
| SPI MOSI | GPIO 23 |
| SPI MISO | GPIO 19 |
| SPI SCK | GPIO 18 |

## Software Dependencies

- **ESP-IDF v5.x** - Espressif IoT Development Framework
- **FreeRTOS** - Real-time operating system (included with ESP-IDF)
- **RadioLib** - LoRa communication library
- **mbedTLS** - Cryptographic functions (AES-CMAC)
- **ESP-MQTT** - MQTT client library

## Building and Flashing

### Setup ESP-IDF Environment

```bash
# Install ESP-IDF (if not already installed)
git clone -b v5.1 --recursive https://github.com/espressif/esp-idf.git
cd esp-idf
./install.sh esp32

# Activate environment
. ./export.sh
```

### Configure Project

```bash
cd drone
idf.py menuconfig

# Configure:
# - WiFi SSID and password
# - MQTT broker address
# - Node ID (unique per device)
```

### Build and Flash

```bash
# Build project
idf.py build

# Flash to device
idf.py -p /dev/ttyUSB0 flash

# Monitor serial output
idf.py -p /dev/ttyUSB0 monitor
```

## Configuration

Edit `main/config.h` to adjust system parameters:

```c
// Physics simulation
#define PHYSICS_TASK_FREQ_HZ 50
#define MAX_VELOCITY_MM_S 500
#define MAX_ACCELERATION_MM_S2 200

// Flocking parameters
#define COHESION_WEIGHT 1.0f
#define SEPARATION_WEIGHT 2.0f
#define ALIGNMENT_WEIGHT 1.5f
#define NEIGHBOR_RADIUS_MM 3000
#define SEPARATION_RADIUS_MM 1000

// Radio communication
#define RADIO_TASK_FREQ_HZ (1.0f/6.0f)  // Every 6 seconds
#define LORA_FREQUENCY 915.0
#define LORA_BANDWIDTH 125.0
#define LORA_SPREADING_FACTOR 7
#define LORA_TX_POWER 17

// Security
#define USE_AES_CMAC 1
#define MAC_TAG_LENGTH 4  // Truncated to 4 bytes
```

## Security Features

### Authentication Mechanism

- **Algorithm**: AES-CMAC (Cipher-based Message Authentication Code)
- **Key Length**: 128 bits (shared secret)
- **Tag Length**: 32 bits (truncated for bandwidth efficiency)
- **Protected Fields**: Node ID, position, velocity, timestamp, sequence number

### Adversarial Testing Scenarios

1. **Replay Attack**: Retransmit previously captured valid packets
2. **Replay Defense**: Drop packets with duplicate sequence numbers
3. **Packet Spoofing**: Inject false position/velocity data
4. **Table Overflow**: Flood with packets from non-existent nodes
5. **Timestamp Manipulation**: Send packets with invalid timestamps

## Data Collection and Analysis

### Serial Logging

Capture real-time telemetry:

```bash
./capture_serial.sh /dev/ttyUSB0
```

Output files:
- `normal_YYYYMMDD_HHMMSS.txt` - Baseline operation
- `replay_attack_YYYYMMDD_HHMMSS.txt` - Replay attack scenario
- `spoofing_attack_YYYYMMDD_HHMMSS.txt` - Spoofing scenario
- etc.

### Visualization and Analysis

Python scripts for metric analysis:

```bash
# Neighbor availability and communication quality
python compare_neighbor_availability.py

# Flocking stability metrics
python compare_flocking.py

# Task timing analysis
python compare_task_timing.py
```

Generated metrics include:
- **Neighbor Availability**: Percentage of time neighbors are visible
- **Packet Loss Rate**: Missing sequence number gaps
- **Sequence Gaps**: Count of discontinuities per neighbor
- **Flocking Stability**: Centroid distance, minimum separation, heading alignment
- **Task Timing**: Execution period, jitter, worst-case latency
- **Energy Consumption**: Estimated based on duty cycle and TX power

## Performance Metrics

### Timing Requirements

| Metric | Target | Typical |
|--------|--------|---------|
| Physics Task Period | 20 ms | 19.8-20.2 ms |
| Physics Task Jitter | <1 ms | 0.3 ms |
| Flocking Task Period | 100 ms | 99.5-100.5 ms |
| Radio TX Interval | 6000 ms | 6000±10 ms |
| End-to-end Latency | <50 ms | 35 ms |

### Flocking Stability

| Scenario | Centroid Distance | Min Separation | Heading Alignment |
|----------|-------------------|----------------|-------------------|
| Normal | Stable (±50mm) | >500mm | >85% |
| Replay Attack | Degraded (±200mm) | >400mm | >70% |
| Table Overflow | Critical (±500mm) | >300mm | >50% |

## Project Structure

```
drone/
├── main/
│   ├── main.cpp              # Main application and task implementations
│   ├── config.h              # System configuration parameters
│   ├── kinematics.c/h        # Physics integration and motion control
│   ├── neighbor_table.c/h    # Neighbor state management
│   ├── radio.h               # LoRa communication interface
│   └── visualization.c/h     # MQTT telemetry publishing
├── build/                    # Build output directory
├── CMakeLists.txt           # CMake configuration
├── sdkconfig                # ESP-IDF SDK configuration
├── *.txt                    # Serial log captures
├── compare_*.py             # Analysis scripts
├── ProjectPlan.ipynb        # Jupyter notebook analysis
└── README.md                # This file
```

## Troubleshooting

### LoRa Communication Issues

- Verify antenna connection and tuning
- Check SPI wiring (MOSI, MISO, SCK, NSS)
- Confirm matching frequency and spreading factor
- Monitor RSSI values in serial output

### MQTT Connection Failures

- Verify WiFi credentials in `menuconfig`
- Check MQTT broker address and port
- Ensure broker allows anonymous connections (or configure auth)
- Monitor WiFi reconnection attempts in logs

### Task Timing Violations

- Increase task stack sizes in `main.cpp`
- Reduce logging verbosity during performance tests
- Check for priority inversion or starvation
- Use `vTaskGetRunTimeStats()` for profiling

### Neighbor Table Issues

- Verify neighbor timeout values (`NEIGHBOR_TIMEOUT_MS`)
- Check sequence number wrapping logic
- Monitor packet reception rate (should be ~1 per 6 seconds per neighbor)
- Validate MAC authentication isn't dropping valid packets

## Future Enhancements

- [ ] Obstacle avoidance using ultrasonic sensors
- [ ] Dynamic leader election for coordinated maneuvers
- [ ] Adaptive transmission power based on RSSI
- [ ] Over-the-air (OTA) firmware updates
- [ ] Web-based real-time visualization dashboard
- [ ] Multi-hop routing for extended range
- [ ] Battery monitoring with low-power sleep modes

## References

- Reynolds, C. W. (1987). "Flocks, herds and schools: A distributed behavioral model"
- Olfati-Saber, R. (2006). "Flocking for multi-agent dynamic systems"
- ESP-IDF Programming Guide: https://docs.espressif.com/projects/esp-idf/
- RadioLib Documentation: https://github.com/jgromes/RadioLib
- NIST SP 800-38B: AES-CMAC Specification

## License

This project is developed for academic purposes as part of COMP0211 - System Engineering coursework.

## Authors

Charlene Chen (@charlene-chenn)

## Acknowledgments

Special thanks to the COMP0211 teaching team and lab demonstrators for guidance and support throughout this project.
