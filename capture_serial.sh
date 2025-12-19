#!/usr/bin/env bash
set -euo pipefail

# Source ESP-IDF environment
export IDF_PATH="/Users/charlenechen/esp/v5.5.1/esp-idf"
source "${IDF_PATH}/export.sh" > /dev/null 2>&1

# ESP32 Serial Monitor Logger
# Captures ESP-IDF monitor output with timestamps to log files
# Usage:
#   ./capture_serial.sh                                    # uses default port
#   ./capture_serial.sh /dev/cu.wchusbserial591B0089921   # explicit port
#   ./capture_serial.sh /dev/cu.wchusbserial591B0089921 drone_log  # custom base filename

PORT="${1:-}"
BASE_FILENAME="${2:-drone_log}"
BAUD="${BAUD:-115200}"
TS_FORMAT="${TS_FORMAT:-absolute}" # absolute | relative | iso | none

timestamp() {
  date +"%Y-%m-%d %H:%M:%S"
}

# Create output files
ALL_LOG="${BASE_FILENAME}_all.txt"
COMMS_LOG="${BASE_FILENAME}_comms.txt"
METRICS_LOG="${BASE_FILENAME}_metrics.txt"

echo "ESP32 Serial Monitor Logger"
echo "=========================="
echo "Port: ${PORT:-<idf default>}"
echo "Baud: ${BAUD}"
echo "Timestamp format: ${TS_FORMAT}"
echo "Logging to:"
echo "  - All output: ${ALL_LOG}"
echo "  - TX/RX only: ${COMMS_LOG}" 
echo "  - Metrics only: ${METRICS_LOG}"
echo ""
echo "Press Ctrl+C to stop logging"
echo ""

# Write session header to all log files
{
  echo "==== Monitor session start $(timestamp) ===="
  echo "Port: ${PORT:-<idf default>}"
  echo "Baud: ${BAUD}"
  echo "Timestamp format: ${TS_FORMAT}"
  echo "===================================================="
} | tee -a "${ALL_LOG}" "${COMMS_LOG}" "${METRICS_LOG}" > /dev/null

# Build idf.py command
if [[ -n "${PORT}" ]]; then
  cmd=(idf.py -p "${PORT}" -b "${BAUD}" monitor --timestamps --timestamp-format "${TS_FORMAT}")
else
  cmd=(idf.py -b "${BAUD}" monitor --timestamps --timestamp-format "${TS_FORMAT}")
fi

# Run monitor and process output
"${cmd[@]}" | while IFS= read -r line; do
  # Always log to all file
  echo "$line" >> "${ALL_LOG}"
  
  # Display on console with color coding
  if [[ "$line" == *"TX:"* ]] || [[ "$line" == *"RX:"* ]]; then
    # Communication logs - blue
    echo -e "\033[34m[COMM]\033[0m $line"
    echo "$line" >> "${COMMS_LOG}"
  elif [[ "$line" == *"METRIC_"* ]]; then
    # Metric logs - green  
    echo -e "\033[32m[METRIC]\033[0m $line"
    echo "$line" >> "${METRICS_LOG}"
  else
    # Other logs - default color
    echo -e "\033[37m[OTHER]\033[0m $line"
  fi
done

# Write session footer
{
  echo ""
  echo "==== Monitor session end $(timestamp) ===="
} | tee -a "${ALL_LOG}" "${COMMS_LOG}" "${METRICS_LOG}" > /dev/null

echo ""
echo "Logs saved to:"
echo "  - All: ${ALL_LOG}"
echo "  - Communications: ${COMMS_LOG}"
echo "  - Metrics: ${METRICS_LOG}"