import serial
import sys
from datetime import datetime
import time

def list_ports():
    """Lists serial port names."""
    from serial.tools.list_ports import comports
    ports = comports()
    print("Available serial ports:")
    if not ports:
        print("  No serial ports found.")
    for port, desc, hwid in sorted(ports):
        print(f"  {port}: {desc} [{hwid}]")

def main():
    """Main function to read from serial and log to separate files."""
    list_ports()
    print("-" * 20)

    try:
        port_name = input("Enter the serial port name (e.g., /dev/tty.usbserial-XXXX or COM3): ")
        base_filename = input("Enter the base filename (e.g., drone_log): ")
    except (EOFError, KeyboardInterrupt):
        print("\nExiting.")
        sys.exit(0)

    # Create separate filenames
    all_logs_file = f"{base_filename}_all.txt"
    comms_file = f"{base_filename}_comms.txt" 
    metrics_file = f"{base_filename}_metrics.txt"
    
    baud_rate = 115200

    def connect_serial():
        """Try to connect to serial port with retry logic."""
        max_retries = 5
        retry_delay = 2.0
        
        for attempt in range(max_retries):
            try:
                # Use shorter timeout without exclusive access to allow sharing
                ser = serial.Serial(port_name, baud_rate, timeout=0.5)
                print(f"Connected to {port_name} at {baud_rate} baud.")
                return ser
            except serial.SerialException as e:
                if attempt < max_retries - 1:
                    print(f"Connection attempt {attempt + 1} failed: {e}")
                    print(f"Retrying in {retry_delay} seconds...")
                    time.sleep(retry_delay)
                else:
                    print(f"Error: Could not open port {port_name} after {max_retries} attempts. {e}")
                    sys.exit(1)
    
    ser = connect_serial()
    print(f"Logging to:")
    print(f"  - All logs: '{all_logs_file}'")
    print(f"  - TX/RX only: '{comms_file}'")
    print(f"  - Metrics only: '{metrics_file}'")
    print("Press Ctrl+C to stop.")

    try:
        with open(all_logs_file, 'w', encoding='utf-8') as f_all, \
             open(comms_file, 'w', encoding='utf-8') as f_comms, \
             open(metrics_file, 'w', encoding='utf-8') as f_metrics:
            
            consecutive_failures = 0
            max_consecutive_failures = 10
            line_fragment_buffer = ""  # Buffer to reconstruct fragmented lines
            
            while True:
                try:
                    # Check if serial port is still open
                    if not ser.is_open:
                        print("Serial port closed, attempting to reconnect...")
                        ser = connect_serial()
                        consecutive_failures = 0
                        continue
                    
                    # Try to read available bytes instead of waiting for complete lines
                    if ser.in_waiting > 0:
                        # Read available data
                        data = ser.read(ser.in_waiting)
                        if data:
                            try:
                                decoded_data = data.decode('utf-8', errors='replace')
                                line_fragment_buffer += decoded_data
                                
                                # Process complete lines from buffer
                                while '\n' in line_fragment_buffer:
                                    line, line_fragment_buffer = line_fragment_buffer.split('\n', 1)
                                    line = line.strip()
                                    
                                    # Skip empty lines
                                    if not line:
                                        continue
                                    
                                    # Try to identify and fix common ESP32 log patterns
                                    # Look for timestamp pattern and try to validate
                                    if '(' in line and ')' in line:
                                        # This might be a valid ESP32 log line
                                        pass
                                    elif len(line) < 8:
                                        # Very short fragment, likely corrupted
                                        continue
                                    
                                    # Only process lines that seem reasonably complete
                                    if len(line) > 5:
                                        timestamp = datetime.now().strftime('%Y-%m-%d %H:%M:%S.%f')[:-3]
                                        log_entry = f"[{timestamp}] {line}"
                                        
                                        # Always write to all logs file
                                        f_all.write(log_entry + '\n')
                                        f_all.flush()
                                        
                                        # Categorize and write to specific files
                                        if "TX:" in line or "RX:" in line:
                                            # Communication logs (TX/RX messages)
                                            f_comms.write(log_entry + '\n')
                                            f_comms.flush()
                                            print(f"[COMM] {log_entry}")
                                        elif "METRIC_" in line:
                                            # Metric logs (performance data)
                                            f_metrics.write(log_entry + '\n') 
                                            f_metrics.flush()
                                            print(f"[METRIC] {log_entry}")
                                        else:
                                            # Other logs (debug, info, etc.)
                                            print(f"[OTHER] {log_entry}")
                                
                                # Clear buffer if it gets too large (prevent memory issues)
                                if len(line_fragment_buffer) > 2000:
                                    line_fragment_buffer = ""
                                
                                consecutive_failures = 0  # Reset failure count on successful read
                            
                            except UnicodeDecodeError as e:
                                print(f"Warning: Could not decode serial data: {e}")
                                line_fragment_buffer = ""  # Clear buffer on decode error
                                consecutive_failures += 1
                    else:
                        # No data available, small delay to prevent busy waiting
                        time.sleep(0.02)
                                
                except serial.SerialException as e:
                    error_msg = str(e).lower()
                    
                    # Don't count "readiness to read but no data" as serious failures
                    if "readiness to read but returned no data" in error_msg:
                        # This is a common ESP32 issue, just wait and continue
                        time.sleep(0.05)
                        continue
                    
                    consecutive_failures += 1
                    print(f"Serial error ({consecutive_failures}/{max_consecutive_failures}): {e}")
                    
                    if consecutive_failures >= max_consecutive_failures:
                        print("Too many consecutive failures. Attempting to reconnect...")
                        try:
                            ser.close()
                        except:
                            pass
                        ser = connect_serial()
                        consecutive_failures = 0
                    else:
                        time.sleep(0.5)  # Wait before retrying
                        
                except Exception as e:
                    print(f"An unexpected error occurred during serial read: {e}")
                    consecutive_failures += 1
                    if consecutive_failures >= max_consecutive_failures:
                        break
                    time.sleep(0.1)  # Wait before retrying on unexpected errors
    except KeyboardInterrupt:
        print("\nStopping logger...")
    except IOError as e:
        print(f"Error writing to log files: {e}")
    except Exception as e:
        print(f"An unexpected error occurred: {e}")
    finally:
        if ser.is_open:
            ser.close()
            print("Serial port closed.")
        print(f"Logs saved to:")
        print(f"  - All: '{all_logs_file}'")
        print(f"  - Communications: '{comms_file}'")
        print(f"  - Metrics: '{metrics_file}'")

if __name__ == '__main__':
    print("Serial Port Logger")
    print("This script reads data from a serial port and saves it to a file.")
    print("Please ensure you have pyserial installed: `pip install pyserial`")
    print("-" * 20)
    main()
