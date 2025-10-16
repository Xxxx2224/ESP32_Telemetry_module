| Supported Targets | ESP32 | ESP32-C2 | ESP32-C3 | ESP32-C6 | ESP32-H2 | ESP32-P4 | ESP32-S2 | ESP32-S3 |
| ----------------- | ----- | -------- | -------- | -------- | -------- | -------- | -------- | -------- |

# ESP32 Telemetry Module

A comprehensive ESP32-based telemetry system designed for real-time data transmission and monitoring of unmanned vehicles using the MAVLink protocol. This module provides wireless telemetry capabilities with WiFi access point functionality, GPS integration, and JSON-based data formatting.

## Overview

This project implements a telemetry module on ESP32 microcontrollers that:
- Creates a WiFi Access Point for wireless communication
- Receives and parses MAVLink telemetry data via UART
- Processes GPS coordinates and vehicle positioning data
- Transmits telemetry data over TCP/IP sockets
- Supports multiple telemetry data formats including JSON

The module is designed to work with autopilot systems and flight controllers that use the MAVLink protocol (such as ArduPilot, PX4, etc.).

## Features

### Core Features
- **WiFi Access Point**: Creates a wireless network for ground station connectivity
- **MAVLink Protocol Support**: Full MAVLink v2.0 protocol implementation
- **UART Communication**: RS-485 half-duplex UART interface for telemetry reception
- **TCP/IP Socket Server**: Network server for real-time data streaming
- **GPS Integration**: TinyGPS library support for position tracking
- **JSON Data Formatting**: Structured telemetry data output using cJSON library
- **FreeRTOS Tasks**: Multi-threaded architecture for concurrent operations

### Supported Telemetry Data
- **Heartbeat Messages**: Vehicle status and mode information
- **GPS Position**: Latitude, longitude, altitude, and fix quality
- **Attitude Data**: Roll, pitch, yaw angles
- **Battery Status**: Voltage and remaining capacity
- **System Status**: Flight mode, armed state, and system health
- **Speed and Navigation**: Ground speed and heading information

## Hardware Requirements

### Required Components
- ESP32 development board (ESP32-S3 recommended, supports all ESP32 variants)
- UART-to-RS485 converter (for autopilot connection)
- GPS module (optional, for independent position tracking)

### Pin Configuration

> **Important**: The MAVLink and GPS UART configurations use overlapping pins. The project includes separate implementations for each use case:
> - Use `main.c`, `main1.c`, `main2.c`, or `main3.c` for MAVLink telemetry
> - Use `gps.c` for GPS-only functionality
> - For simultaneous use, reconfigure one interface to use different GPIO pins

#### UART2 (MAVLink Communication)
- **TX Pin**: GPIO 4
- **RX Pin**: GPIO 5
- **RTS Pin**: GPIO 17
- **CTS Pin**: GPIO 18
- **Baud Rate**: 57600 bps (configurable)
- **Mode**: RS-485 Half-Duplex

#### GPS UART (Optional - used in gps.c variant)
> **Note**: The GPS variant uses swapped TX/RX pins compared to MAVLink. Use `gps.c` implementation when GPS is needed instead of MAVLink communication, or reconfigure pins to avoid conflicts.
- **TX Pin**: GPIO 5 (conflicts with MAVLink RX)
- **RX Pin**: GPIO 4 (conflicts with MAVLink TX)
- **Baud Rate**: 9600 bps

## Software Dependencies

### ESP-IDF
This project requires Espressif ESP-IDF framework:
- **Minimum Version**: ESP-IDF v4.4 or higher
- **Recommended**: ESP-IDF v5.0+

### Libraries
- **FreeRTOS**: Task management (included in ESP-IDF)
- **MAVLink**: Telemetry protocol (headers included in project)
- **cJSON**: JSON parsing and generation
- **TinyGPS**: GPS data parsing (for GPS variant)
- **lwIP**: TCP/IP stack (included in ESP-IDF)

## Build and Installation

### Prerequisites
1. Install ESP-IDF following the [official guide](https://docs.espressif.com/projects/esp-idf/en/latest/esp32/get-started/)
2. Set up the ESP-IDF environment:
   ```bash
   . $HOME/esp/esp-idf/export.sh
   ```

### Building the Project
```bash
# Navigate to project directory
cd ESP32_Telemetry_module

# Configure the project (optional - use default settings)
idf.py menuconfig

# Build the project
idf.py build

# Flash to ESP32
idf.py -p /dev/ttyUSB0 flash

# Monitor serial output
idf.py -p /dev/ttyUSB0 monitor
```

## Configuration

### WiFi Settings
Edit the following defines in `main/main.c` or `main/main1.c`:
```c
#define SSID "ESP32S3"              // WiFi network name - Change to unique identifier
#define PASSWORD "YourSecurePass"   // WiFi password - Change to strong password
```

> **Security Recommendations**:
> - Change the default SSID to a unique identifier to avoid conflicts (e.g., "MyDrone_Telemetry")
> - Use a strong password with at least 12 characters including:
>   - Uppercase and lowercase letters
>   - Numbers
>   - Special characters (!@#$%^&*)
> - Never use default or example passwords in production
> - Example format: `"MyDr0ne!T3l3m#2024"` (create your own unique password)

### Network Configuration
- **Default IP Address**: 192.168.4.1
- **TCP Server Port**: 8000
- **Max Connections**: 3 simultaneous clients

### UART Configuration
- **Baud Rate**: 57600 bps (MAVLink standard)
- **Data Bits**: 8
- **Parity**: None
- **Stop Bits**: 1
- **Flow Control**: RTS/CTS enabled

## Usage

### Connecting to the Module
1. Power on the ESP32 module
2. Connect to the WiFi network (SSID: ESP32S3)
3. Use TCP client to connect to `192.168.4.1:8000`
4. The module will stream telemetry data in real-time

### Ground Station Connection
The module can be used with:
- **Mission Planner** (Windows)
- **QGroundControl** (Cross-platform)
- **MAVProxy** (Command-line tool)
- Custom ground station applications

Configure the ground station to use TCP connection with the module's IP address and port.

## Project Structure

```
ESP32_Telemetry_module/
├── CMakeLists.txt              # Project build configuration
├── sdkconfig                   # ESP-IDF configuration
├── README.md                   # This file
└── main/
    ├── CMakeLists.txt          # Main component configuration
    ├── main.c                  # Main telemetry application
    ├── main1.c                 # Alternative implementation with client send
    ├── main2.c                 # JSON-based telemetry variant
    ├── main3.c                 # Advanced JSON with mutex protection
    ├── gps.c                   # GPS module integration
    ├── task.c                  # FreeRTOS task examples
    ├── Queue1.c                # Queue communication examples
    ├── semaphor.c              # Semaphore examples
    ├── struct.c                # Data structure examples
    ├── paralel.c               # Parallel task examples
    └── nparalel.c              # Non-parallel task examples
```

### Main Variants
- **main.c**: Basic WiFi AP with MAVLink UART reception and socket server
- **main1.c**: Adds client transmission of heartbeat data
- **main2.c**: JSON formatting of telemetry data with multiple message types
- **main3.c**: Enhanced version with mutex protection and extended telemetry support

## Telemetry Data Format

### MAVLink Messages Supported
- `HEARTBEAT` (ID: 0): System status and mode
- `GPS_RAW_INT` (ID: 24): Raw GPS data
- `ATTITUDE` (ID: 30): Vehicle attitude
- `GLOBAL_POSITION_INT` (ID: 33): Global position
- `SYS_STATUS` (ID: 1): System status and battery
- `VFR_HUD` (ID: 74): HUD information

### JSON Output Example
```json
{
  "id": 1,
  "type": 2,
  "autopilot": 3,
  "base_mode": 81,
  "custom_mode": 0,
  "system_status": 4,
  "mavlink_version": 3,
  "battery_voltage": 12600,
  "battery_remaining": 95,
  "lat": 473977420,
  "lon": -1223748200,
  "alt": 15000,
  "speed": 50,
  "roll": 0.05,
  "pitch": 0.02,
  "yaw": 1.57,
  "satellites_visible": 12,
  "fix_type": 3
}
```

## Troubleshooting

### Common Issues
1. **WiFi AP not visible**: Check SSID configuration and antenna connection
2. **No UART data received**: Verify baud rate (57600) and pin connections
3. **Socket connection fails**: Ensure correct IP (192.168.4.1) and port (8000)
4. **MAVLink parsing errors**: Confirm MAVLink version compatibility

### Debug Logging
Enable ESP-IDF logging for diagnostics:
```bash
idf.py menuconfig
# Component config → Log output → Set log level to Debug
```

## Development Notes

### Adding New Message Types
To support additional MAVLink messages:
1. Include the message header from MAVLink library
2. Add case in the `uart_mavlink()` message switch
3. Decode the message using `mavlink_msg_<message>_decode()`
4. Update JSON structure if using JSON output

### Performance Considerations
- UART buffer size: 2048 bytes (configurable)
- Queue sizes optimized for real-time operation
- Task priorities set for telemetry precedence
- Socket timeouts configured for reliable transmission

## License

This project is provided as-is for educational and research purposes.

## Contributing

Contributions are welcome! Please ensure:
- Code follows ESP-IDF coding standards
- All changes are tested on hardware
- Comments are clear and descriptive
- Commit messages are meaningful

## Support

For issues and questions:
- Check ESP-IDF documentation: https://docs.espressif.com/projects/esp-idf/
- MAVLink protocol: https://mavlink.io/
- ESP32 forums: https://esp32.com/

## Acknowledgments

- Espressif Systems for ESP-IDF framework
- MAVLink project for the telemetry protocol
- FreeRTOS for the real-time operating system
- TinyGPS library for GPS parsing
