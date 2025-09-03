# RaspRover IDF - ESP32-Based Robot Control System

A pure C++ ESP-IDF implementation of the Waveshare UGV base ROS robot control system, converted from the original Arduino-based project.

## Overview

RaspRover IDF is a comprehensive ESP32-based robot control system that provides:

- **Motion Control**: PID-based motor control with encoder feedback
- **IMU Integration**: ICM20948 9-DOF sensor with DMP support
- **Robotic Arm**: RoArm-M2 with 5 servos and inverse kinematics
- **Communication**: UART JSON commands, WiFi, ESP-NOW, HTTP server
- **Sensors**: OLED display, battery monitoring, LED control
- **File System**: SPIFFS for configuration storage

## Project Structure

```
RaspRover/
├── CMakeLists.txt              # Project CMake configuration
├── main/                       # Main application source
│   ├── CMakeLists.txt         # Component CMake configuration
│   ├── RaspRover.cpp          # Main application source (C++)
│   ├── ugv_config.h           # Configuration and pin definitions
│   ├── motion_module.h        # Motion control and motor management
│   ├── imu_controller.h       # IMU sensor control (ICM20948)
│   ├── servo_controller.h     # Servo and robotic arm control
│   ├── oled_controller.h      # OLED display control
│   ├── wifi_controller.h      # WiFi management
│   ├── http_server.h          # HTTP server and web interface
│   ├── json_parser.h          # JSON command parsing
│   ├── battery_controller.h   # Battery monitoring
│   └── led_controller.h       # LED control and status indication
├── components/                 # External components
│   ├── Adafruit_ICM20X/      # ICM20X sensor library
│   ├── Adafruit_SSD1306/     # SSD1306 OLED library
│   ├── ArduinoJson/           # JSON parsing library
│   ├── ESP32Encoder/          # Encoder library
│   ├── ICM20948_WE/           # ICM20948 sensor library
│   ├── INA219_WE/             # INA219 current sensor library
│   ├── PID_v2/                # PID control library
│   ├── SCServo/               # Servo control library
│   └── SimpleKalmanFilter/    # Kalman filter library
└── README.md                  # This file
```

## Features

### Core Functionality
- **Multi-threaded Architecture**: FreeRTOS-based task management
- **Real-time Control**: High-frequency motion and sensor control loops
- **Modular Design**: Clean separation of concerns with well-defined interfaces
- **Error Handling**: Comprehensive error handling and recovery mechanisms

### Motion Control
- **Dual Motor Control**: Independent left/right motor control
- **PID Control**: Configurable PID parameters for each motor
- **Encoder Feedback**: Quadrature encoder support for closed-loop control
- **Speed Control**: Linear and angular velocity control (ROS-style)

### Sensor Integration
- **IMU (ICM20948)**: 9-DOF sensor with DMP for orientation
- **Battery Monitoring**: Voltage, current, and temperature monitoring
- **OLED Display**: 128x64 pixel display for status information
- **LED Indicators**: Configurable LED patterns for system status

### Communication
- **UART Interface**: JSON command protocol at 115200 baud
- **WiFi Support**: Station and Access Point modes
- **HTTP Server**: Web interface for remote control
- **ESP-NOW**: Peer-to-peer communication

### Robotic Arm (RoArm-M2)
- **5-DOF Control**: Base, shoulder, elbow, and gripper joints
- **Inverse Kinematics**: Cartesian position control
- **Trajectory Planning**: Smooth motion planning
- **Force Control**: Configurable torque limits

## Hardware Requirements

### ESP32 Development Board
- **Processor**: ESP32 (Dual-core, 240MHz)
- **Memory**: 520KB SRAM, 4MB Flash
- **GPIO**: Sufficient GPIO pins for all peripherals

### Motor Control
- **Motors**: 2x DC motors with encoders
- **Motor Driver**: Dual H-bridge motor driver
- **Encoders**: Quadrature encoders for feedback

### Sensors
- **IMU**: ICM20948 9-DOF sensor
- **Current Sensor**: INA219 for battery monitoring
- **Display**: SSD1306 OLED display (128x64)

### Robotic Arm
- **Servos**: 5x SCServo compatible servos
- **Power**: 12V power supply for servos
- **Communication**: UART interface for servo control

## Pin Configuration

### Motor Control Pins
```cpp
#define AIN1 GPIO_NUM_32      // Left motor direction 1
#define AIN2 GPIO_NUM_33      // Left motor direction 2
#define PWMA GPIO_NUM_25      // Left motor PWM
#define BIN1 GPIO_NUM_26      // Right motor direction 1
#define BIN2 GPIO_NUM_27      // Right motor direction 2
#define PWMB GPIO_NUM_14      // Right motor PWM
```

### Encoder Pins
```cpp
#define LEFT_ENCODER_A GPIO_NUM_34
#define LEFT_ENCODER_B GPIO_NUM_35
#define RIGHT_ENCODER_A GPIO_NUM_36
#define RIGHT_ENCODER_B GPIO_NUM_39
```

### I2C Pins
```cpp
#define S_SDA GPIO_NUM_21     // I2C data line
#define S_SCL GPIO_NUM_22     // I2C clock line
```

### Servo Control Pins
```cpp
#define SERVO_RXD GPIO_NUM_18 // Servo UART receive
#define SERVO_TXD GPIO_NUM_19 // Servo UART transmit
```

## Building the Project

### Prerequisites
1. **ESP-IDF**: Install ESP-IDF v4.4 or later
2. **CMake**: Version 3.22 or later
3. **Python**: Python 3.7 or later

### Quick Setup (Recommended)
```bash
# Navigate to project directory
cd RaspRover

# Run the automatic setup script
./setup.sh

# This will:
# 1. Install ESP-IDF if needed
# 2. Install all dependencies
# 3. Test the initial build
```

### Manual Installation

#### Install ESP-IDF
```bash
# Clone ESP-IDF
mkdir -p $HOME/esp
cd $HOME/esp
git clone --recursive https://github.com/espressif/esp-idf.git
cd esp-idf
git checkout v4.4.5

# Install ESP-IDF
./install.sh esp32

# Add to PATH (add to ~/.bashrc for permanent access)
echo 'source $HOME/esp/esp-idf/export.sh' >> ~/.bashrc
source ~/.bashrc
```

#### Install Dependencies
```bash
# Navigate to project directory
cd RaspRover

# Install all dependencies
./install_dependencies.sh

# Or install specific components:
./install_dependencies.sh --idf-only      # ESP-IDF dependencies only
./install_dependencies.sh --custom-only   # Custom components only
./install_dependencies.sh --system-only   # System packages only
```

### Build Steps
```bash
# Set up ESP-IDF environment (if not already done)
source $HOME/esp/esp-idf/export.sh

# Navigate to project directory
cd RaspRover

# Configure the project
idf.py menuconfig

# Build the project
idf.py build

# Flash to ESP32
idf.py flash

# Monitor serial output
idf.py monitor
```

### Using Build Scripts
```bash
# Build the project
./build.sh build

# Flash to ESP32 (auto-detect port)
./build.sh flash

# Flash to specific port
./build.sh flash /dev/ttyUSB0

# Monitor serial output
./build.sh monitor

# Build, flash, and monitor in one command
./build.sh all /dev/ttyUSB0
```

### Configuration
Use `idf.py menuconfig` to configure:
- **Serial Configuration**: UART settings
- **WiFi Configuration**: SSID, password, and mode
- **Component Configuration**: Enable/disable features
- **Partition Table**: Flash memory layout

## Usage

### JSON Commands

The system accepts JSON commands via UART, WiFi, or ESP-NOW:

#### Motion Control
```json
{"T":1,"L":0.5,"R":0.5}           // Set motor speeds
{"T":13,"X":0.1,"Z":0.3}         // ROS-style motion control
{"T":0}                           // Emergency stop
```

#### Configuration
```json
{"T":900,"main":1,"module":2}     // Set robot type and module
{"T":2,"P":200,"I":2500,"D":0,"L":255}  // Set PID parameters
```

#### Display Control
```json
{"T":3,"lineNum":0,"Text":"Hello"} // Set OLED text
```

### Web Interface

Access the web interface at `http://[ESP32_IP]` to:
- View system status
- Control robot motion
- Configure parameters
- Monitor sensors

### API Endpoints

- `GET /api/status` - System status
- `POST /api/control` - Motion control
- `GET /api/sensor` - Sensor data
- `GET /api/config` - Configuration

## Development

### Architecture

The system uses a multi-threaded architecture with the following tasks:

1. **Main Task** (Core 0): System coordination and command processing
2. **IMU Task** (Core 1): Sensor data acquisition and processing
3. **Motion Task** (Core 0): Motor control and PID computation

### Adding New Features

1. **Create Header File**: Define interfaces in appropriate header
2. **Implement Functions**: Add implementation in source files
3. **Update Main Loop**: Integrate with main application flow
4. **Add Configuration**: Include in configuration structures

### Code Style

- **C++17 Standard**: Use modern C++ features
- **ESP-IDF APIs**: Leverage ESP-IDF components
- **Error Handling**: Use ESP_OK/ESP_ERR return codes
- **Documentation**: Include Doxygen-style comments

## Troubleshooting

### Common Issues

1. **Build Errors**: Ensure ESP-IDF is properly installed
2. **Flash Failures**: Check USB connection and board selection
3. **Runtime Errors**: Monitor serial output for error messages
4. **WiFi Issues**: Verify SSID and password configuration

### Debug Information

Enable debug output by setting log levels in `menuconfig`:
- **Component Log Level**: Set to DEBUG for detailed output
- **Serial Output**: Ensure UART is configured correctly

## Contributing

1. **Fork the Repository**: Create your own fork
2. **Create Feature Branch**: Work on new features in separate branches
3. **Submit Pull Request**: Include detailed description of changes
4. **Follow Style Guide**: Maintain consistent code style

## License

This project is licensed under the GNU General Public License v3.0. See the LICENSE file for details.

## Acknowledgments

- **Waveshare**: Original UGV base ROS project
- **ESP-IDF**: Espressif IoT Development Framework
- **Open Source Community**: Various libraries and components

## Support

For issues and questions:
1. Check the troubleshooting section
2. Review ESP-IDF documentation
3. Search existing issues
4. Create new issue with detailed information

## Version History

- **v1.0.0**: Initial C++ IDF implementation
  - Core motion control
  - IMU integration
  - Basic web interface
  - JSON command protocol

---

**Note**: This is a work-in-progress conversion from the Arduino-based UGV base ROS project. Some features may not be fully implemented yet.
