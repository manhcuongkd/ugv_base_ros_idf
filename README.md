# RaspRover IDF - ESP32-Based Robot Control System

A comprehensive ESP-IDF implementation of the Waveshare UGV base ROS robot control system, converted from the original Arduino-based project with full modular architecture.

## 🚀 Overview

ugv_base_ros_idf IDF is a complete ESP32-based robot control system featuring:

- **🎮 Motion Control**: PID-based motor control with encoder feedback and speed rate limiting
- **📡 IMU Integration**: ICM20948 9-DOF sensor with DMP support and quaternion calculations
- **🦾 Robotic Arm**: RoArm-M2 with 5 servos, inverse kinematics, and trajectory planning
- **🌐 Communication**: UART JSON commands, WiFi, ESP-NOW, HTTP REST API
- **📊 Sensors**: OLED display, battery monitoring, LED control, gimbal stabilization
- **💾 Storage**: SPIFFS file system for configuration and mission storage
- **🔧 Modular Design**: Clean separation with dedicated controller modules

## 📁 Project Structure

```
ugv_base_ros_idf/
├── CMakeLists.txt                    # Project CMake configuration
├── idf_component.yml                 # ESP-IDF component configuration
├── sdkconfig                         # ESP-IDF configuration
├── inc/                              # Header files (modular design)
│   ├── ugv_config.h                 # Configuration and pin definitions
│   ├── system_manager.h             # System initialization and management
│   ├── motion_module.h              # Motion control and motor management
│   ├── imu_controller.h             # IMU sensor control (ICM20948)
│   ├── servo_controller.h           # Servo and robotic arm control
│   ├── gimbal_controller.h          # Gimbal control and stabilization
│   ├── oled_controller.h            # OLED display control
│   ├── wifi_controller.h            # WiFi management
│   ├── http_server.h                # HTTP server and REST API
│   ├── json_parser.h                # JSON command parsing
│   ├── battery_controller.h         # Battery monitoring
│   ├── led_controller.h             # LED control and status indication
│   ├── esp_now_controller.h         # ESP-NOW communication
│   ├── files_controller.h           # File system operations
│   ├── uart_controller.h            # UART communication
│   ├── mission_system.h             # Mission execution system
│   ├── ugv_advanced.h               # Advanced UGV features
│   ├── module_handlers.h            # Module-specific handlers
│   ├── common_utils.h               # Common utility macros and validation
│   └── system_info.h                # System information and status
├── main/                             # Source files (modular implementation)
│   ├── CMakeLists.txt               # Component CMake configuration
│   ├── main.cpp                     # Main application entry point
│   ├── system_manager.cpp           # System initialization
│   ├── motion_module.cpp            # Motion control implementation
│   ├── imu_controller.cpp           # IMU sensor implementation
│   ├── servo_controller.cpp         # Servo control implementation
│   ├── gimbal_controller.cpp        # Gimbal control implementation
│   ├── oled_controller.cpp          # OLED display implementation
│   ├── wifi_controller.cpp          # WiFi management implementation
│   ├── http_server.cpp              # HTTP server implementation
│   ├── json_parser.cpp              # JSON parsing implementation
│   ├── battery_controller.cpp       # Battery monitoring implementation
│   ├── led_controller.cpp           # LED control implementation
│   ├── esp_now_controller.cpp       # ESP-NOW implementation
│   ├── files_controller.cpp         # File system implementation
│   ├── uart_controller.cpp          # UART communication implementation
│   ├── mission_system.cpp           # Mission system implementation
│   ├── ugv_advanced.cpp             # Advanced features implementation
│   ├── module_handlers.cpp          # Module handlers implementation
│   └── system_info.cpp              # System information implementation
├── build.sh                          # Build automation script
├── setup.sh                          # Automated setup script
├── install_dependencies.sh           # Dependency installation script
├── switch_idf.sh                     # ESP-IDF version switching script
├── partitions.csv                    # Partition table configuration
├── src/                              # Additional source components
└── README.md                         # This file
```

## ✨ Features

### 🏗️ Core Architecture
- **Multi-threaded Design**: FreeRTOS-based task management with dedicated cores
- **Modular Implementation**: Clean separation of concerns with well-defined interfaces
- **Real-time Control**: High-frequency motion and sensor control loops (20-100Hz)
- **Error Handling**: Comprehensive error handling and recovery mechanisms
- **Memory Management**: Efficient memory usage with proper cleanup

### 🎮 Motion Control
- **Dual Motor Control**: Independent left/right motor control with PWM
- **PID Control**: Configurable PID parameters for each motor with anti-windup
- **Encoder Feedback**: Quadrature encoder support for closed-loop control
- **Speed Rate Limiting**: Configurable speed rate multipliers (0.0-1.0)
- **ROS-style Control**: Linear and angular velocity control interface
- **Emergency Stop**: Immediate motor shutdown capability

### 📡 Sensor Integration
- **IMU (ICM20948)**: 9-DOF sensor with DMP for orientation and quaternion calculations
- **Battery Monitoring**: Voltage, current, and power monitoring with INA219
- **OLED Display**: 128x64 pixel display for real-time status information
- **LED Indicators**: Configurable RGB LED patterns for system status
- **Gimbal Stabilization**: Pan/tilt gimbal with IMU-based stabilization

### 🌐 Communication
- **UART Interface**: JSON command protocol at 115200 baud with dedicated task
- **WiFi Support**: Station and Access Point modes with auto-reconnection
- **HTTP REST API**: Complete web interface with JSON endpoints
- **ESP-NOW**: Peer-to-peer communication for multi-robot coordination
- **File System**: SPIFFS for configuration storage and mission files

### 🦾 Robotic Arm (RoArm-M2)
- **5-DOF Control**: Base, shoulder, elbow, wrist, and gripper joints
- **Inverse Kinematics**: Cartesian position control with workspace limits
- **Trajectory Planning**: Smooth motion planning with speed control
- **Force Control**: Configurable torque limits and compliance
- **UI Control**: User interface control mode for manual operation

### 🎯 Mission System
- **Mission Execution**: Automated mission playback with step-by-step execution
- **File Operations**: Mission file creation, editing, and management
- **Path Following**: Waypoint-based navigation with speed control
- **Error Recovery**: Automatic error handling and mission abort capability

## 🔧 Hardware Requirements

### ESP32 Development Board
- **Processor**: ESP32 (Dual-core, 240MHz)
- **Memory**: 520KB SRAM, 4MB+ Flash (recommended 8MB)
- **GPIO**: Sufficient GPIO pins for all peripherals
- **Power**: 5V/3.3V power supply with adequate current capacity

### Motor Control
- **Motors**: 2x DC motors with encoders (12V recommended)
- **Motor Driver**: Dual H-bridge motor driver (L298N or similar)
- **Encoders**: Quadrature encoders for feedback (1000+ PPR recommended)

### Sensors
- **IMU**: ICM20948 9-DOF sensor (I2C interface)
- **Current Sensor**: INA219 for battery monitoring (I2C interface)
- **Display**: SSD1306 OLED display (128x64, I2C interface)

### Robotic Arm
- **Servos**: 5x SCServo compatible servos (12V power)
- **Power**: 12V power supply for servos (separate from ESP32)
- **Communication**: UART interface for servo control

## 📌 Pin Configuration

### Motor Control Pins
```cpp
#define AIN1 GPIO_NUM_21      // Left motor direction 1
#define AIN2 GPIO_NUM_17      // Left motor direction 2
#define PWMA GPIO_NUM_25      // Left motor PWM
#define BIN1 GPIO_NUM_22      // Right motor direction 1
#define BIN2 GPIO_NUM_23      // Right motor direction 2
#define PWMB GPIO_NUM_26      // Right motor PWM
```

### Encoder Pins
```cpp
#define AENCA GPIO_NUM_35     // Left encoder A
#define AENCB GPIO_NUM_34     // Left encoder B
#define BENCA GPIO_NUM_27     // Right encoder A
#define BENCB GPIO_NUM_16     // Right encoder B
```

### I2C Pins
```cpp
#define S_SDA GPIO_NUM_32     // I2C data line
#define S_SCL GPIO_NUM_33     // I2C clock line
```

### Servo Control Pins
```cpp
#define SERVO_RXD GPIO_NUM_18 // Servo UART receive
#define SERVO_TXD GPIO_NUM_19 // Servo UART transmit
```

### UART Pins
```cpp
#define UART_TX GPIO_NUM_1    // UART transmit (USB)
#define UART_RX GPIO_NUM_3    // UART receive (USB)
```

## 🚀 Installation & Setup

### ESP-IDF Version Compatibility

This project supports both ESP-IDF v4.4.5 and v5.0+ with seamless switching:

- **ESP-IDF v4.4.5**: Fully tested and stable (default)
- **ESP-IDF v5.0+**: Fully tested with latest features
- **Dual Installation**: Both versions can coexist on the same system
- **Easy Switching**: Use `./switch_idf.sh` to switch between versions
- **Build Compatibility**: Same codebase works with both versions

### Prerequisites
- **ESP-IDF**: v4.4.5 or v5.0+ (both fully supported and tested)
- **CMake**: Version 3.22 or later
- **Python**: Python 3.7 or later
- **Git**: For cloning repositories

### Quick Setup (Recommended)

The project includes automated setup scripts that support both ESP-IDF v4.4.5 and v5.0+:

#### For ESP-IDF v4.4.5 (Default)
```bash
# Navigate to project directory
cd ugv_base_ros_idf

# Run the automatic setup script (installs v4.4.5 by default)
./setup.sh

# This will:
# 1. Install ESP-IDF v4.4.5 if needed
# 2. Install all dependencies
# 3. Test the initial build
```

#### For ESP-IDF v5.0+
```bash
# Navigate to project directory
cd ugv_base_ros_idf

# Set ESP-IDF version and run setup
export IDF_VERSION="v5.0"
./setup.sh

# Or use the version switcher after initial setup
./switch_idf.sh --v5
```

#### Version Switching
```bash
# Switch between ESP-IDF versions
./switch_idf.sh --v4    # Switch to ESP-IDF v4.4.5
./switch_idf.sh --v5    # Switch to ESP-IDF v5.0
./switch_idf.sh --list  # List available versions
./switch_idf.sh --current  # Show current version
```

### Manual Installation

#### Install ESP-IDF v4.4
```bash
# Clone ESP-IDF v4.4
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

#### Install ESP-IDF v5.0+
```bash
# Clone ESP-IDF v5.0 (separate directory for dual version support)
mkdir -p $HOME/esp
cd $HOME/esp
git clone --recursive https://github.com/espressif/esp-idf.git esp-idf-v5
cd esp-idf-v5
git checkout v5.0

# Install ESP-IDF v5.0
./install.sh esp32

# Add to PATH (add to ~/.bashrc for permanent access)
echo 'source $HOME/esp/esp-idf-v5/export.sh' >> ~/.bashrc
source ~/.bashrc
```

#### Install Dependencies
```bash
# Navigate to project directory
cd ugv_base_ros_idf

# Install all dependencies
./install_dependencies.sh

# Or install specific components:
./install_dependencies.sh --idf-only      # ESP-IDF dependencies only
./install_dependencies.sh --custom-only   # Custom components only
./install_dependencies.sh --system-only   # System packages only
```

### Build & Flash

#### Standard Build Process
```bash
# Set up ESP-IDF environment (if not already done)
# For ESP-IDF v4.4.5:
source $HOME/esp/esp-idf/export.sh

# For ESP-IDF v5.0+:
source $HOME/esp/esp-idf-v5/export.sh

# Or use the version switcher:
./switch_idf.sh --v4  # or --v5

# Navigate to project directory
cd ugv_base_ros_idf

# Configure the project
idf.py menuconfig

# Build the project
idf.py build

# Flash to ESP32
idf.py flash

# Monitor serial output
idf.py monitor
```

#### Using Build Scripts
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
- **Serial Configuration**: UART settings and log levels
- **WiFi Configuration**: SSID, password, and mode
- **Component Configuration**: Enable/disable features
- **Partition Table**: Flash memory layout
- **FreeRTOS**: Task priorities and stack sizes

## 📡 Usage

### JSON Commands

The system accepts JSON commands via UART, WiFi, or ESP-NOW. Commands use the format `{"T":command_id,"param1":value1,"param2":value2}`:

#### Motion Control
```json
{"T":0}                                    // Emergency stop
{"T":1,"L":0.5,"R":0.5}                   // Set motor speeds (left/right)
{"T":11,"L":164,"R":164}                  // PWM input control
{"T":13,"X":0.1,"Z":0.3}                  // ROS-style motion control (linear/angular)
{"T":2,"P":200,"I":2500,"D":0,"L":255}    // Set motor PID parameters
{"T":138,"L":1.0,"R":1.0}                 // Set speed rate multipliers
{"T":139}                                  // Get speed rate multipliers
{"T":140}                                  // Save speed rate multipliers
```

#### Robotic Arm Control
```json
{"T":100}                                  // Move to initial position
{"T":101,"joint":0,"rad":0,"spd":0,"acc":10}           // Single joint control (radians)
{"T":102,"base":0,"shoulder":0,"elbow":1.57,"hand":1.57,"spd":0,"acc":10}  // All joints control
{"T":104,"x":235,"y":0,"z":234,"t":3.14,"spd":0.25}    // Cartesian position control
{"T":1041,"x":235,"y":0,"z":234,"t":3.14}              // Direct Cartesian control
{"T":105}                                  // Get servo feedback (radians)
{"T":106,"cmd":1.57,"spd":0,"acc":0}      // End-effector hand control
{"T":107,"tor":200}                        // Set grab torque
{"T":108,"joint":3,"p":16,"i":0}          // Set joint PID parameters
{"T":109}                                  // Reset all PID parameters
{"T":144,"E":100,"Z":0,"R":0}             // Arm UI control mode
{"T":210,"cmd":1}                          // Torque control (enable/disable)
```

#### IMU & Sensors
```json
{"T":126}                                  // Get IMU data
{"T":127}                                  // IMU calibration step
{"T":128}                                  // Get IMU offset values
{"T":129,"gx":0,"gy":0,"gz":0,"ax":0,"ay":0,"az":0,"cx":0,"cy":0,"cz":0}  // Set IMU offsets
```

#### Display & LED Control
```json
{"T":3,"lineNum":0,"Text":"Hello World"}  // Set OLED text
{"T":-3}                                   // Reset OLED to default
{"T":132,"IO4":255,"IO5":255}             // LED control
```

#### Gimbal Control
```json
{"T":133,"X":45,"Y":45,"SPD":0,"ACC":0}   // Simple gimbal control
{"T":134,"X":45,"Y":45,"SX":300,"SY":300} // Move gimbal with speed
{"T":135}                                  // Stop gimbal
{"T":137,"s":1,"y":0}                      // Gimbal stabilization
{"T":141,"X":0,"Y":0,"SPD":300}           // User gimbal control
```

#### File System Operations
```json
{"T":200}                                  // Scan files
{"T":201,"name":"file.txt","content":"content"}  // Create file
{"T":202,"name":"file.txt"}                // Read file
{"T":203,"name":"file.txt"}                // Delete file
{"T":204,"name":"file.txt","content":"line"}    // Append line
{"T":205,"name":"file.txt","lineNum":3,"content":"content"}  // Insert line
{"T":206,"name":"file.txt","lineNum":3,"content":"content"}  // Replace line
{"T":207,"name":"file.txt","lineNum":3}    // Read specific line
{"T":208,"name":"file.txt","lineNum":3}    // Delete specific line
```

#### Mission System
```json
{"T":220,"name":"mission_a","intro":"description"}  // Create mission
{"T":221,"name":"mission_a"}                // Get mission content
{"T":222,"name":"mission_a","step":"{\"T\":104,\"x\":235,\"y\":0,\"z\":234,\"t\":3.14,\"spd\":0.25}"}  // Append step
{"T":223,"name":"mission_a","spd":0.25}     // Append feedback step
{"T":224,"name":"mission_a","delay":3000}   // Append delay step
{"T":225,"name":"mission_a","stepNum":3,"step":"{\"T\":104}"}  // Insert step
{"T":226,"name":"mission_a","stepNum":3,"spd":0.25}  // Insert feedback step
{"T":227,"stepNum":3,"delay":3000}         // Insert delay step
{"T":228,"name":"mission_a","stepNum":3,"step":"{\"T\":114}"}  // Replace step
{"T":229,"name":"mission_a","stepNum":3,"spd":0.25}  // Replace feedback step
{"T":230,"name":"mission_a","stepNum":3,"delay":3000}  // Replace delay step
{"T":231,"name":"mission_a","stepNum":3}    // Delete step
{"T":241,"name":"mission_a","stepNum":3}    // Move to specific step
{"T":242,"name":"mission_a","times":3}      // Play mission
```

#### ESP-NOW Communication
```json
{"T":300,"mode":1,"mac":"FF:FF:FF:FF:FF:FF"}  // Broadcast follower
{"T":301,"mode":3}                         // ESP-NOW configuration
{"T":302}                                  // Get MAC address
{"T":303,"mac":"FF:FF:FF:FF:FF:FF"}       // Add follower
{"T":304,"mac":"FF:FF:FF:FF:FF:FF"}       // Remove follower
{"T":305,"dev":0,"b":0,"s":0,"e":1.57,"h":1.57,"cmd":0,"megs":"hello!"}  // Group control
{"T":306,"mac":"FF:FF:FF:FF:FF:FF","dev":0,"b":0,"s":0,"e":1.57,"h":1.57,"cmd":0,"megs":"hello!"}  // Single control
```

#### WiFi Configuration
```json
{"T":401,"cmd":3}                          // WiFi on boot mode
{"T":402,"ssid":"RoArm-M2","password":"12345678"}  // Set Access Point
{"T":403,"ssid":"JSBZY-2.4G","password":"waveshare0755"}  // Set Station
{"T":404,"ap_ssid":"RoArm-M2","ap_password":"12345678","sta_ssid":"JSBZY-2.4G","sta_password":"waveshare0755"}  // AP+STA mode
{"T":405}                                  // Get WiFi info
{"T":406}                                  // Create config by status
{"T":407,"mode":3,"ap_ssid":"RoArm-M2","ap_password":"12345678","sta_ssid":"JSBZY-2.4G","sta_password":"waveshare0755"}  // Create config by input
{"T":408}                                  // Stop WiFi
```

#### System & Configuration
```json
{"T":4,"cmd":0}                            // Set module type
{"T":900,"main":1,"module":0}              // Set robot type and module
{"T":130}                                  // Get base feedback
{"T":131,"cmd":1}                          // Base feedback flow control
{"T":142,"cmd":0}                          // Set feedback flow interval
{"T":143,"cmd":0}                          // UART echo mode
{"T":136,"cmd":3000}                       // Set heartbeat interval
{"T":501,"raw":1,"new":11}                 // Set servo ID
{"T":502,"id":11}                          // Set servo middle position
{"T":503,"id":14,"p":16}                   // Set servo PID
{"T":600}                                  // Reboot system
{"T":601}                                  // Get free flash space
{"T":602}                                  // Get boot mission info
{"T":603}                                  // Reset boot mission
{"T":604}                                  // Clear NVS storage
{"T":605,"cmd":1}                          // Info print mode
{"T":999}                                  // Reset emergency state
```

### HTTP REST API

Access the web interface at `http://[ESP32_IP]` or use the REST API endpoints:

#### System Status & Information
```bash
GET /api/status                    # Get system status
GET /api/info                      # Get system information
GET /api/version                   # Get firmware version
```

#### Motion Control
```bash
POST /api/motion/speed
Content-Type: application/json
{"left": 0.5, "right": 0.5}

POST /api/motion/ros
Content-Type: application/json
{"linear": 0.1, "angular": 0.3}

POST /api/motion/emergency         # Emergency stop
```

#### Robotic Arm Control
```bash
POST /api/arm/joint
Content-Type: application/json
{"joint": 0, "rad": 1.57, "spd": 0.5, "acc": 10}

POST /api/arm/pose
Content-Type: application/json
{"x": 235, "y": 0, "z": 234, "t": 3.14, "spd": 0.25}

POST /api/arm/gripper
Content-Type: application/json
{"cmd": 1.57, "spd": 0.5, "acc": 10}
```

#### IMU & Sensors
```bash
GET /api/imu/data                  # Get IMU data
GET /api/imu/offset                # Get IMU offsets
POST /api/imu/calibrate            # Start IMU calibration
```

#### Display & LED Control
```bash
POST /api/display/text
Content-Type: application/json
{"line": 0, "text": "Hello World"}

POST /api/display/default          # Reset to default display

POST /api/led
Content-Type: application/json
{"io4": 255, "io5": 255}
```

#### File System Operations
```bash
GET /api/files                     # List all files
POST /api/files/create
Content-Type: application/json
{"name": "file.txt", "content": "content"}

GET /api/files/{filename}          # Read file content
DELETE /api/files/{filename}       # Delete file
```

#### Mission System
```bash
GET /api/missions                  # List all missions
POST /api/missions/create
Content-Type: application/json
{"name": "mission_a", "intro": "description"}

GET /api/missions/{name}           # Get mission content
POST /api/missions/{name}/play
Content-Type: application/json
{"times": 3}
```

#### WiFi Configuration
```bash
GET /api/wifi/info                 # Get WiFi information
POST /api/wifi/ap
Content-Type: application/json
{"ssid": "RoArm-M2", "password": "12345678"}

POST /api/wifi/sta
Content-Type: application/json
{"ssid": "MyNetwork", "password": "password"}
```

#### System Configuration
```bash
POST /api/config/robot-type
Content-Type: application/json
{"main": 1, "module": 0}

POST /api/config/pid
Content-Type: application/json
{"p": 200, "i": 2500, "d": 0, "limit": 255}

POST /api/system/reboot            # Reboot system
GET /api/system/flash-space        # Get free flash space
```


## 🏗️ Development

### Architecture

The system uses a multi-threaded architecture with the following tasks:

1. **Main Task** (Core 0, Priority 5): System coordination and command processing
2. **IMU Task** (Core 1, Priority 4): Sensor data acquisition and processing
3. **Motion Task** (Core 0, Priority 3): Motor control and PID computation
4. **UART Task** (Core 0, Priority 4): UART communication and command parsing

### Task Communication

- **Queues**: Inter-task communication using FreeRTOS queues
- **Shared Memory**: Protected access to global data structures
- **Semaphores**: Synchronization for critical sections
- **Timers**: Periodic task execution and timeouts

### Adding New Features

1. **Create Header File**: Define interfaces in `inc/` directory
2. **Implement Functions**: Add implementation in `main/` directory
3. **Update CMakeLists.txt**: Add new source files to build
4. **Update Main Loop**: Integrate with main application flow
5. **Add Configuration**: Include in configuration structures

### Code Style

- **C++17 Standard**: Use modern C++ features
- **ESP-IDF APIs**: Leverage ESP-IDF components and patterns
- **Error Handling**: Use ESP_OK/ESP_ERR return codes consistently
- **Documentation**: Include Doxygen-style comments
- **Memory Management**: Proper allocation and cleanup

## 🔍 Troubleshooting

### Common Issues

1. **Build Errors**: 
   - Ensure ESP-IDF is properly installed and sourced
   - Check CMake version compatibility
   - Verify all dependencies are installed
   - Use `./switch_idf.sh --current` to verify active ESP-IDF version

2. **ESP-IDF Version Issues**:
   - Use `./switch_idf.sh --list` to see available versions
   - Ensure correct version is active: `./switch_idf.sh --v4` or `--v5`
   - Check Python environment compatibility for ESP-IDF v5.0+

3. **Flash Failures**: 
   - Check USB connection and driver installation
   - Verify correct port selection
   - Try different USB cable or port

4. **Runtime Errors**: 
   - Monitor serial output for error messages
   - Check log levels in menuconfig
   - Verify hardware connections

5. **WiFi Issues**: 
   - Verify SSID and password configuration
   - Check WiFi signal strength
   - Ensure correct WiFi mode (STA/AP)

### Debug Information

Enable debug output by setting log levels in `menuconfig`:
- **Component Log Level**: Set to DEBUG for detailed output
- **Serial Output**: Ensure UART is configured correctly
- **FreeRTOS**: Enable task monitoring and stack overflow detection

### Performance Optimization

- **Task Priorities**: Adjust task priorities based on requirements
- **Stack Sizes**: Monitor stack usage and adjust if needed
- **Memory Usage**: Use heap monitoring tools to track memory usage
- **CPU Usage**: Monitor CPU usage across cores

### Completed Features ✅
- ✅ Core motion control with PID
- ✅ IMU integration with DMP
- ✅ Robotic arm control with inverse kinematics
- ✅ UART communication with JSON parsing
- ✅ WiFi and HTTP server
- ✅ ESP-NOW communication
- ✅ OLED display control
- ✅ Battery monitoring
- ✅ LED control
- ✅ File system operations
- ✅ Mission system
- ✅ Gimbal control
- ✅ Modular architecture
- ✅ Speed rate limiting
- ✅ Emergency stop functionality

### In Progress 🔄
- 🔄 Advanced mission features
- 🔄 Enhanced error recovery
- 🔄 Performance optimization

## 🤝 Contributing

1. **Fork the Repository**: Create your own fork
2. **Create Feature Branch**: Work on new features in separate branches
3. **Follow Code Style**: Maintain consistent code style and documentation
4. **Test Thoroughly**: Ensure all features work correctly
5. **Submit Pull Request**: Include detailed description of changes

## 📄 License

This project is licensed under the GNU General Public License v3.0. See the LICENSE file for details.

## 🙏 Acknowledgments

- **Waveshare**: Original UGV base ROS project
- **ESP-IDF**: Espressif IoT Development Framework
- **Open Source Community**: Various libraries and components
- **Contributors**: All developers who have contributed to this project

## 📞 Support

For issues and questions:
1. Check the troubleshooting section
2. Review ESP-IDF documentation
3. Search existing issues
4. Create new issue with detailed information

## 📈 Version History

- **v2.1.0**: ESP-IDF v5.0 Support & Arduino Logic Integration
  - Full ESP-IDF v5.0 compatibility with dual version support
  - Complete Arduino-based logic implementation for all modules
  - Enhanced motion control with quadrature encoder ISR
  - Improved servo control with UART communication simulation
  - Arduino-style OLED display modes and periodic updates
  - IMU data availability timing-based checks
  - Version switching system with automated setup scripts
  - All "in a real implementation" placeholders replaced with proper logic

- **v2.0.0**: Complete modular refactoring
  - Modular architecture with separate source files
  - UART task integration
  - Speed rate limiting implementation
  - Enhanced error handling
  - Complete feature implementation

- **v1.0.0**: Initial C++ IDF implementation
  - Core motion control
  - IMU integration
  - Basic web interface
  - JSON command protocol

---

**Note**: This project is a complete conversion from the Arduino-based UGV base ROS project with full ESP-IDF implementation and modular architecture.
