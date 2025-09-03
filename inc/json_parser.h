#ifndef JSON_PARSER_H
#define JSON_PARSER_H

#include <esp_err.h>
#include <stdint.h>
#include <stdbool.h>
#include <cJSON.h>
#include "ugv_config.h"

#ifdef __cplusplus
extern "C" {
#endif

// JSON command structure
typedef struct {
    uint16_t type;         // Command type
    union {
        // Speed control command
        struct {
            float left_speed;      // Left motor speed
            float right_speed;     // Right motor speed
        } speed_ctrl;
        
        // PWM input command
        struct {
            int16_t left_pwm;      // Left motor PWM
            int16_t right_pwm;     // Right motor PWM
        } pwm_input;
        
        // ROS control command
        struct {
            float linear_speed;    // Linear speed (m/s)
            float angular_speed;   // Angular speed (rad/s)
        } ros_ctrl;
        
        // PID parameters command
        struct {
            float kp;              // Proportional gain
            float ki;              // Integral gain
            float kd;              // Derivative gain
            float limit;           // Output limit
        } pid_params;
        

        
        // Emergency stop command
        struct {
            bool emergency;        // Emergency stop flag
        } emergency;
        
        // Configuration command
        struct {
            uint8_t main_type;     // Main robot type
            uint8_t module_type;   // Module type
            uint8_t cmd;           // Command parameter
        } config;
        
        // ESP-NOW command
        struct {
            uint8_t mac[6];        // MAC address
            char message[64];      // Message
        } esp_now;
        
        // Mission command
        struct {
            char name[32];         // Mission name
            char step[128];        // Mission step
        } mission;
        
        // Servo control command
        struct {
            uint8_t servo_id;      // Servo ID
            int16_t position;      // Target position
            uint8_t speed;         // Movement speed
        } servo_ctrl;
        
        // Robotic arm command
        struct {
            double base;           // Base angle
            double shoulder;       // Shoulder angle
            double elbow;          // Elbow angle
            double hand;           // Hand angle
        } robotic_arm;
        
        // IMU calibration command
        struct {
            uint8_t calibration_type; // Calibration type
        } imu_calibrate;
        
        // IMU configuration command
        struct {
            uint16_t gyro_range;      // Gyroscope range
            uint16_t accel_range;     // Accelerometer range
            uint16_t sample_rate;     // Sample rate
        } imu_config;
        
        // System reset command
        struct {
            uint8_t reset_type;       // Reset type
        } system_reset;
        
        // System shutdown command
        struct {
            uint8_t shutdown_type;    // Shutdown type
        } system_shutdown;
        
        // Speed rate command
        struct {
            float left_rate;          // Left motor speed rate
            float right_rate;         // Right motor speed rate
        } speed_rate;
        
        // Heartbeat command
        struct {
            uint32_t delay_ms;        // Heartbeat delay in milliseconds
        } heartbeat;
        
        // Feedback flow command
        struct {
            uint32_t interval_ms;     // Feedback interval in milliseconds
        } feedback_flow;
        
        // UART echo command
        struct {
            bool echo_mode;           // Echo mode flag
        } uart_echo;
        
        // Robotic arm joint control
        struct {
            uint8_t joint;            // Joint number (1-4)
            double angle_rad;         // Angle in radians
            uint16_t speed;           // Movement speed
            uint16_t acceleration;    // Acceleration
        } joint_ctrl;
        
        // Robotic arm all joints control
        struct {
            double base;              // Base angle
            double shoulder;          // Shoulder angle
            double elbow;             // Elbow angle
            double hand;              // Hand angle
            uint16_t speed;           // Movement speed
            uint16_t acceleration;    // Acceleration
        } joints_ctrl;
        
        // Robotic arm axis control
        struct {
            uint8_t axis;             // Axis number (1-4)
            double position;          // Target position
            uint16_t speed;           // Movement speed
        } axis_ctrl;
        
        // Robotic arm XYZT control
        struct {
            double x;                 // X position
            double y;                 // Y position
            double z;                 // Z position
            double t;                 // Theta angle
            uint16_t speed;           // Movement speed
        } xyzt_ctrl;
        
        // End effector control
        struct {
            uint8_t cmd;              // Command type
            uint16_t speed;           // Movement speed
            uint16_t acceleration;    // Acceleration
        } eoat_ctrl;
        
        // Torque control
        struct {
            uint16_t torque;          // Torque value
        } torque_ctrl;
        
        // Joint PID control
        struct {
            uint8_t joint;            // Joint number
            uint16_t p;               // Proportional gain
            uint16_t i;               // Integral gain
        } joint_pid;
        
        // Dynamic adaptation
        struct {
            uint8_t mode;             // Mode (0=stop, 1=start)
            uint16_t b;               // Base joint torque
            uint16_t s;               // Shoulder joint torque
            uint16_t e;               // Elbow joint torque
            uint16_t h;               // Hand joint torque
        } dynamic_adaptation;
        
        // Constant control
        struct {
            uint8_t mode;             // Mode (0=angle, 1=xyzt)
            uint8_t axis;             // Axis number
            uint8_t cmd;              // Command (0=stop, 1=increase, 2=decrease)
            uint16_t speed;           // Speed
        } constant_ctrl;
        
        // Gimbal control simple
        struct {
            double x;                 // X angle
            double y;                 // Y angle
            uint16_t speed;           // Movement speed
            uint16_t acceleration;    // Acceleration
        } gimbal_simple;
        
        // Gimbal control move
        struct {
            double x;                 // X angle
            double y;                 // Y angle
            uint16_t speed_x;         // X speed
            uint16_t speed_y;         // Y speed
        } gimbal_move;
        
        // Gimbal steady
        struct {
            bool steady;              // Steady mode flag
            double y_goal;            // Y goal angle
        } gimbal_steady;
        
        // Gimbal user control
        struct {
            double x;                 // X direction (-1, 0, 1, 2)
            double y;                 // Y direction (-1, 0, 1, 2)
            uint16_t speed;           // Movement speed
        } gimbal_user;
        
        // File operations
        struct {
            char name[32];            // File name
            char content[128];        // File content
            uint16_t line_num;        // Line number for operations
        } file_op;
        
        // File line operations
        struct {
            char name[32];            // File name
            uint16_t line_num;        // Line number
            char content[128];        // Line content
        } file_line_op;
        
        // Mission operations
        struct {
            char name[32];            // Mission name
            char intro[64];           // Mission introduction
            char step[128];           // Step content
            uint16_t line_num;        // Line number for operations
            float speed;              // Speed parameter
            uint32_t delay;           // Delay parameter
        } mission_op;
        
        // Mission step operations
        struct {
            char name[32];            // Mission name
            uint16_t step_num;        // Step number
            char step[128];           // Step content
        } mission_step_op;
        
        // ESP-NOW operations
        struct {
            uint8_t mode;             // Mode
            uint8_t mac[6];           // MAC address
            char message[64];         // Message
        } esp_now_op;
        
        // ESP-NOW group control
        struct {
            uint8_t dev;              // Device number
            double b;                 // Base angle
            double s;                 // Shoulder angle
            double e;                 // Elbow angle
            double h;                 // Hand angle
            uint8_t cmd;              // Command
            char message[64];         // Message
        } esp_now_group;
        
        // WiFi configuration
        struct {
            uint8_t mode;             // WiFi mode
            char ssid[32];            // SSID
            char password[64];        // Password
        } wifi_config;
        
        // WiFi AP+STA configuration
        struct {
            char ap_ssid[32];         // AP SSID
            char ap_password[64];     // AP password
            char sta_ssid[32];        // STA SSID
            char sta_password[64];    // STA password
        } wifi_apsta;
        
        // WiFi config create
        struct {
            uint8_t mode;             // WiFi mode
            char ap_ssid[32];         // AP SSID
            char ap_password[64];     // AP password
            char sta_ssid[32];        // STA SSID
            char sta_password[64];    // STA password
        } wifi_config_create;
        
        // EOAT type configuration
        struct {
            uint8_t mode;             // EOAT mode
        } eoat_type;
        
        // EOAT configuration
        struct {
            uint8_t pos;              // Position
            double ea;                // EA parameter
            double eb;                // EB parameter
        } eoat_config;
        
        // EOAT torque control
        struct {
            uint16_t torque;          // Torque value
        } eoat_torque;
        
        // New X axis configuration
        struct {
            double angle;             // Angle
        } new_x_axis;
        
        // Delay milliseconds
        struct {
            uint32_t delay;           // Delay in milliseconds
        } delay_millis;
        
        // Single joint angle control
        struct {
            uint8_t joint;            // Joint number
            double angle;             // Angle
            double speed;             // Speed
            double acceleration;      // Acceleration
        } joint_angle;
        
        // Joints angle control
        struct {
            double base;              // Base angle
            double shoulder;          // Shoulder angle
            double elbow;             // Elbow angle
            double hand;              // Hand angle
            double speed;             // Speed
            double acceleration;      // Acceleration
        } joints_angle;
        
        // Single axis control
        struct {
            uint8_t axis;             // Axis number
            double pos;               // Position
            double speed;             // Speed
        } single_axis;
        
        // Servo configuration
        struct {
            uint8_t raw_id;           // Raw servo ID
            uint8_t new_id;           // New servo ID
            uint16_t p_value;         // P value for PID
        } servo_config;
        
        // System configuration
        struct {
            uint8_t main_type;        // Main robot type
            uint8_t module_type;      // Module type
            uint8_t cmd;              // Command parameter
        } system_config;
        
        // IMU offset configuration
        struct {
            double gx;                // Gyro X offset
            double gy;                // Gyro Y offset
            double gz;                // Gyro Z offset
            double ax;                // Accel X offset
            double ay;                // Accel Y offset
            double az;                // Accel Z offset
            double cx;                // Compass X offset
            double cy;                // Compass Y offset
            double cz;                // Compass Z offset
        } imu_offset;
        
        // Delay command
        struct {
            uint32_t delay_ms;        // Delay in milliseconds
        } delay_cmd;
        
        // Arm UI control
        struct {
            double elevation;         // Elevation parameter
            double azimuth;           // Azimuth parameter
            double roll;              // Roll parameter
        } arm_ui;
        
        // OLED control
        struct {
            uint8_t line_num;         // Line number
            char text[64];            // Text content
        } oled_ctrl;
        
        // Switch control
        struct {
            int16_t pwm_a;           // PWM A value (-255 to 255)
            int16_t pwm_b;           // PWM B value (-255 to 255)
        } switch_ctrl;
        
        // Light control
        struct {
            uint8_t led;             // LED brightness (0-255)
        } light_ctrl;
        
        // Main type and module type
        struct {
            uint8_t main;            // Main robot type
            uint8_t module;          // Module type
        } mm_type;
        
        // Generic data
        uint8_t data[128];
        
        // UART control structure
        struct {
            bool echo_mode;
            bool heartbeat_enabled;
            uint32_t heartbeat_delay_ms;
            uint32_t last_cmd_time;
            uint32_t feedback_interval_ms;
            bool base_feedback_flow;
        } uart_control;
    } payload;
    
    // Command metadata
    uint32_t timestamp;    // Command timestamp
    uint8_t source;        // Command source (0=UART, 1=WiFi, 2=ESP-NOW)
    bool valid;            // Command validity flag
} json_command_t;

// UART control structure
typedef struct {
    bool echo_mode;
    bool heartbeat_enabled;
    uint32_t heartbeat_delay_ms;
    uint32_t last_cmd_time;
    uint32_t feedback_interval_ms;
    bool base_feedback_flow;
} uart_control_t;

// JSON feedback structure
typedef struct {
    uint16_t type;         // Feedback type
    union {
        // Base info feedback
        struct {
            float left_speed;      // Left motor speed
            float right_speed;     // Right motor speed
            float gyro_x;          // Gyroscope X
            float gyro_y;          // Gyroscope Y
            float gyro_z;          // Gyroscope Z
            float accel_x;         // Accelerometer X
            float accel_y;         // Accelerometer Y
            float accel_z;         // Accelerometer Z
            float mag_x;           // Magnetometer X
            float mag_y;           // Magnetometer Y
            float mag_z;           // Magnetometer Z
            int32_t odometry_left; // Left encoder count
            int32_t odometry_right; // Right encoder count
            float voltage;         // Battery voltage
        } base_info;
        
        // IMU data feedback
        struct {
            float gyro_x;          // Gyroscope X
            float gyro_y;          // Gyroscope Y
            float gyro_z;          // Gyroscope Z
            float accel_x;         // Accelerometer X
            float accel_y;         // Accelerometer Y
            float accel_z;         // Accelerometer Z
            float mag_x;           // Magnetometer X
            float mag_y;           // Magnetometer Y
            float mag_z;           // Magnetometer Z
        } imu_data;
        
        // ESP-NOW feedback
        struct {
            uint8_t mac[6];        // MAC address
            char message[64];      // Message
        } esp_now;
        
        // Servo error feedback
        struct {
            uint8_t servo_id;      // Servo ID
            uint8_t status;        // Error status
        } servo_error;
        
        // Generic data
        uint8_t data[128];
    } payload;
    
    // Feedback metadata
    uint32_t timestamp;    // Feedback timestamp
    bool valid;            // Feedback validity flag
    
    // Additional feedback fields
    char status[32];       // Status string
    bool motion_available; // Motion data available flag
    struct {
        float left_speed;      // Left motor speed
        float right_speed;     // Right motor speed
        int32_t left_encoder;  // Left encoder count
        int32_t right_encoder; // Right encoder count
    } motion;
    bool imu_available;    // IMU data available flag
    struct {
        float accel_x;         // Accelerometer X
        float accel_y;         // Accelerometer Y
        float accel_z;         // Accelerometer Z
        float gyro_x;          // Gyroscope X
        float gyro_y;          // Gyroscope Y
        float gyro_z;          // Gyroscope Z
        float temperature;     // Temperature
    } imu;
    bool battery_available; // Battery data available flag
    struct {
        float voltage;         // Battery voltage
        float current;         // Battery current
        float power;           // Battery power
        float percentage;      // Battery percentage
    } battery;
    struct {
        uint32_t uptime;       // System uptime
        uint32_t free_heap;    // Free heap memory
        float cpu_usage;       // CPU usage percentage
    } system;
} json_feedback_t;

// Command types (matching the original UGV base ROS)
#define CMD_EMERGENCY_STOP        0
#define CMD_SPEED_CTRL           1
#define CMD_SET_MOTOR_PID        2
#define CMD_OLED_CTRL            3
#define CMD_OLED_DEFAULT         4
#define CMD_MODULE_TYPE          5
#define CMD_PWM_INPUT           11
#define CMD_ROS_CTRL            13
#define CMD_EOAT_TYPE           124
#define CMD_CONFIG_EOAT          125
#define CMD_RESET_EMERGENCY     999
#define CMD_SET_ROBOT_CONFIG    900

// Additional command types
#define CMD_SERVO_CTRL           4
#define CMD_ROBOTIC_ARM         5
#define CMD_IMU_CALIBRATE       6
#define CMD_IMU_CONFIG          7
#define CMD_SYSTEM_RESET        8
#define CMD_SYSTEM_SHUTDOWN     9
#define CMD_SYSTEM_STATUS       10
#define CMD_ESP_NOW_SEND        1004

// Missing command types from original Arduino project
// Motion & Control
#define CMD_GET_SPD_RATE        139
#define CMD_SAVE_SPD_RATE       140
#define CMD_SWITCH_OFF          115
#define CMD_CONSTANT_CTRL       123
#define CMD_HEART_BEAT_SET      136
#define CMD_FEEDBACK_FLOW_INTERVAL 142
#define CMD_UART_ECHO_MODE      143

// Robotic Arm Control
#define CMD_MOVE_INIT           100
#define CMD_SINGLE_JOINT_CTRL   101
#define CMD_JOINTS_RAD_CTRL     102
#define CMD_SINGLE_AXIS_CTRL    103
#define CMD_XYZT_GOAL_CTRL      104
#define CMD_XYZT_DIRECT_CTRL    1041
#define CMD_SWITCH_CTRL         113
#define CMD_LIGHT_CTRL          114
#define CMD_MM_TYPE_SET         900
#define CMD_SERVO_RAD_FEEDBACK  105
#define CMD_EOAT_HAND_CTRL      106
#define CMD_EOAT_GRAB_TORQUE    107
#define CMD_SET_JOINT_PID       108
#define CMD_RESET_PID           109
#define CMD_SET_NEW_X           110
#define CMD_DELAY_MILLIS        111
#define CMD_DYNAMIC_ADAPTATION  112
#define CMD_SINGLE_JOINT_ANGLE  121
#define CMD_JOINTS_ANGLE_CTRL   122
#define CMD_ARM_CTRL_UI         144

// Gimbal Control
#define CMD_GIMBAL_CTRL_SIMPLE  133
#define CMD_GIMBAL_CTRL_MOVE    134
#define CMD_GIMBAL_CTRL_STOP    135
#define CMD_GIMBAL_STEADY       137
#define CMD_GIMBAL_USER_CTRL    141

// File & Mission System
#define CMD_SCAN_FILES          200
#define CMD_CREATE_FILE         201
#define CMD_READ_FILE           202
#define CMD_DELETE_FILE         203
#define CMD_APPEND_LINE         204
#define CMD_INSERT_LINE         205
#define CMD_REPLACE_LINE        206
#define CMD_READ_LINE           207
#define CMD_DELETE_LINE         208
#define CMD_CREATE_MISSION      220
#define CMD_MISSION_CONTENT     221
#define CMD_APPEND_STEP_JSON    222
#define CMD_APPEND_STEP_FB      223
#define CMD_APPEND_DELAY        224
#define CMD_INSERT_STEP_JSON    225
#define CMD_INSERT_STEP_FB      226
#define CMD_INSERT_DELAY        227
#define CMD_REPLACE_STEP_JSON   228
#define CMD_REPLACE_STEP_FB     229
#define CMD_REPLACE_DELAY       230
#define CMD_DELETE_STEP         231
#define CMD_MOVE_TO_STEP        241
#define CMD_MISSION_PLAY        242

// ESP-NOW Communication
#define CMD_BROADCAST_FOLLOWER  300
#define CMD_ESP_NOW_CONFIG      301
#define CMD_GET_MAC_ADDRESS     302
#define CMD_ESP_NOW_ADD_FOLLOWER 303
#define CMD_ESP_NOW_REMOVE_FOLLOWER 304
#define CMD_ESP_NOW_GROUP_CTRL  305
#define CMD_ESP_NOW_SINGLE      306

// WiFi Configuration
#define CMD_WIFI_ON_BOOT        401
#define CMD_SET_AP              402
#define CMD_SET_STA             403
#define CMD_WIFI_APSTA          404
#define CMD_WIFI_INFO           405
#define CMD_WIFI_CONFIG_CREATE_BY_STATUS 406
#define CMD_WIFI_CONFIG_CREATE_BY_INPUT 407
#define CMD_WIFI_STOP           408

// Servo Configuration
#define CMD_SET_SERVO_ID        501
#define CMD_SET_MIDDLE          502
#define CMD_SET_SERVO_PID       503

// System Control
#define CMD_REBOOT              600
#define CMD_FREE_FLASH_SPACE    601
#define CMD_BOOT_MISSION_INFO   602
#define CMD_RESET_BOOT_MISSION  603
#define CMD_NVS_CLEAR           604
#define CMD_INFO_PRINT          605

// Additional missing commands
#define CMD_GET_IMU_DATA       126
#define CMD_CALI_IMU_STEP      127
#define CMD_GET_IMU_OFFSET     128
#define CMD_SET_IMU_OFFSET     129
#define CMD_BASE_FEEDBACK       130
#define CMD_BASE_FEEDBACK_FLOW  131
#define CMD_LED_CTRL            132
#define CMD_SET_SPD_RATE        138
#define CMD_TORQUE_CTRL         210

// Feedback types
#define FEEDBACK_BASE_INFO       1001
#define FEEDBACK_IMU_DATA        1002
#define CMD_ESP_NOW_RECV        1003
#define CMD_ESP_NOW_SEND        1004
#define CMD_BUS_SERVO_ERROR     1005
#define FEEDBACK_IMU_OFFSET     129

// Command sources
#define CMD_SOURCE_UART          0
#define CMD_SOURCE_WIFI          1
#define CMD_SOURCE_ESP_NOW       2

// Function prototypes

/**
 * @brief Initialize JSON parser
 * @return ESP_OK on success, error code otherwise
 */
esp_err_t json_parser_init(void);

/**
 * @brief Parse JSON command string
 * @param json_str JSON string to parse
 * @param cmd Pointer to command structure
 * @return ESP_OK on success, error code otherwise
 */
esp_err_t json_parser_parse_command(const char *json_str, json_command_t *cmd);

/**
 * @brief Process JSON command
 * @param cmd Pointer to command structure
 * @return ESP_OK on success, error code otherwise
 */
esp_err_t json_parser_process_command(json_command_t *cmd);

/**
 * @brief Create JSON feedback string
 * @param feedback Pointer to feedback structure
 * @param json_str Buffer for JSON string
 * @param max_len Maximum buffer length
 * @return ESP_OK on success, error code otherwise
 */
esp_err_t json_parser_create_feedback(const json_feedback_t *feedback, char *json_str, size_t max_len);

/**
 * @brief Send base feedback
 * @return ESP_OK on success, error code otherwise
 */
esp_err_t json_parser_send_base_feedback(void);

/**
 * @brief Send IMU feedback
 * @return ESP_OK on success, error code otherwise
 */
esp_err_t json_parser_send_imu_feedback(void);

/**
 * @brief Send ESP-NOW feedback
 * @param mac MAC address
 * @param message Message string
 * @return ESP_OK on success, error code otherwise
 */
esp_err_t json_parser_send_esp_now_feedback(const uint8_t *mac, const char *message);

/**
 * @brief Send servo error feedback
 * @param servo_id Servo ID
 * @param status Error status
 * @return ESP_OK on success, error code otherwise
 */
esp_err_t json_parser_send_servo_error_feedback(uint8_t servo_id, uint8_t status);

/**
 * @brief Handle emergency stop command
 * @return ESP_OK on success, error code otherwise
 */
esp_err_t json_parser_handle_emergency_stop(void);

/**
 * @brief Handle speed control command
 * @param left_speed Left motor speed
 * @param right_speed Right motor speed
 * @return ESP_OK on success, error code otherwise
 */
esp_err_t json_parser_handle_speed_ctrl(float left_speed, float right_speed);

/**
 * @brief Handle ROS control command
 * @param linear_speed Linear speed
 * @param angular_speed Angular speed
 * @return ESP_OK on success, error code otherwise
 */
esp_err_t json_parser_handle_ros_ctrl(float linear_speed, float angular_speed);

/**
 * @brief Handle PID parameters command
 * @param kp Proportional gain
 * @param ki Integral gain
 * @param kd Derivative gain
 * @param limit Output limit
 * @return ESP_OK on success, error code otherwise
 */
esp_err_t json_parser_handle_pid_params(float kp, float ki, float kd, float limit);

/**
 * @brief Handle OLED control command
 * @param line_num Line number
 * @param text Text to display
 * @return ESP_OK on success, error code otherwise
 */
esp_err_t json_parser_handle_oled_ctrl(uint8_t line_num, const char *text);

/**
 * @brief Handle configuration command
 * @param main_type Main robot type
 * @param module_type Module type
 * @return ESP_OK on success, error code otherwise
 */
esp_err_t json_parser_handle_config(uint8_t main_type, uint8_t module_type);

/**
 * @brief Handle ESP-NOW command
 * @param mac MAC address
 * @param message Message string
 * @return ESP_OK on success, error code otherwise
 */
esp_err_t json_parser_handle_esp_now(const uint8_t *mac, const char *message);

/**
 * @brief Handle mission command
 * @param name Mission name
 * @param step Mission step
 * @return ESP_OK on success, error code otherwise
 */
esp_err_t json_parser_handle_mission(const char *name, const char *step);

/**
 * @brief Validate JSON command
 * @param cmd Pointer to command structure
 * @return True if valid, false otherwise
 */
bool json_parser_validate_command(const json_command_t *cmd);

/**
 * @brief Get command type string
 * @param type Command type
 * @return Command type string
 */
const char* json_parser_get_command_type_string(uint16_t type);

/**
 * @brief Get feedback type string
 * @param type Feedback type
 * @return Feedback type string
 */
const char* json_parser_get_feedback_type_string(uint16_t type);

// Utility functions

/**
 * @brief Convert MAC address to string
 * @param mac MAC address array
 * @param mac_str Buffer for MAC string
 * @param max_len Maximum buffer length
 * @return ESP_OK on success, error code otherwise
 */
esp_err_t json_parser_mac_to_string(const uint8_t *mac, char *mac_str, size_t max_len);

/**
 * @brief Convert string to MAC address
 * @param mac_str MAC address string
 * @param mac Buffer for MAC address array
 * @return ESP_OK on success, error code otherwise
 */
esp_err_t json_parser_string_to_mac(const char *mac_str, uint8_t *mac);

/**
 * @brief Get current timestamp
 * @return Current timestamp in milliseconds
 */
uint32_t json_parser_get_timestamp(void);

// UART controller function declarations
void uart_controller_send_feedback(void);

// OLED controller function declarations
esp_err_t oled_controller_set_text(uint8_t line_num, const char *text);
esp_err_t oled_controller_reset_to_default(void);

#ifdef __cplusplus
}
#endif

#endif // JSON_PARSER_H
