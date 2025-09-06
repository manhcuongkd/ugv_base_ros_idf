#ifndef UGV_CONFIG_H
#define UGV_CONFIG_H

#include <stdint.h>
#include <stdbool.h>
#include <driver/gpio.h>

#ifdef __cplusplus
extern "C" {
#endif

// Pin definitions for RaspRover (matching Arduino)
#define AIN1 GPIO_NUM_21      // Left motor direction 1
#define AIN2 GPIO_NUM_17      // Left motor direction 2
#define PWMA GPIO_NUM_25      // Left motor PWM
#define BIN1 GPIO_NUM_22      // Right motor direction 1
#define BIN2 GPIO_NUM_23      // Right motor direction 2
#define PWMB GPIO_NUM_26      // Right motor PWM

// Encoder pins (matching Arduino)
#define AENCA GPIO_NUM_35     // Left encoder A
#define AENCB GPIO_NUM_34     // Left encoder B
#define BENCA GPIO_NUM_27     // Right encoder A
#define BENCB GPIO_NUM_16     // Right encoder B

// I2C pins (matching Arduino ugv_config.h: S_SDA 32, S_SCL 33)
#define S_SDA GPIO_NUM_32     // I2C data line 
#define S_SCL GPIO_NUM_33     // I2C clock line

// UART pins for servo control
#define SERVO_RXD GPIO_NUM_18
#define SERVO_TXD GPIO_NUM_19

// LED control pins
#define LED_PIN GPIO_NUM_2

// OLED display pins (I2C)
#define OLED_SDA S_SDA
#define OLED_SCL S_SCL

// Battery monitoring pins and I2C address
#define BATTERY_VOLTAGE_PIN GPIO_NUM_4
#define INA219_I2C_ADDR     0x42    // INA219 I2C address (matches Arduino)
#define OLED_I2C_ADDR       0x3C    // SSD1306 OLED I2C address

// Servo IDs for RoArm-M2
#define BASE_SERVO_ID          11
#define SHOULDER_DRIVING_SERVO_ID 12
#define SHOULDER_DRIVEN_SERVO_ID  13
#define ELBOW_SERVO_ID         14
#define GRIPPER_SERVO_ID       15

// Servo configuration
#define ARM_SERVO_MIDDLE_POS   2047
#define ARM_SERVO_MIDDLE_ANGLE 180
#define ARM_SERVO_POS_RANGE    4096
#define ARM_SERVO_ANGLE_RANGE  360
#define ARM_SERVO_INIT_SPEED   600
#define ARM_SERVO_INIT_ACC     20

// Arm dimensions (mm)
#define ARM_L1_LENGTH_MM       126.06
#define ARM_L2_LENGTH_MM_A     236.82
#define ARM_L2_LENGTH_MM_B     30.00
#define ARM_L3_LENGTH_MM_A_0   280.15
#define ARM_L3_LENGTH_MM_B_0   1.73

// PWM configuration
#define PWM_FREQ               5000
#define PWM_RESOLUTION         LEDC_TIMER_13_BIT
#define PWM_MAX_DUTY           8191

// PID configuration
#define PID_SAMPLE_TIME_MS     20
#define PID_OUTPUT_LIMIT       255

// Motor configuration (matching Arduino)
#define MOTOR_MAX_SPEED        1.0f    // m/s
#define MOTOR_MAX_ANGULAR      3.14f   // rad/s

// Motion constants (from Arduino implementation)
#define WHEEL_D                0.0800  // Wheel diameter in meters
#define ONE_CIRCLE_PLUSES      660     // Encoder pulses per wheel revolution
#define TRACK_WIDTH            0.172   // Distance between wheels in meters
#define SET_MOTOR_DIR          false   // Motor direction flag

// PWM frequency (matching Arduino)
#define PWM_FREQ_ARDUINO       100000  // 100kHz PWM frequency

// PID constants (matching Arduino)
#define PID_KP                 200.0f  // Proportional gain
#define PID_KI                 2500.0f // Integral gain
#define PID_KD                 0.0f    // Derivative gain
#define PID_THRESHOLD_PWM      5       // Minimum PWM threshold
#define PID_WINDUP_LIMITS      100.0f  // Integral windup limits

// IMU configuration (ICM20948)
#define ICM20948_I2C_ADDR       0x68           // Default I2C address (AD0_VAL = 0)
#define ICM20948_I2C_ADDR_ALT   0x69           // Alternative address (AD0_VAL = 1)
#define ICM20948_WHO_AM_I       0x00           // WHO_AM_I register
#define ICM20948_WHO_AM_I_VAL   0xEA           // Expected WHO_AM_I response
#define IMU_SAMPLE_RATE_HZ      100
#define IMU_CALIBRATION_SAMPLES 1000

// Communication configuration
#define UART_BAUD_RATE         115200
#define WIFI_SSID              "RaspRover_AP"
#define WIFI_PASSWORD          "rasprover123"
#define HTTP_SERVER_PORT       80

// Task priorities
#define MAIN_TASK_PRIORITY     5
#define IMU_TASK_PRIORITY      4
#define MOTION_TASK_PRIORITY   3
#define COMM_TASK_PRIORITY     2

// Task stack sizes
#define MAIN_TASK_STACK_SIZE   8192
#define IMU_TASK_STACK_SIZE    4096
#define MOTION_TASK_STACK_SIZE 4096
#define COMM_TASK_STACK_SIZE   4096

// Queue sizes
#define JSON_CMD_QUEUE_SIZE    10
#define IMU_DATA_QUEUE_SIZE    20
#define MOTION_CMD_QUEUE_SIZE  10

// Configuration structure
typedef struct {
    uint8_t main_type;           // 1=RaspRover, 2=UGV Rover, 3=UGV Beast
    uint8_t module_type;         // 0=None, 1=RoArm-M2, 2=Gimbal
    uint8_t info_print;          // 0=Off, 1=Debug, 2=Flow feedback
    uint8_t esp_now_mode;        // 0=None, 1=Group leader, 2=Single leader, 3=Follower
    bool ctrl_by_broadcast;      // Allow broadcast control
    bool steady_mode;            // Gimbal steady mode
    bool base_feedback_flow;     // Enable base feedback
    uint8_t eem_mode;           // End effector mode: 0=Gripper, 1=Wrist
} ugv_config_t;

// Robot types
#define ROBOT_TYPE_RASPROVER   1
#define ROBOT_TYPE_UGV_ROVER   2
#define ROBOT_TYPE_UGV_BEAST   3

// Module types
#define MODULE_TYPE_NONE       0
#define MODULE_TYPE_ROARM_M2   1
#define MODULE_TYPE_GIMBAL     2

// ESP-NOW modes
#define ESP_NOW_MODE_NONE      0
#define ESP_NOW_MODE_GROUP_LEADER 1
#define ESP_NOW_MODE_SINGLE_LEADER 2
#define ESP_NOW_MODE_FOLLOWER 3

// Default configuration
#define DEFAULT_MAIN_TYPE      ROBOT_TYPE_RASPROVER
#define DEFAULT_MODULE_TYPE    MODULE_TYPE_GIMBAL
#define DEFAULT_INFO_PRINT     1
#define DEFAULT_ESP_NOW_MODE   ESP_NOW_MODE_FOLLOWER
#define DEFAULT_CTRL_BY_BROADCAST true
#define DEFAULT_STEADY_MODE    false
#define DEFAULT_BASE_FEEDBACK_FLOW true
#define DEFAULT_EEM_MODE       0

// Emergency stop configuration
#define EMERGENCY_STOP_TIMEOUT_MS 1000

// Heartbeat configuration
#define HEARTBEAT_INTERVAL_MS  1000

// File system configuration
#define SPIFFS_MAX_FILES       5
#define SPIFFS_PARTITION_LABEL "spiffs"

#ifdef __cplusplus
}
#endif

#endif // UGV_CONFIG_H
