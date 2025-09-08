#ifndef SERVO_CONTROLLER_H
#define SERVO_CONTROLLER_H

#include <stdint.h>
#include <stdbool.h>
#include <esp_err.h>
#include "ugv_config.h"

#ifdef __cplusplus
extern "C" {
#endif

// SCServo Protocol Constants
#define SCSERVO_HEADER1 0xFF
#define SCSERVO_HEADER2 0xFF
#define SCSERVO_BROADCAST_ID 0xFE

// SCServo Instructions
#define INST_PING 0x01
#define INST_READ 0x02
#define INST_WRITE 0x03
#define INST_REG_WRITE 0x04
#define INST_ACTION 0x05
#define INST_FACTORY_RESET 0x06
#define INST_REBOOT 0x08
#define INST_SYNC_WRITE 0x83
#define INST_BULK_READ 0x92

// SCServo Memory Map (SCSCL Registers)
//-------EPROM(只读)--------
#define SCSCL_VERSION_L 3
#define SCSCL_VERSION_H 4

//-------EPROM(读写)--------
#define SCSCL_ID 5
#define SCSCL_BAUD_RATE 6
#define SCSCL_MIN_ANGLE_LIMIT_L 9
#define SCSCL_MIN_ANGLE_LIMIT_H 10
#define SCSCL_MAX_ANGLE_LIMIT_L 11
#define SCSCL_MAX_ANGLE_LIMIT_H 12
#define SCSCL_CW_DEAD 26
#define SCSCL_CCW_DEAD 27

//-------SRAM(读写)--------
#define SCSCL_TORQUE_ENABLE 40
#define SCSCL_GOAL_POSITION_L 42
#define SCSCL_GOAL_POSITION_H 43
#define SCSCL_GOAL_TIME_L 44
#define SCSCL_GOAL_TIME_H 45
#define SCSCL_GOAL_SPEED_L 46
#define SCSCL_GOAL_SPEED_H 47
#define SCSCL_LOCK 48

//-------SRAM(只读)--------
#define SCSCL_PRESENT_POSITION_L 56
#define SCSCL_PRESENT_POSITION_H 57
#define SCSCL_PRESENT_SPEED_L 58
#define SCSCL_PRESENT_SPEED_H 59
#define SCSCL_PRESENT_LOAD_L 60
#define SCSCL_PRESENT_LOAD_H 61
#define SCSCL_PRESENT_VOLTAGE 62
#define SCSCL_PRESENT_TEMPERATURE 63
#define SCSCL_MOVING 66
#define SCSCL_PRESENT_CURRENT_L 69
#define SCSCL_PRESENT_CURRENT_H 70

// Servo status
typedef enum {
    SERVO_STATUS_OK = 0,           // Servo working normally
    SERVO_STATUS_ERROR,             // Servo error
    SERVO_STATUS_TIMEOUT,           // Communication timeout
    SERVO_STATUS_INVALID_ID,        // Invalid servo ID
    SERVO_STATUS_NOT_RESPONDING,    // Servo not responding
    SERVO_STATUS_OVERLOAD,          // Servo overload
    SERVO_STATUS_OVERHEAT,          // Servo overheating
    SERVO_STATUS_VOLTAGE_ERROR      // Voltage error
} servo_status_t;

// Servo feedback data
typedef struct {
    uint8_t id;                     // Servo ID
    uint16_t position;              // Current position (0-4095)
    uint16_t target_position;       // Target position (0-4095)
    uint16_t speed;                 // Current speed
    uint16_t load;                  // Current load
    uint16_t voltage;               // Current voltage (mV)
    uint16_t temperature;           // Current temperature (0.1°C)
    uint8_t status;                 // Status byte
    bool moving;                     // Moving flag
    bool error;                      // Error flag
    uint32_t timestamp;             // Last update timestamp
} servo_feedback_t;

// Servo configuration
typedef struct {
    uint16_t id;                    // Servo ID
    uint16_t min_position;          // Minimum position
    uint16_t max_position;          // Maximum position
    uint16_t max_speed;             // Maximum speed
    uint16_t max_acceleration;      // Maximum acceleration
    uint16_t max_torque;            // Maximum torque
    uint16_t min_voltage;           // Minimum voltage (mV)
    uint16_t max_voltage;           // Maximum voltage (mV)
    uint16_t max_temperature;       // Maximum temperature (0.1°C)
    bool enable_torque;             // Enable torque
    bool enable_led;                // Enable LED
    uint8_t response_time;          // Response time
} servo_config_t;

// Arm joint configuration
typedef struct {
    uint8_t joint_id;               // Joint ID
    uint8_t servo_id;               // Servo ID
    float min_angle;                // Minimum angle (degrees)
    float max_angle;                // Maximum angle (degrees)
    float gear_ratio;               // Gear ratio
    float offset_angle;             // Offset angle (degrees)
    float pid_kp;                   // PID proportional gain
    float pid_ki;                   // PID integral gain
    float pid_kd;                   // PID derivative gain
    float pid_limit;                // PID output limit
} arm_joint_config_t;

// Arm pose (forward kinematics)
typedef struct {
    float x;                        // X coordinate (mm)
    float y;                        // Y coordinate (mm)
    float z;                        // Z coordinate (mm)
    float roll;                     // Roll angle (radians)
    float pitch;                    // Pitch angle (radians)
    float yaw;                      // Yaw angle (radians)
} arm_pose_t;

// Arm joint angles
typedef struct {
    float base;                     // Base joint angle (degrees)
    float shoulder;                 // Shoulder joint angle (degrees)
    float elbow;                    // Elbow joint angle (degrees)
    float gripper;                  // Gripper angle (degrees)
} arm_joint_angles_t;

// Function prototypes

/**
 * @brief Initialize servo controller
 * @return ESP_OK on success, error code otherwise
 */
esp_err_t servo_controller_init(void);

// Advanced Arm Control Functions
esp_err_t servo_controller_set_ui_control(float elevation, float azimuth, float roll);
esp_err_t servo_controller_set_x_position(float position, float speed);
esp_err_t servo_controller_set_y_position(float position, float speed);
esp_err_t servo_controller_set_z_position(float position, float speed);
esp_err_t servo_controller_set_theta_position(float position, float speed);
esp_err_t servo_controller_set_gripper_torque(uint16_t torque);
esp_err_t servo_controller_set_x_axis_orientation(float angle);
esp_err_t servo_controller_set_acceleration(uint8_t joint, uint16_t acceleration);
esp_err_t servo_controller_get_current_pose(arm_pose_t *pose);

/**
 * @brief Deinitialize servo controller
 * @return ESP_OK on success, error code otherwise
 */
esp_err_t servo_controller_deinit(void);

/**
 * @brief Initialize specific servo
 * @param servo_id Servo ID
 * @return ESP_OK on success, error code otherwise
 */
esp_err_t servo_controller_init_servo(uint8_t servo_id);

/**
 * @brief Initialize all servos
 * @return ESP_OK on success, error code otherwise
 */
esp_err_t servo_controller_init_all_servos(void);

/**
 * @brief Check servo status
 * @param servo_id Servo ID
 * @param status Pointer to status
 * @return ESP_OK on success, error code otherwise
 */
esp_err_t servo_controller_check_servo_status(uint8_t servo_id, servo_status_t *status);

/**
 * @brief Get servo feedback
 * @param servo_id Servo ID
 * @param feedback Pointer to feedback structure
 * @return ESP_OK on success, error code otherwise
 */
esp_err_t servo_controller_get_servo_feedback(uint8_t servo_id, servo_feedback_t *feedback);

/**
 * @brief Set servo position
 * @param servo_id Servo ID
 * @param position Target position (0-4095)
 * @param speed Movement speed
 * @return ESP_OK on success, error code otherwise
 */
esp_err_t servo_controller_set_position(uint8_t servo_id, uint16_t position, uint16_t speed);

/**
 * @brief Set servo angle
 * @param servo_id Servo ID
 * @param angle Target angle (degrees)
 * @param speed Movement speed
 * @return ESP_OK on success, error code otherwise
 */
esp_err_t servo_controller_set_angle(uint8_t servo_id, float angle, uint16_t speed);

/**
 * @brief Set servo speed
 * @param servo_id Servo ID
 * @param speed Target speed
 * @return ESP_OK on success, error code otherwise
 */
esp_err_t servo_controller_set_speed(uint8_t servo_id, uint16_t speed);

/**
 * @brief Set servo acceleration
 * @param servo_id Servo ID
 * @param acceleration Target acceleration
 * @return ESP_OK on success, error code otherwise
 */
esp_err_t servo_controller_set_acceleration(uint8_t servo_id, uint16_t acceleration);

/**
 * @brief Set servo torque
 * @param servo_id Servo ID
 * @param torque Target torque
 * @return ESP_OK on success, error code otherwise
 */
esp_err_t servo_controller_set_torque(uint8_t servo_id, uint16_t torque);

/**
 * @brief Enable/disable servo torque
 * @param servo_id Servo ID
 * @param enable True to enable, false to disable
 * @return ESP_OK on success, error code otherwise
 */
esp_err_t servo_controller_enable_torque(uint8_t servo_id, bool enable);

/**
 * @brief Enable/disable servo LED
 * @param servo_id Servo ID
 * @param enable True to enable, false to disable
 * @return ESP_OK on success, error code otherwise
 */
esp_err_t servo_controller_enable_led(uint8_t servo_id, bool enable);

/**
 * @brief Reset servo to factory defaults
 * @param servo_id Servo ID
 * @return ESP_OK on success, error code otherwise
 */
esp_err_t servo_controller_reset_servo(uint8_t servo_id);

/**
 * @brief Emergency stop all servos
 * @return ESP_OK on success, error code otherwise
 */
esp_err_t servo_controller_emergency_stop(void);

/**
 * @brief Reset emergency stop
 * @return ESP_OK on success, error code otherwise
 */
esp_err_t servo_controller_reset_emergency(void);

/**
 * @brief Disable all servos (disable torque)
 * @return ESP_OK on success, error code otherwise
 */
esp_err_t servo_controller_disable_all(void);

// RoArm-M2 specific functions

/**
 * @brief Initialize RoArm-M2
 * @return ESP_OK on success, error code otherwise
 */
esp_err_t servo_controller_init_roarm_m2(void);

/**
 * @brief Move RoArm-M2 to initial position
 * @return ESP_OK on success, error code otherwise
 */
esp_err_t servo_controller_move_init_position(void);

/**
 * @brief Move RoArm-M2 to home position
 * @return ESP_OK on success, error code otherwise
 */
esp_err_t servo_controller_move_home_position(void);

/**
 * @brief Set RoArm-M2 joint angles
 * @param angles Pointer to joint angles
 * @param speed Movement speed
 * @return ESP_OK on success, error code otherwise
 */
esp_err_t servo_controller_set_joint_angles(const arm_joint_angles_t *angles, uint16_t speed);

/**
 * @brief Set RoArm-M2 pose
 * @param pose Pointer to target pose
 * @param speed Movement speed
 * @return ESP_OK on success, error code otherwise
 */
esp_err_t servo_controller_set_pose(const arm_pose_t *pose, uint16_t speed);

/**
 * @brief Get RoArm-M2 current pose
 * @param pose Pointer to current pose
 * @return ESP_OK on success, error code otherwise
 */
esp_err_t servo_controller_get_pose(arm_pose_t *pose);

/**
 * @brief Get RoArm-M2 current joint angles
 * @param angles Pointer to current joint angles
 * @return ESP_OK on success, error code otherwise
 */
esp_err_t servo_controller_get_joint_angles(arm_joint_angles_t *angles);

/**
 * @brief Calculate forward kinematics
 * @param angles Joint angles
 * @param pose Calculated pose
 * @return ESP_OK on success, error code otherwise
 */
esp_err_t servo_controller_forward_kinematics(arm_joint_angles_t *angles, arm_pose_t *pose);

/**
 * @brief Calculate inverse kinematics
 * @param pose Target pose
 * @param angles Calculated joint angles
 * @return ESP_OK on success, error code otherwise
 */
esp_err_t servo_controller_inverse_kinematics(arm_pose_t *pose, arm_joint_angles_t *angles);

/**
 * @brief Set gripper position
 * @param position Gripper position (0-100%)
 * @param speed Movement speed
 * @return ESP_OK on success, error code otherwise
 */
esp_err_t servo_controller_set_gripper_position(uint8_t position, uint16_t speed);

/**
 * @brief Open gripper
 * @param speed Movement speed
 * @return ESP_OK on success, error code otherwise
 */
esp_err_t servo_controller_open_gripper(uint16_t speed);

/**
 * @brief Close gripper
 * @param speed Movement speed
 * @return ESP_OK on success, error code otherwise
 */
esp_err_t servo_controller_close_gripper(uint16_t speed);

/**
 * @brief Set end effector mode
 * @param mode End effector mode (0=Gripper, 1=Wrist)
 * @return ESP_OK on success, error code otherwise
 */
esp_err_t servo_controller_set_end_effector_mode(uint8_t mode);

/**
 * @brief Configure end effector assembly
 * @param position Assembly position
 * @param ea EA parameter
 * @param eb EB parameter
 * @return ESP_OK on success, error code otherwise
 */
esp_err_t servo_controller_config_end_effector(uint8_t position, float ea, float eb);

/**
 * @brief Arm UI control (E, Z, R parameters)
 * @param e E parameter
 * @param z Z parameter
 * @param r R parameter
 * @return ESP_OK on success, error code otherwise
 */
esp_err_t servo_controller_arm_ui_control(float e, float z, float r);

/**
 * @brief Set joint PID parameters
 * @param joint Joint ID (0-3)
 * @param p Proportional gain
 * @param i Integral gain
 * @return ESP_OK on success, error code otherwise
 */
esp_err_t servo_controller_set_joint_pid(uint8_t joint, float p, float i);

/**
 * @brief Reset all PID parameters to default
 * @return ESP_OK on success, error code otherwise
 */
esp_err_t servo_controller_reset_pid(void);

/**
 * @brief Initialize arm movement
 * @return ESP_OK on success, error code otherwise
 */
esp_err_t servo_controller_move_init(void);

/**
 * @brief Get servo feedback data
 * @return ESP_OK on success, error code otherwise
 */
esp_err_t servo_controller_get_feedback(void);

/**
 * @brief Control torque for all servos
 * @param enable True to enable torque, false to disable
 * @return ESP_OK on success, error code otherwise
 */
esp_err_t servo_controller_torque_control(bool enable);

/**
 * @brief Change servo ID
 * @param old_id Current servo ID
 * @param new_id New servo ID
 * @return ESP_OK on success, error code otherwise
 */
esp_err_t servo_controller_change_id(uint8_t old_id, uint8_t new_id);

/**
 * @brief Set servo middle position
 * @param servo_id Servo ID
 * @return ESP_OK on success, error code otherwise
 */
esp_err_t servo_controller_set_middle_position(uint8_t servo_id);

/**
 * @brief Set servo PID parameters
 * @param servo_id Servo ID
 * @param p Proportional gain
 * @return ESP_OK on success, error code otherwise
 */
esp_err_t servo_controller_set_servo_pid(uint8_t servo_id, float p);

// Utility functions

/**
 * @brief Convert position to angle
 * @param position Servo position (0-4095)
 * @return Angle in degrees
 */
float servo_controller_position_to_angle(uint16_t position);

/**
 * @brief Convert angle to position
 * @param angle Angle in degrees
 * @return Servo position (0-4095)
 */
uint16_t servo_controller_angle_to_position(float angle);

/**
 * @brief Check if servo is moving
 * @param servo_id Servo ID
 * @return True if moving, false otherwise
 */
bool servo_controller_is_moving(uint8_t servo_id);

/**
 * @brief Wait for servo to stop moving
 * @param servo_id Servo ID
 * @param timeout_ms Timeout in milliseconds
 * @return ESP_OK on success, error code otherwise
 */
esp_err_t servo_controller_wait_for_stop(uint8_t servo_id, uint32_t timeout_ms);

/**
 * @brief Get servo temperature
 * @param servo_id Servo ID
 * @param temperature Pointer to temperature value
 * @return ESP_OK on success, error code otherwise
 */
esp_err_t servo_controller_get_temperature(uint8_t servo_id, uint16_t *temperature);

/**
 * @brief Get servo voltage
 * @param servo_id Servo ID
 * @param voltage Pointer to voltage value
 * @return ESP_OK on success, error code otherwise
 */
esp_err_t servo_controller_get_voltage(uint8_t servo_id, uint16_t *voltage);

/**
 * @brief Get servo load
 * @param servo_id Servo ID
 * @param load Pointer to load value
 * @return ESP_OK on success, error code otherwise
 */
esp_err_t servo_controller_get_load(uint8_t servo_id, uint16_t *load);

/**
 * @brief Send servo error feedback
 * @param servo_id Servo ID
 * @param error_status Error status code
 * @return ESP_OK on success, error code otherwise
 */
esp_err_t servo_controller_send_error_feedback(uint8_t servo_id, uint8_t error_status);

// SCServo Protocol Functions
esp_err_t scservo_write_pos(uint8_t id, uint16_t position, uint16_t time, uint16_t speed);
esp_err_t scservo_write_pos_ex(uint8_t id, int16_t position, uint16_t speed, uint8_t acc);
esp_err_t scservo_reg_write_pos(uint8_t id, uint16_t position, uint16_t time, uint16_t speed);
esp_err_t scservo_reg_write_pos_ex(uint8_t id, int16_t position, uint16_t speed, uint8_t acc);
esp_err_t scservo_sync_write_pos(uint8_t *ids, uint8_t id_count, uint16_t *positions, uint16_t *times, uint16_t *speeds);
esp_err_t scservo_sync_write_pos_ex(uint8_t *ids, uint8_t id_count, int16_t *positions, uint16_t *speeds, uint8_t *accs);
esp_err_t scservo_enable_torque(uint8_t id, uint8_t enable);
esp_err_t scservo_read_pos(uint8_t id, uint16_t *position);
esp_err_t scservo_read_speed(uint8_t id, uint16_t *speed);
esp_err_t scservo_read_load(uint8_t id, uint16_t *load);
esp_err_t scservo_read_voltage(uint8_t id, uint8_t *voltage);
esp_err_t scservo_read_temperature(uint8_t id, uint8_t *temperature);
esp_err_t scservo_read_moving(uint8_t id, uint8_t *moving);
esp_err_t scservo_ping(uint8_t id);
esp_err_t scservo_factory_reset(uint8_t id);
esp_err_t scservo_reboot(uint8_t id);

// Global variables (extern declarations)
// servo_feedback array is defined as static in servo_controller.cpp
extern bool RoArmM2_initCheckSucceed;
extern arm_joint_config_t arm_joint_config[4];  // 4 joints (base, shoulder, elbow, gripper)

#ifdef __cplusplus
}
#endif

#endif // SERVO_CONTROLLER_H
