#ifndef MOTION_MODULE_H
#define MOTION_MODULE_H

#include <stdint.h>
#include <stdbool.h>
#include <esp_err.h>
#include "ugv_config.h"

#ifdef __cplusplus
extern "C" {
#endif

// Motor control structure
typedef struct {
    float left_speed;      // Left motor speed (m/s)
    float right_speed;     // Right motor speed (m/s)
    float linear_speed;    // Linear speed (m/s)
    float angular_speed;   // Angular speed (rad/s)
    int left_pwm;         // Left motor PWM value
    int right_pwm;        // Right motor PWM value
    bool emergency_stop;   // Emergency stop flag
} motion_control_t;

// Encoder data structure
typedef struct {
    int32_t left_count;    // Left encoder count
    int32_t right_count;   // Right encoder count
    float left_speed;      // Left wheel speed (m/s)
    float right_speed;     // Right wheel speed (m/s)
    float left_rpm;        // Left wheel RPM
    float right_rpm;       // Right wheel RPM
} encoder_data_t;

// PID controller structure
typedef struct {
    float kp;              // Proportional gain
    float ki;              // Integral gain
    float kd;              // Derivative gain
    float setpoint;        // Target value
    float input;           // Current value
    float output;          // Controller output
    float integral;        // Integral term
    float prev_error;      // Previous error
    float output_limit;    // Output limit
    bool enabled;          // Controller enabled flag
} pid_controller_t;

// Motion command structure
typedef struct {
    uint8_t type;          // Command type
    float left_speed;      // Left motor speed
    float right_speed;     // Right motor speed
    float linear_speed;    // Linear speed
    float angular_speed;   // Angular speed
    uint32_t duration_ms;  // Command duration
} motion_command_t;

// Command types
#define MOTION_CMD_STOP          0
#define MOTION_CMD_SPEED_CTRL   1
#define MOTION_CMD_ROS_CTRL     13
#define MOTION_CMD_EMERGENCY    14

// Function prototypes

/**
 * @brief Initialize the motion module
 * @return ESP_OK on success, error code otherwise
 */
esp_err_t motion_module_init(void);

/**
 * @brief Initialize encoders
 * @return ESP_OK on success, error code otherwise
 */
esp_err_t motion_module_init_encoders(void);

/**
 * @brief Initialize PID controllers
 * @return ESP_OK on success, error code otherwise
 */
esp_err_t motion_module_init_pid(void);

/**
 * @brief Update encoder readings
 * @return ESP_OK on success, error code otherwise
 */
esp_err_t motion_module_update_encoders(void);

/**
 * @brief Get current encoder data
 * @param data Pointer to encoder data structure
 * @return ESP_OK on success, error code otherwise
 */
esp_err_t motion_module_get_encoder_data(encoder_data_t *data);

/**
 * @brief Compute PID control for both motors
 * @return ESP_OK on success, error code otherwise
 */
esp_err_t motion_module_compute_pid(void);

/**
 * @brief Apply motor control signals
 * @return ESP_OK on success, error code otherwise
 */
esp_err_t motion_module_apply_motor_control(void);

/**
 * @brief Set motor speeds
 * @param left_speed Left motor speed (-1.0 to 1.0)
 * @param right_speed Right motor speed (-1.0 to 1.0)
 * @return ESP_OK on success, error code otherwise
 */
esp_err_t motion_module_set_speeds(float left_speed, float right_speed);

/**
 * @brief Set ROS-style motion command
 * @param linear_speed Linear speed (m/s)
 * @param angular_speed Angular speed (rad/s)
 * @return ESP_OK on success, error code otherwise
 */
esp_err_t motion_module_set_ros_motion(float linear_speed, float angular_speed);

/**
 * @brief Set ROS-style control (alias for set_ros_motion)
 * @param linear_speed Linear speed (m/s)
 * @param angular_speed Angular speed (rad/s)
 * @return ESP_OK on success, error code otherwise
 */
esp_err_t motion_module_set_ros_control(float linear_speed, float angular_speed);

/**
 * @brief Emergency stop
 * @return ESP_OK on success, error code otherwise
 */
esp_err_t motion_module_emergency_stop(void);

/**
 * @brief Reset emergency stop
 * @return ESP_OK on success, error code otherwise
 */
esp_err_t motion_module_reset_emergency(void);

/**
 * @brief Set PID parameters
 * @param left_pid Pointer to left motor PID controller
 * @param right_pid Pointer to right motor PID controller
 * @return ESP_OK on success, error code otherwise
 */
esp_err_t motion_module_set_pid_params(pid_controller_t *left_pid, pid_controller_t *right_pid);

/**
 * @brief Set PID parameters directly
 * @param kp Proportional gain
 * @param ki Integral gain
 * @param kd Derivative gain
 * @param limit Output limit
 * @return ESP_OK on success, error code otherwise
 */
esp_err_t motion_module_set_pid_params_direct(float kp, float ki, float kd, float limit);

/**
 * @brief Get current speed rates
 * @param left_rate Pointer to left speed rate
 * @param right_rate Pointer to right speed rate
 * @return ESP_OK on success, error code otherwise
 */
esp_err_t motion_module_get_speed_rates(float *left_rate, float *right_rate);

/**
 * @brief Save speed rates to NVS
 * @return ESP_OK on success, error code otherwise
 */
esp_err_t motion_module_save_speed_rates(void);

/**
 * @brief Set module type
 * @param main_type Main module type
 * @param module_type Module type
 * @return ESP_OK on success, error code otherwise
 */
esp_err_t motion_module_set_type(uint8_t main_type, uint8_t module_type);

/**
 * @brief Get current motion configuration
 * @param wheel_d Pointer to store wheel diameter
 * @param pulses Pointer to store encoder pulses per revolution
 * @param track_w Pointer to store track width
 * @param motor_dir Pointer to store motor direction flag
 * @return ESP_OK on success, error code otherwise
 */
esp_err_t motion_module_get_config(double *wheel_d, int32_t *pulses, double *track_w, bool *motor_dir);

/**
 * @brief Load motion configuration from NVS
 * @return ESP_OK on success, error code otherwise
 */
esp_err_t motion_module_load_config_from_nvs(void);

/**
 * @brief Get current motion status
 * @param status Pointer to motion control structure
 * @return ESP_OK on success, error code otherwise
 */
esp_err_t motion_module_get_status(motion_control_t *status);

/**
 * @brief Process motion command
 * @param cmd Pointer to motion command
 * @return ESP_OK on success, error code otherwise
 */
esp_err_t motion_module_process_command(motion_command_t *cmd);

/**
 * @brief Set speed rate limits
 * @param left_rate Left motor speed rate (0.0 to 1.0)
 * @param right_rate Right motor speed rate (0.0 to 1.0)
 * @return ESP_OK on success, error code otherwise
 */
esp_err_t motion_module_set_speed_rates(float left_rate, float right_rate);

/**
 * @brief Set motor speed rate (alias for set_speed_rates)
 * @param left_rate Left motor speed rate (0.0 to 1.0)
 * @param right_rate Right motor speed rate (0.0 to 1.0)
 * @return ESP_OK on success, error code otherwise
 */
esp_err_t motion_module_set_speed_rate(float left_rate, float right_rate);

/**
 * @brief Initialize PWM channels
 * @return ESP_OK on success, error code otherwise
 */
esp_err_t motion_module_init_pwm(void);

/**
 * @brief Set motor direction and PWM
 * @param motor_id Motor ID (0=Left, 1=Right)
 * @param direction Direction (-1=Reverse, 0=Stop, 1=Forward)
 * @param pwm_value PWM value (0 to PWM_MAX_DUTY)
 * @return ESP_OK on success, error code otherwise
 */
esp_err_t motion_module_set_motor(uint8_t motor_id, int8_t direction, uint16_t pwm_value);

/**
 * @brief Get left wheel speed (called from main loop)
 */
void motion_module_get_left_speed(void);

/**
 * @brief Get right wheel speed (called from main loop)
 */
void motion_module_get_right_speed(void);

/**
 * @brief Compute left motor PID (called from main loop)
 */
void motion_module_left_pid_compute(void);

/**
 * @brief Compute right motor PID (called from main loop)
 */
void motion_module_right_pid_compute(void);

// Global variables (defined as static in motion_module.cpp)
// pid_controller_t left_pid_controller;
// pid_controller_t right_pid_controller;
// encoder_data_t current_encoder_data;
// motion_control_t current_motion_status;

#ifdef __cplusplus
}
#endif

#endif // MOTION_MODULE_H
