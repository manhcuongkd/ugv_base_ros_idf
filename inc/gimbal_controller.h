#ifndef GIMBAL_CONTROLLER_H
#define GIMBAL_CONTROLLER_H

#include <esp_err.h>
#include <driver/ledc.h>
#include <freertos/FreeRTOS.h>
#include <freertos/task.h>
#include <math.h>
#include "imu_controller.h"

// Gimbal Configuration
#define GIMBAL_PAN_PIN 26      // Pan servo pin
#define GIMBAL_TILT_PIN 27     // Tilt servo pin
#define GIMBAL_PAN_CENTER 1500 // Pan center position (us)
#define GIMBAL_TILT_CENTER 1500 // Tilt center position (us)
#define GIMBAL_PAN_MIN 500     // Pan minimum position (us)
#define GIMBAL_PAN_MAX 2500    // Pan maximum position (us)
#define GIMBAL_TILT_MIN 500    // Tilt minimum position (us)
#define GIMBAL_TILT_MAX 2500   // Tilt maximum position (us)
#define GIMBAL_PWM_FREQ 50     // PWM frequency (Hz)
#define GIMBAL_PWM_RESOLUTION 16 // PWM resolution (bits)

// Gimbal Control Modes
#define GIMBAL_MODE_MANUAL 0
#define GIMBAL_MODE_AUTO 1
#define GIMBAL_MODE_TRACKING 2
#define GIMBAL_MODE_STEADY 3

// Gimbal Movement Types
#define GIMBAL_MOVE_ABSOLUTE 0
#define GIMBAL_MOVE_RELATIVE 1
#define GIMBAL_MOVE_VELOCITY 2

// Gimbal Control Structure
typedef struct {
    uint8_t mode;
    uint16_t pan_position;
    uint16_t tilt_position;
    uint16_t pan_target;
    uint16_t tilt_target;
    float pan_speed;
    float tilt_speed;
    float pan_acceleration;
    float tilt_acceleration;
    bool pan_moving;
    bool tilt_moving;
    uint32_t last_update_time;
    bool calibrated;
    uint16_t pan_min_limit;
    uint16_t pan_max_limit;
    uint16_t tilt_min_limit;
    uint16_t tilt_max_limit;
} gimbal_control_t;

// Gimbal Command Structure
typedef struct {
    uint8_t move_type;
    float pan_value;
    float tilt_value;
    float speed;
    float acceleration;
} gimbal_command_t;

// Function Prototypes
esp_err_t gimbal_controller_init(void);
esp_err_t gimbal_controller_deinit(void);
esp_err_t gimbal_controller_set_mode(uint8_t mode);
esp_err_t gimbal_controller_set_position(uint16_t pan, uint16_t tilt);
esp_err_t gimbal_controller_set_pan_tilt(float pan, float tilt);
esp_err_t gimbal_controller_set_relative_position(int16_t pan_delta, int16_t tilt_delta);
esp_err_t gimbal_controller_set_velocity(float pan_vel, float tilt_vel);
esp_err_t gimbal_controller_stop(void);
esp_err_t gimbal_controller_center(void);
esp_err_t gimbal_controller_set_speed(float pan_speed, float tilt_speed);
esp_err_t gimbal_controller_set_acceleration(float pan_acc, float tilt_acc);
esp_err_t gimbal_controller_get_position(uint16_t *pan, uint16_t *tilt);
esp_err_t gimbal_controller_get_status(gimbal_control_t *status);
esp_err_t gimbal_controller_process_command(gimbal_command_t *cmd);

// Advanced Functions
esp_err_t gimbal_controller_enable_steady_mode(bool enable);
esp_err_t gimbal_controller_set_steady_goal(float pan_goal, float tilt_goal);
esp_err_t gimbal_controller_enable_tracking_mode(bool enable);
esp_err_t gimbal_controller_set_tracking_target(float pan_target, float tilt_target);
esp_err_t gimbal_controller_calibrate(void);
esp_err_t gimbal_controller_set_limits(uint16_t pan_min, uint16_t pan_max, 
                                      uint16_t tilt_min, uint16_t tilt_max);

// Utility Functions
uint16_t gimbal_controller_degrees_to_pulse(float degrees, uint8_t axis);
float gimbal_controller_pulse_to_degrees(uint16_t pulse, uint8_t axis);
esp_err_t gimbal_controller_smooth_move(uint16_t pan_target, uint16_t tilt_target, 
                                       float speed, float acceleration);

/**
 * @brief Stabilize gimbal using IMU data
 * @param imu_data Pointer to IMU data structure
 * @return ESP_OK on success, error code otherwise
 */
esp_err_t gimbal_controller_stabilize(const imu_data_t *imu_data);

#endif // GIMBAL_CONTROLLER_H
