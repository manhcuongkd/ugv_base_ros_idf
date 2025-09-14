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
#define GIMBAL_PAN_ID 2        // Pan servo ID
#define GIMBAL_TILT_ID 1       // Tilt servo ID
#define GIMBAL_PAN_CENTER 1500 // Pan center position (us)
#define GIMBAL_TILT_CENTER 1500 // Tilt center position (us)
#define GIMBAL_PAN_MIN 500     // Pan minimum position (us)
#define GIMBAL_PAN_MAX 2500    // Pan maximum position (us)
#define GIMBAL_TILT_MIN 500    // Tilt minimum position (us)
#define GIMBAL_TILT_MAX 2500   // Tilt maximum position (us)
#define GIMBAL_PWM_FREQ 50     // PWM frequency (Hz)
#define GIMBAL_PWM_RESOLUTION 16 // PWM resolution (bits)
#define SERVO_STOP_DELAY 3     // Servo stop delay (ms)

// Servo Communication
#define GIMBAL_SERVO_BAUD_RATE 1000000  // Servo communication baud rate
#define GIMBAL_SERVO_TIMEOUT 100        // Servo communication timeout (ms)

// Task Configuration
#define GIMBAL_UPDATE_RATE_HZ           50      // Gimbal control loop frequency
#define GIMBAL_TASK_STACK_SIZE          4096    // Gimbal task stack size
#define GIMBAL_TASK_PRIORITY            4       // Gimbal task priority
#define GIMBAL_QUEUE_SIZE               10      // Command queue size

// Position Constants
#define GIMBAL_CENTER_POSITION          2047    // Center position for both axes
#define GIMBAL_LEFT_POSITION            1400    // Pan left position
#define GIMBAL_RIGHT_POSITION           2600    // Pan right position  
#define GIMBAL_UP_POSITION              1400    // Tilt up position
#define GIMBAL_DOWN_POSITION            2600    // Tilt down position

// Speed and Acceleration Limits
#define GIMBAL_MIN_SPEED                0.1f    // Minimum speed value
#define GIMBAL_MAX_SPEED                10.0f   // Maximum speed value
#define GIMBAL_MIN_ACCELERATION         0.1f    // Minimum acceleration value
#define GIMBAL_MAX_ACCELERATION         10.0f   // Maximum acceleration value

// Stabilization Parameters
#define GIMBAL_STABILIZATION_THRESHOLD  1.0f    // Minimum compensation to apply
#define GIMBAL_GYRO_COMPENSATION_GAIN   0.1f    // Gyroscope compensation gain

// SCServo Protocol Constants
#define SCSERVO_UART_NUM                UART_NUM_1  // UART port for servo communication
#define SCSERVO_BAUD_RATE               1000000     // Servo communication baud rate
#define SCSERVO_HEADER1                 0xFF        // Protocol header byte 1
#define SCSERVO_HEADER2                 0xFF        // Protocol header byte 2
#define SCSERVO_INST_SYNC_WRITE         0x83        // Sync write instruction
#define SCSERVO_BROADCAST_ID            0xFE        // Broadcast ID for multiple servos
#define SMS_STS_ACC_REGISTER            41          // Acceleration register address
#define SCSERVO_DATA_LENGTH_PER_SERVO   7           // Data bytes per servo in sync write

// Angle Conversion Constants
#define PAN_ANGLE_MIN                   -180.0f     // Pan minimum angle (degrees)
#define PAN_ANGLE_MAX                   180.0f      // Pan maximum angle (degrees)
#define TILT_ANGLE_MIN                  -30.0f      // Tilt minimum angle (degrees)
#define TILT_ANGLE_MAX                  90.0f       // Tilt maximum angle (degrees)
#define SERVO_POSITION_RANGE            4095.0f     // Servo position range (0-4095)
#define PAN_ANGLE_RANGE                 360.0f      // Pan angle range (degrees)
#define TILT_ANGLE_RANGE                120.0f      // Tilt angle range (degrees)

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

// Servo Feedback Structure
typedef struct {
    bool status;
    uint16_t pos;
    int16_t speed;
    int16_t load;
    float voltage;
    float current;
    float temper;
    uint8_t mode;
} gimbal_feedback_t;

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

// Servo Feedback Functions
esp_err_t gimbal_controller_get_feedback(gimbal_feedback_t *pan_fb, gimbal_feedback_t *tilt_fb);
esp_err_t gimbal_controller_set_torque(uint8_t servo_id, bool enable);

// User Control Functions
esp_err_t gimbal_controller_user_control(int8_t input_x, int8_t input_y, uint16_t speed);

/**
 * @brief Stabilize gimbal using IMU data
 * @param imu_data Pointer to IMU data structure
 * @return ESP_OK on success, error code otherwise
 */
esp_err_t gimbal_controller_stabilize(const imu_data_t *imu_data);

#endif // GIMBAL_CONTROLLER_H
