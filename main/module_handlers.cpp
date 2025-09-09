#include <stdio.h>
#include <string.h>
#include <math.h>
#include <esp_log.h>
#include <esp_timer.h>
#include <cJSON.h>
#include <freertos/FreeRTOS.h>
#include <freertos/task.h>
#include <driver/uart.h>

#include "../inc/module_handlers.h"
#include "../inc/ugv_config.h"
#include "../inc/servo_controller.h"
#include "../inc/gimbal_controller.h"
#include "../inc/esp_now_controller.h"
#include "../inc/imu_controller.h"
#include "../inc/uart_controller.h"

static const char *TAG = "Module_Handlers";

// UART configuration (matching uart_controller.cpp)
#define UART_NUM UART_NUM_0

// Private function prototypes
static esp_err_t uart_send_feedback(const char *json_string);

// Global variables for module control
static uint32_t prev_time = 0;
static bool steady_mode = false;
static float steady_goal_y = 0.0f;
static float steady_bias = 0.0f;
static uint8_t esp_now_mode = ESP_NOW_MODE_OFF;

// RoArm-M2 constant control variables
static bool const_cmd_base_x = false;
static bool const_cmd_shoulder_y = false;
static bool const_cmd_elbow_z = false;
static bool const_cmd_eoat_t = false;
static double const_goal_base = 0.0;
static double const_goal_shoulder = 0.0;
static double const_goal_elbow = 0.0;
static double const_goal_eoat = 0.0;

// Arm position variables
static double radB = 0.0, radS = 0.0, radE = 0.0, radT = 0.0;
static double lastX = 0.0, lastY = 0.0, lastZ = 0.0, lastT = 0.0;

// Use the gimbal_feedback_t from gimbal_controller.h
static gimbal_feedback_t gimbal_feedback[2]; // [0] = pan, [1] = tilt

// ============================================================================
// MODULE TYPE SPECIFIC FUNCTIONS
// ============================================================================

/**
 * @brief RoArm-M2 module type handler
 */
void module_type_roarm_m2(void) {
    uint32_t curr_time = esp_timer_get_time() / 1000;
    
    // Constant control every 10ms
    if (curr_time - prev_time >= 10) {
        constant_handle();
        prev_time = curr_time;
    }
    
    // Get position from servo feedback
    roarm_m2_get_pos_by_servo_feedback();
    
    // ESP-NOW flow control as flow-leader
    // Note: ESP-NOW flow control functions would be implemented here
    // when ESP-NOW functionality is added to the project
    
    // Info feedback if enabled
    if (ugv_config.info_print == 2) {
        roarm_m2_info_feedback();
    }
}

/**
 * @brief Gimbal module type handler
 */
void module_type_gimbal(void) {
    // Get gimbal feedback
    get_gimbal_feedback();
    
    // Apply gimbal stabilization
    gimbal_steady(steady_goal_y);
}

// ============================================================================
// ROARM-M2 SPECIFIC FUNCTIONS
// ============================================================================

/**
 * @brief Constant control handler for RoArm-M2
 */
void constant_handle(void) {
    if (!const_cmd_base_x && !const_cmd_shoulder_y && !const_cmd_elbow_z && !const_cmd_eoat_t) {
        const_goal_base = radB;
        const_goal_shoulder = radS;
        const_goal_elbow = radE;
        const_goal_eoat = radT;
    }
    
    // Apply constant control to servos
    if (const_cmd_base_x) {
        // Move base servo to constant goal
        scservo_write_pos(ROARM_BASE_SERVO_ID, (uint16_t)(const_goal_base * 180.0 / M_PI), 100, 100);
    }
    
    if (const_cmd_shoulder_y) {
        // Move shoulder servos to constant goal
        scservo_write_pos(ROARM_SHOULDER_DRIVING_SERVO_ID, (uint16_t)(const_goal_shoulder * 180.0 / M_PI), 100, 100);
        scservo_write_pos(ROARM_SHOULDER_DRIVEN_SERVO_ID, (uint16_t)(const_goal_shoulder * 180.0 / M_PI), 100, 100);
    }
    
    if (const_cmd_elbow_z) {
        // Move elbow servo to constant goal
        scservo_write_pos(ROARM_ELBOW_SERVO_ID, (uint16_t)(const_goal_elbow * 180.0 / M_PI), 100, 100);
    }
    
    if (const_cmd_eoat_t) {
        // Move end-effector servo to constant goal
        scservo_write_pos(ROARM_GRIPPER_SERVO_ID, (uint16_t)(const_goal_eoat * 180.0 / M_PI), 100, 100);
    }
}

/**
 * @brief Get RoArm-M2 position from servo feedback
 */
void roarm_m2_get_pos_by_servo_feedback(void) {
    // Get servo positions and convert to radians
    uint16_t base_pos, shoulder_pos, elbow_pos, gripper_pos;
    
    if (scservo_read_pos(ROARM_BASE_SERVO_ID, &base_pos) == ESP_OK) {
        radB = (double)base_pos * M_PI / 180.0;
    }
    
    if (scservo_read_pos(ROARM_SHOULDER_DRIVING_SERVO_ID, &shoulder_pos) == ESP_OK) {
        radS = (double)shoulder_pos * M_PI / 180.0;
    }
    
    if (scservo_read_pos(ROARM_ELBOW_SERVO_ID, &elbow_pos) == ESP_OK) {
        radE = (double)elbow_pos * M_PI / 180.0;
    }
    
    if (scservo_read_pos(ROARM_GRIPPER_SERVO_ID, &gripper_pos) == ESP_OK) {
        radT = (double)gripper_pos * M_PI / 180.0;
    }
    
    // Update last position (simplified kinematics)
    lastX = 100.0 + 50.0 * cos(radB) * cos(radS);
    lastY = 50.0 * sin(radB) * cos(radS);
    lastZ = 100.0 + 50.0 * sin(radS);
    lastT = radT;
}

/**
 * @brief Send RoArm-M2 info feedback
 */
void roarm_m2_info_feedback(void) {
    ESP_LOGI(TAG, "Sending RoArm-M2 info feedback");
    
    // Create feedback JSON
    cJSON *feedback = cJSON_CreateObject();
    cJSON_AddNumberToObject(feedback, "T", 1051); // RoArm-M2 feedback type
    
    // Arm position feedback
    cJSON_AddNumberToObject(feedback, "ax", lastX);
    cJSON_AddNumberToObject(feedback, "ay", lastY);
    cJSON_AddNumberToObject(feedback, "az", lastZ);
    cJSON_AddNumberToObject(feedback, "ab", radB);
    cJSON_AddNumberToObject(feedback, "as", radS);
    cJSON_AddNumberToObject(feedback, "ae", radE);
    cJSON_AddNumberToObject(feedback, "at", lastT);
    
    // Servo torque feedback (simplified)
    cJSON_AddNumberToObject(feedback, "torB", 0);
    cJSON_AddNumberToObject(feedback, "torS", 0);
    cJSON_AddNumberToObject(feedback, "torE", 0);
    cJSON_AddNumberToObject(feedback, "torH", 0);
    
    // Send via UART
    char *json_string = cJSON_Print(feedback);
    if (json_string) {
        esp_err_t ret = uart_send_feedback(json_string);
        if (ret == ESP_OK) {
            ESP_LOGD(TAG, "RoArm-M2 feedback sent via UART: %s", json_string);
        } else {
            ESP_LOGE(TAG, "Failed to send RoArm-M2 feedback via UART: %s", esp_err_to_name(ret));
            // Fallback to logging if UART fails
            ESP_LOGI(TAG, "RoArm-M2 feedback (fallback): %s", json_string);
        }
        free(json_string);
    }
    
    cJSON_Delete(feedback);
}

// ============================================================================
// GIMBAL SPECIFIC FUNCTIONS
// ============================================================================

/**
 * @brief Get gimbal feedback from servos
 */
void get_gimbal_feedback(void) {
    // Get pan servo feedback
    gimbal_feedback_t pan_fb, tilt_fb;
    esp_err_t ret = gimbal_controller_get_feedback(&pan_fb, &tilt_fb);
    
    if (ret == ESP_OK) {
        gimbal_feedback[0] = pan_fb;  // Pan
        gimbal_feedback[1] = tilt_fb; // Tilt
        
        ESP_LOGD(TAG, "Gimbal feedback - Pan: pos=%d, speed=%d, load=%d", 
                 pan_fb.pos, pan_fb.speed, pan_fb.load);
        ESP_LOGD(TAG, "Gimbal feedback - Tilt: pos=%d, speed=%d, load=%d", 
                 tilt_fb.pos, tilt_fb.speed, tilt_fb.load);
    } else {
        ESP_LOGW(TAG, "Failed to get gimbal feedback");
        // Set default values
        gimbal_feedback[0].status = false;
        gimbal_feedback[1].status = false;
    }
}

/**
 * @brief Set gimbal steady mode
 * @param input_cmd True to enable steady mode, false to disable
 * @param input_y Y-axis bias for steady mode
 */
void gimbal_steady_set(bool input_cmd, float input_y) {
    steady_mode = input_cmd;
    
    if (input_y < GIMBAL_STEADY_BIAS_MIN) {
        steady_bias = GIMBAL_STEADY_BIAS_MIN;
    } else if (input_y > GIMBAL_STEADY_BIAS_MAX) {
        steady_bias = GIMBAL_STEADY_BIAS_MAX;
    } else {
        steady_bias = input_y;
    }
    
    ESP_LOGI(TAG, "Gimbal steady mode: %s, bias: %.2f", 
             steady_mode ? "enabled" : "disabled", steady_bias);
}

/**
 * @brief Apply gimbal stabilization
 * @param input_bias_y Y-axis bias for stabilization
 */
void gimbal_steady(float input_bias_y) {
    if (!steady_mode) {
        return;
    }
    
    // Get IMU data for stabilization
    const imu_data_t *imu_data = imu_controller_get_data();
    
    if (!imu_data) {
        ESP_LOGW(TAG, "Failed to get IMU data for gimbal stabilization");
        return;
    }
    
    // Calculate stabilization angles based on IMU data
    float pan_angle = 0.0f;
    float tilt_angle = input_bias_y + steady_bias;
    
    // Apply sophisticated IMU-based stabilization with PID control
    static float prev_roll = 0.0f, prev_pitch = 0.0f;
    static float roll_integral = 0.0f, pitch_integral = 0.0f;
    static uint32_t last_stabilization_time = 0;
    
    uint32_t current_time = esp_timer_get_time() / 1000; // Convert to ms
    float dt = (current_time - last_stabilization_time) / 1000.0f; // Convert to seconds
    if (dt <= 0.0f || dt > 0.1f) dt = 0.01f; // Limit dt to reasonable range
    last_stabilization_time = current_time;
    
    // PID parameters for stabilization
    const float KP_ROLL = 2.0f;   // Proportional gain for roll
    const float KI_ROLL = 0.1f;   // Integral gain for roll
    const float KD_ROLL = 0.5f;   // Derivative gain for roll
    const float KP_PITCH = 2.0f;  // Proportional gain for pitch
    const float KI_PITCH = 0.1f;  // Integral gain for pitch
    const float KD_PITCH = 0.5f;  // Derivative gain for pitch
    
    // Dead zone to prevent jitter from small movements
    const float DEAD_ZONE = 0.05f; // 0.05 radians (~3 degrees)
    
    // Roll stabilization (pan control)
    float roll_error = -imu_data->roll; // Negative for counter-rotation
    if (fabs(roll_error) > DEAD_ZONE) {
        // Proportional term
        float roll_p = roll_error * KP_ROLL;
        
        // Integral term with windup protection
        roll_integral += roll_error * dt;
        roll_integral = fmaxf(-1.0f, fminf(1.0f, roll_integral)); // Limit integral windup
        float roll_i = roll_integral * KI_ROLL;
        
        // Derivative term
        float roll_d = (roll_error - prev_roll) / dt * KD_ROLL;
        
        // PID output
        float roll_output = roll_p + roll_i + roll_d;
        pan_angle += roll_output;
        
        prev_roll = roll_error;
    } else {
        // Reset integral when in dead zone
        roll_integral *= 0.95f; // Gradual decay
    }
    
    // Pitch stabilization (tilt control)
    float pitch_error = -imu_data->pitch; // Negative for counter-rotation
    if (fabs(pitch_error) > DEAD_ZONE) {
        // Proportional term
        float pitch_p = pitch_error * KP_PITCH;
        
        // Integral term with windup protection
        pitch_integral += pitch_error * dt;
        pitch_integral = fmaxf(-1.0f, fminf(1.0f, pitch_integral)); // Limit integral windup
        float pitch_i = pitch_integral * KI_PITCH;
        
        // Derivative term
        float pitch_d = (pitch_error - prev_pitch) / dt * KD_PITCH;
        
        // PID output
        float pitch_output = pitch_p + pitch_i + pitch_d;
        tilt_angle += pitch_output;
        
        prev_pitch = pitch_error;
    } else {
        // Reset integral when in dead zone
        pitch_integral *= 0.95f; // Gradual decay
    }
    
    // Constrain angles
    pan_angle = fmaxf(-90.0f, fminf(90.0f, pan_angle));
    tilt_angle = fmaxf(-90.0f, fminf(90.0f, tilt_angle));
    
    // Apply gimbal control
    gimbal_controller_user_control((int8_t)pan_angle, (int8_t)tilt_angle, 100);
    
    ESP_LOGD(TAG, "Gimbal steady: pan=%.2f, tilt=%.2f", pan_angle, tilt_angle);
}

// ============================================================================
// ESP-NOW FLOW CONTROL FUNCTIONS
// ============================================================================



// ============================================================================
// PUBLIC INTERFACE FUNCTIONS
// ============================================================================

/**
 * @brief Set constant control commands for RoArm-M2
 * @param base_x Base X control enable
 * @param shoulder_y Shoulder Y control enable
 * @param elbow_z Elbow Z control enable
 * @param eoat_t End-effector T control enable
 */
void module_handlers_set_constant_control(bool base_x, bool shoulder_y, bool elbow_z, bool eoat_t) {
    const_cmd_base_x = base_x;
    const_cmd_shoulder_y = shoulder_y;
    const_cmd_elbow_z = elbow_z;
    const_cmd_eoat_t = eoat_t;
    
    ESP_LOGI(TAG, "Constant control set: base_x=%d, shoulder_y=%d, elbow_z=%d, eoat_t=%d",
             base_x, shoulder_y, elbow_z, eoat_t);
}

/**
 * @brief Set constant control goals for RoArm-M2
 * @param base_goal Base goal angle (radians)
 * @param shoulder_goal Shoulder goal angle (radians)
 * @param elbow_goal Elbow goal angle (radians)
 * @param eoat_goal End-effector goal angle (radians)
 */
void module_handlers_set_constant_goals(double base_goal, double shoulder_goal, double elbow_goal, double eoat_goal) {
    const_goal_base = base_goal;
    const_goal_shoulder = shoulder_goal;
    const_goal_elbow = elbow_goal;
    const_goal_eoat = eoat_goal;
    
    ESP_LOGI(TAG, "Constant goals set: base=%.2f, shoulder=%.2f, elbow=%.2f, eoat=%.2f",
             base_goal, shoulder_goal, elbow_goal, eoat_goal);
}

/**
 * @brief Set ESP-NOW mode
 * @param mode ESP-NOW mode (0=off, 1=group, 2=single)
 */
void module_handlers_set_esp_now_mode(uint8_t mode) {
    esp_now_mode = mode;
    ESP_LOGI(TAG, "ESP-NOW mode set to: %d", mode);
}

/**
 * @brief Get current arm positions
 * @param base Base angle (radians)
 * @param shoulder Shoulder angle (radians)
 * @param elbow Elbow angle (radians)
 * @param eoat End-effector angle (radians)
 */
void module_handlers_get_arm_positions(double *base, double *shoulder, double *elbow, double *eoat) {
    if (base) *base = radB;
    if (shoulder) *shoulder = radS;
    if (elbow) *elbow = radE;
    if (eoat) *eoat = radT;
}

/**
 * @brief Get current gimbal feedback
 * @param pan_fb Pan servo feedback
 * @param tilt_fb Tilt servo feedback
 */
void module_handlers_get_gimbal_feedback(gimbal_feedback_t *pan_fb, gimbal_feedback_t *tilt_fb) {
    if (pan_fb) *pan_fb = gimbal_feedback[0];
    if (tilt_fb) *tilt_fb = gimbal_feedback[1];
}

// ============================================================================
// PRIVATE FUNCTIONS
// ============================================================================

/**
 * @brief Send feedback data via UART
 * @param json_string JSON string to send
 * @return ESP_OK on success, error code otherwise
 */
static esp_err_t uart_send_feedback(const char *json_string) {
    if (!json_string) {
        return ESP_ERR_INVALID_ARG;
    }
    
    size_t len = strlen(json_string);
    if (len == 0) {
        return ESP_ERR_INVALID_ARG;
    }
    
    // Send JSON string via UART
    int bytes_written = uart_write_bytes(UART_NUM, json_string, len);
    if (bytes_written != len) {
        ESP_LOGE(TAG, "UART write incomplete: wrote %d of %d bytes", bytes_written, len);
        return ESP_FAIL;
    }
    
    // Send newline to terminate the JSON message
    const char *newline = "\n";
    bytes_written = uart_write_bytes(UART_NUM, newline, 1);
    if (bytes_written != 1) {
        ESP_LOGE(TAG, "Failed to send UART newline");
        return ESP_FAIL;
    }
    
    // Wait for transmission to complete
    uart_wait_tx_done(UART_NUM, portMAX_DELAY);
    
    return ESP_OK;
}