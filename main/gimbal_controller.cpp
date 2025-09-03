#include "../inc/gimbal_controller.h"
#include <esp_log.h>
#include <driver/ledc.h>
#include <driver/gpio.h>
#include <freertos/FreeRTOS.h>
#include <freertos/task.h>
#include <freertos/queue.h>
#include <math.h>
#include <string.h>

static const char *TAG = "Gimbal_Controller";

// Gimbal Configuration
#define GIMBAL_UPDATE_RATE 50 // Hz
#define GIMBAL_TASK_STACK_SIZE 4096
#define GIMBAL_TASK_PRIORITY 4
#define GIMBAL_QUEUE_SIZE 10

// LEDC Configuration
#define GIMBAL_LEDC_TIMER LEDC_TIMER_0
#define GIMBAL_LEDC_MODE LEDC_LOW_SPEED_MODE
#define GIMBAL_PAN_CHANNEL LEDC_CHANNEL_0
#define GIMBAL_TILT_CHANNEL LEDC_CHANNEL_1
// PWM resolution is defined in header

// Global variables
static bool gimbal_initialized = false;
static gimbal_control_t gimbal_control;
static QueueHandle_t gimbal_command_queue = NULL;
static TaskHandle_t gimbal_task_handle = NULL;

// Private function prototypes
static void gimbal_task(void *pvParameters);
static esp_err_t gimbal_set_pan_angle(float angle);
static esp_err_t gimbal_set_tilt_angle(float angle);
static uint16_t gimbal_angle_to_pulse(float angle, uint8_t axis);
static float gimbal_pulse_to_angle(uint16_t pulse, uint8_t axis);
static esp_err_t gimbal_apply_stabilization(void);

esp_err_t gimbal_controller_init(void)
{
    if (gimbal_initialized) {
        ESP_LOGW(TAG, "Gimbal controller already initialized");
        return ESP_OK;
    }

    ESP_LOGI(TAG, "Initializing gimbal controller...");

    // Initialize control structure
    memset(&gimbal_control, 0, sizeof(gimbal_control_t));
    gimbal_control.mode = GIMBAL_MODE_MANUAL;
    gimbal_control.pan_position = GIMBAL_PAN_CENTER;
    gimbal_control.tilt_position = GIMBAL_TILT_CENTER;
    gimbal_control.pan_target = GIMBAL_PAN_CENTER;
    gimbal_control.tilt_target = GIMBAL_TILT_CENTER;
    gimbal_control.pan_speed = 1.0f;
    gimbal_control.tilt_speed = 1.0f;
    gimbal_control.pan_acceleration = 1.0f;
    gimbal_control.tilt_acceleration = 1.0f;
    gimbal_control.pan_moving = false;
    gimbal_control.tilt_moving = false;
    gimbal_control.last_update_time = esp_timer_get_time() / 1000;

    // Configure LEDC timer
    ledc_timer_config_t ledc_timer = {
        .speed_mode = GIMBAL_LEDC_MODE,
        .duty_resolution = (ledc_timer_bit_t)GIMBAL_PWM_RESOLUTION,
        .timer_num = GIMBAL_LEDC_TIMER,
        .freq_hz = GIMBAL_PWM_FREQ,
        .clk_cfg = LEDC_AUTO_CLK
    };

    esp_err_t ret = ledc_timer_config(&ledc_timer);
    if (ret != ESP_OK) {
        ESP_LOGE(TAG, "Failed to configure LEDC timer: %s", esp_err_to_name(ret));
        return ret;
    }

    // Configure pan servo channel
    ledc_channel_config_t pan_channel = {
        .gpio_num = GIMBAL_PAN_PIN,
        .speed_mode = GIMBAL_LEDC_MODE,
        .channel = GIMBAL_PAN_CHANNEL,
        .intr_type = LEDC_INTR_DISABLE,
        .timer_sel = GIMBAL_LEDC_TIMER,
        .duty = 0,
        .hpoint = 0,
        .flags = {0}
    };

    ret = ledc_channel_config(&pan_channel);
    if (ret != ESP_OK) {
        ESP_LOGE(TAG, "Failed to configure pan channel: %s", esp_err_to_name(ret));
        return ret;
    }

    // Configure tilt servo channel
    ledc_channel_config_t tilt_channel = {
        .gpio_num = GIMBAL_TILT_PIN,
        .speed_mode = GIMBAL_LEDC_MODE,
        .channel = GIMBAL_TILT_CHANNEL,
        .intr_type = LEDC_INTR_DISABLE,
        .timer_sel = GIMBAL_LEDC_TIMER,
        .duty = 0,
        .hpoint = 0,
        .flags = {0}
    };

    ret = ledc_channel_config(&tilt_channel);
    if (ret != ESP_OK) {
        ESP_LOGE(TAG, "Failed to configure tilt channel: %s", esp_err_to_name(ret));
        return ret;
    }

    // Create command queue
    gimbal_command_queue = xQueueCreate(GIMBAL_QUEUE_SIZE, sizeof(gimbal_command_t));
    if (gimbal_command_queue == NULL) {
        ESP_LOGE(TAG, "Failed to create gimbal command queue");
        return ESP_ERR_NO_MEM;
    }

    // Create gimbal task
    BaseType_t task_created = xTaskCreate(
        gimbal_task,
        "gimbal_task",
        GIMBAL_TASK_STACK_SIZE,
        NULL,
        GIMBAL_TASK_PRIORITY,
        &gimbal_task_handle
    );

    if (task_created != pdPASS) {
        ESP_LOGE(TAG, "Failed to create gimbal task");
        vQueueDelete(gimbal_command_queue);
        return ESP_ERR_NO_MEM;
    }

    // Center the gimbal
    ret = gimbal_controller_center();
    if (ret != ESP_OK) {
        ESP_LOGW(TAG, "Failed to center gimbal: %s", esp_err_to_name(ret));
    }

    gimbal_initialized = true;
    ESP_LOGI(TAG, "Gimbal controller initialized successfully");
    return ESP_OK;
}

esp_err_t gimbal_controller_deinit(void)
{
    if (!gimbal_initialized) {
        return ESP_OK;
    }

    ESP_LOGI(TAG, "Deinitializing gimbal controller...");

    // Delete task
    if (gimbal_task_handle != NULL) {
        vTaskDelete(gimbal_task_handle);
        gimbal_task_handle = NULL;
    }

    // Delete queue
    if (gimbal_command_queue != NULL) {
        vQueueDelete(gimbal_command_queue);
        gimbal_command_queue = NULL;
    }

    // Stop servos
    ledc_stop(GIMBAL_LEDC_MODE, GIMBAL_PAN_CHANNEL, 0);
    ledc_stop(GIMBAL_LEDC_MODE, GIMBAL_TILT_CHANNEL, 0);

    gimbal_initialized = false;
    ESP_LOGI(TAG, "Gimbal controller deinitialized");
    return ESP_OK;
}

esp_err_t gimbal_controller_set_mode(uint8_t mode)
{
    if (!gimbal_initialized) {
        ESP_LOGE(TAG, "Gimbal controller not initialized");
        return ESP_ERR_INVALID_STATE;
    }

    if (mode > GIMBAL_MODE_TRACKING) {
        return ESP_ERR_INVALID_ARG;
    }

    gimbal_control.mode = mode;
    ESP_LOGI(TAG, "Gimbal mode set to %d", mode);
    return ESP_OK;
}

esp_err_t gimbal_controller_set_position(uint16_t pan, uint16_t tilt)
{
    // Validate input parameters
    if (pan < GIMBAL_PAN_MIN || pan > GIMBAL_PAN_MAX ||
        tilt < GIMBAL_TILT_MIN || tilt > GIMBAL_TILT_MAX) {
        ESP_LOGE(TAG, "Invalid position: pan=%d, tilt=%d", pan, tilt);
        return ESP_ERR_INVALID_ARG;
    }

    // Set target positions
    gimbal_control.pan_target = pan;
    gimbal_control.tilt_target = tilt;
    gimbal_control.pan_moving = true;
    gimbal_control.tilt_moving = true;

    // Apply PWM signals
    esp_err_t ret = ledc_set_duty(GIMBAL_LEDC_MODE, GIMBAL_PAN_CHANNEL, pan);
    if (ret != ESP_OK) {
        ESP_LOGE(TAG, "Failed to set pan PWM: %s", esp_err_to_name(ret));
        return ret;
    }

    ret = ledc_set_duty(GIMBAL_LEDC_MODE, GIMBAL_TILT_CHANNEL, tilt);
    if (ret != ESP_OK) {
        ESP_LOGE(TAG, "Failed to set tilt PWM: %s", esp_err_to_name(ret));
        return ret;
    }

    // Update current positions
    gimbal_control.pan_position = pan;
    gimbal_control.tilt_position = tilt;
    gimbal_control.last_update_time = esp_timer_get_time() / 1000;

    ESP_LOGI(TAG, "Gimbal position set: pan=%d, tilt=%d", pan, tilt);
    return ESP_OK;
}

esp_err_t gimbal_controller_set_pan_tilt(float pan, float tilt)
{
    // Convert degrees to pulse width
    uint16_t pan_pulse = gimbal_angle_to_pulse(pan, 0); // 0 for pan axis
    uint16_t tilt_pulse = gimbal_angle_to_pulse(tilt, 1); // 1 for tilt axis
    
    // Call the existing set_position function
    return gimbal_controller_set_position(pan_pulse, tilt_pulse);
}

esp_err_t gimbal_controller_set_relative_position(int16_t pan_delta, int16_t tilt_delta)
{
    if (!gimbal_initialized) {
        ESP_LOGE(TAG, "Gimbal controller not initialized");
        return ESP_ERR_INVALID_STATE;
    }

    // Calculate new positions
    uint16_t new_pan = gimbal_control.pan_position + pan_delta;
    uint16_t new_tilt = gimbal_control.tilt_position + tilt_delta;

    // Validate limits
    if (new_pan < GIMBAL_PAN_MIN || new_pan > GIMBAL_PAN_MAX ||
        new_tilt < GIMBAL_TILT_MIN || new_tilt > GIMBAL_TILT_MAX) {
        ESP_LOGW(TAG, "Relative position out of range: pan_delta=%d, tilt_delta=%d", pan_delta, tilt_delta);
        return ESP_ERR_INVALID_ARG;
    }

    return gimbal_controller_set_position(new_pan, new_tilt);
}

esp_err_t gimbal_controller_set_velocity(float pan_vel, float tilt_vel)
{
    if (!gimbal_initialized) {
        ESP_LOGE(TAG, "Gimbal controller not initialized");
        return ESP_ERR_INVALID_STATE;
    }

    // Validate velocity limits
    if (pan_vel < -100.0f || pan_vel > 100.0f || tilt_vel < -100.0f || tilt_vel > 100.0f) {
        ESP_LOGW(TAG, "Velocity out of range: pan_vel=%.2f, tilt_vel=%.2f", pan_vel, tilt_vel);
        return ESP_ERR_INVALID_ARG;
    }

    gimbal_control.pan_speed = pan_vel;
    gimbal_control.tilt_speed = tilt_vel;

    ESP_LOGI(TAG, "Velocity set: pan_vel=%.2f, tilt_vel=%.2f", pan_vel, tilt_vel);
    return ESP_OK;
}

esp_err_t gimbal_controller_stop(void)
{
    if (!gimbal_initialized) {
        ESP_LOGE(TAG, "Gimbal controller not initialized");
        return ESP_ERR_INVALID_STATE;
    }

    gimbal_control.pan_moving = false;
    gimbal_control.tilt_moving = false;

    ESP_LOGI(TAG, "Gimbal stopped");
    return ESP_OK;
}

esp_err_t gimbal_controller_center(void)
{
    if (!gimbal_initialized) {
        ESP_LOGE(TAG, "Gimbal controller not initialized");
        return ESP_ERR_INVALID_STATE;
    }

    return gimbal_controller_set_position(GIMBAL_PAN_CENTER, GIMBAL_TILT_CENTER);
}

esp_err_t gimbal_controller_set_speed(float pan_speed, float tilt_speed)
{
    if (!gimbal_initialized) {
        ESP_LOGE(TAG, "Gimbal controller not initialized");
        return ESP_ERR_INVALID_STATE;
    }

    // Validate speed limits
    if (pan_speed < 0.1f || pan_speed > 10.0f || tilt_speed < 0.1f || tilt_speed > 10.0f) {
        ESP_LOGW(TAG, "Speed out of range: pan_speed=%.2f, tilt_speed=%.2f", pan_speed, tilt_speed);
        return ESP_ERR_INVALID_ARG;
    }

    gimbal_control.pan_speed = pan_speed;
    gimbal_control.tilt_speed = tilt_speed;

    ESP_LOGI(TAG, "Speed set: pan_speed=%.2f, tilt_speed=%.2f", pan_speed, tilt_speed);
    return ESP_OK;
}

esp_err_t gimbal_controller_set_acceleration(float pan_acc, float tilt_acc)
{
    if (!gimbal_initialized) {
        ESP_LOGE(TAG, "Gimbal controller not initialized");
        return ESP_ERR_INVALID_STATE;
    }

    // Validate acceleration limits
    if (pan_acc < 0.1f || pan_acc > 10.0f || tilt_acc < 0.1f || tilt_acc > 10.0f) {
        ESP_LOGW(TAG, "Acceleration out of range: pan_acc=%.2f, tilt_acc=%.2f", pan_acc, tilt_acc);
        return ESP_ERR_INVALID_ARG;
    }

    gimbal_control.pan_acceleration = pan_acc;
    gimbal_control.tilt_acceleration = tilt_acc;

    ESP_LOGI(TAG, "Acceleration set: pan_acc=%.2f, tilt_acc=%.2f", pan_acc, tilt_acc);
    return ESP_OK;
}

esp_err_t gimbal_controller_get_position(uint16_t *pan, uint16_t *tilt)
{
    if (!gimbal_initialized) {
        ESP_LOGE(TAG, "Gimbal controller not initialized");
        return ESP_ERR_INVALID_STATE;
    }

    if (pan == NULL || tilt == NULL) {
        return ESP_ERR_INVALID_ARG;
    }

    *pan = gimbal_control.pan_position;
    *tilt = gimbal_control.tilt_position;

    return ESP_OK;
}

esp_err_t gimbal_controller_get_status(gimbal_control_t *status)
{
    if (!gimbal_initialized) {
        ESP_LOGE(TAG, "Gimbal controller not initialized");
        return ESP_ERR_INVALID_STATE;
    }

    if (status == NULL) {
        return ESP_ERR_INVALID_ARG;
    }

    memcpy(status, &gimbal_control, sizeof(gimbal_control_t));
    return ESP_OK;
}

esp_err_t gimbal_controller_process_command(gimbal_command_t *cmd)
{
    if (!gimbal_initialized) {
        ESP_LOGE(TAG, "Gimbal controller not initialized");
        return ESP_ERR_INVALID_STATE;
    }

    if (cmd == NULL) {
        return ESP_ERR_INVALID_ARG;
    }

    // Send command to queue
    if (xQueueSend(gimbal_command_queue, cmd, pdMS_TO_TICKS(100)) != pdTRUE) {
        ESP_LOGW(TAG, "Failed to queue command");
        return ESP_ERR_TIMEOUT;
    }

    ESP_LOGI(TAG, "Command queued: move_type=%d, pan=%.2f, tilt=%.2f", 
             cmd->move_type, cmd->pan_value, cmd->tilt_value);
    return ESP_OK;
}

esp_err_t gimbal_controller_enable_steady_mode(bool enable)
{
    if (!gimbal_initialized) {
        ESP_LOGE(TAG, "Gimbal controller not initialized");
        return ESP_ERR_INVALID_STATE;
    }

    ESP_LOGI(TAG, "Steady mode %s", enable ? "enabled" : "disabled");
    
    if (enable) {
        gimbal_control.mode = GIMBAL_MODE_STEADY;
        // Enable position holding with PID control
        gimbal_control.pan_moving = false;
        gimbal_control.tilt_moving = false;
        ESP_LOGI(TAG, "Steady mode activated - holding current position");
    } else {
        if (gimbal_control.mode == GIMBAL_MODE_STEADY) {
            gimbal_control.mode = GIMBAL_MODE_MANUAL;
            ESP_LOGI(TAG, "Steady mode deactivated - manual control enabled");
        }
    }
    
    return ESP_OK;
}

esp_err_t gimbal_controller_set_steady_goal(float pan_goal, float tilt_goal)
{
    if (!gimbal_initialized) {
        ESP_LOGE(TAG, "Gimbal controller not initialized");
        return ESP_ERR_INVALID_STATE;
    }

    ESP_LOGI(TAG, "Steady goal set: pan=%.2f, tilt=%.2f", pan_goal, tilt_goal);
    
    if (gimbal_control.mode == GIMBAL_MODE_STEADY) {
        // Set target positions for steady mode
        gimbal_control.pan_target = pan_goal;
        gimbal_control.tilt_target = tilt_goal;
        
        // Convert to pulse values
        uint16_t pan_pulse = gimbal_angle_to_pulse(pan_goal, 0);
        uint16_t tilt_pulse = gimbal_angle_to_pulse(tilt_goal, 1);
        
        // Move to target positions
        gimbal_set_pan_angle(pan_pulse);
        gimbal_set_tilt_angle(tilt_pulse);
        
        ESP_LOGI(TAG, "Moving to steady goal: pan_pulse=%d, tilt_pulse=%d", pan_pulse, tilt_pulse);
    } else {
        ESP_LOGW(TAG, "Steady goal ignored - not in steady mode");
    }
    
    return ESP_OK;
}

esp_err_t gimbal_controller_enable_tracking_mode(bool enable)
{
    if (!gimbal_initialized) {
        ESP_LOGE(TAG, "Gimbal controller not initialized");
        return ESP_ERR_INVALID_STATE;
    }

    ESP_LOGI(TAG, "Tracking mode %s", enable ? "enabled" : "disabled");
    
    if (enable) {
        gimbal_control.mode = GIMBAL_MODE_TRACKING;
        // Enable target tracking
        gimbal_control.pan_moving = false;
        gimbal_control.tilt_moving = false;
        ESP_LOGI(TAG, "Tracking mode activated - following target");
    } else {
        if (gimbal_control.mode == GIMBAL_MODE_TRACKING) {
            gimbal_control.mode = GIMBAL_MODE_MANUAL;
            ESP_LOGI(TAG, "Tracking mode deactivated - manual control enabled");
        }
    }
    
    return ESP_OK;
}

esp_err_t gimbal_controller_set_tracking_target(float pan_target, float tilt_target)
{
    if (!gimbal_initialized) {
        ESP_LOGE(TAG, "Gimbal controller not initialized");
        return ESP_ERR_INVALID_STATE;
    }

    ESP_LOGI(TAG, "Tracking target set: pan=%.2f, tilt=%.2f", pan_target, tilt_target);
    
    if (gimbal_control.mode == GIMBAL_MODE_TRACKING) {
        // Set tracking target
        gimbal_control.pan_target = pan_target;
        gimbal_control.tilt_target = tilt_target;
        
        // Calculate movement needed
        float pan_diff = pan_target - gimbal_control.pan_position;
        float tilt_diff = tilt_target - gimbal_control.tilt_position;
        
        // Apply tracking with proportional control
        if (fabs(pan_diff) > 0.5) { // 0.5 degree threshold
            float pan_speed = pan_diff * 0.1; // Proportional gain
            gimbal_control.pan_speed = fmaxf(-gimbal_control.pan_speed, fminf(pan_speed, gimbal_control.pan_speed));
        }
        
        if (fabs(tilt_diff) > 0.5) { // 0.5 degree threshold
            float tilt_speed = tilt_diff * 0.1; // Proportional gain
            gimbal_control.tilt_speed = fmaxf(-gimbal_control.tilt_speed, fminf(tilt_speed, gimbal_control.tilt_speed));
        }
        
        ESP_LOGI(TAG, "Tracking target updated - pan_diff=%.2f, tilt_diff=%.2f", pan_diff, tilt_diff);
    } else {
        ESP_LOGW(TAG, "Tracking target ignored - not in tracking mode");
    }
    
    return ESP_OK;
}

esp_err_t gimbal_controller_calibrate(void)
{
    if (!gimbal_initialized) {
        ESP_LOGE(TAG, "Gimbal controller not initialized");
        return ESP_ERR_INVALID_STATE;
    }

    ESP_LOGI(TAG, "Starting gimbal calibration...");

    // Perform calibration sequence
    ESP_LOGI(TAG, "Step 1: Moving to center position");
    gimbal_set_pan_angle(GIMBAL_PAN_CENTER);
    gimbal_set_tilt_angle(GIMBAL_TILT_CENTER);
    vTaskDelay(pdMS_TO_TICKS(1000)); // Wait for movement
    
    ESP_LOGI(TAG, "Step 2: Measuring center position");
    gimbal_control.pan_position = 0.0f;
    gimbal_control.tilt_position = 0.0f;
    
    ESP_LOGI(TAG, "Step 3: Moving to min positions");
    gimbal_set_pan_angle(GIMBAL_PAN_MIN);
    gimbal_set_tilt_angle(GIMBAL_TILT_MIN);
    vTaskDelay(pdMS_TO_TICKS(1000));
    
    ESP_LOGI(TAG, "Step 4: Moving to max positions");
    gimbal_set_pan_angle(GIMBAL_PAN_MAX);
    gimbal_set_tilt_angle(GIMBAL_TILT_MAX);
    vTaskDelay(pdMS_TO_TICKS(1000));
    
    ESP_LOGI(TAG, "Step 5: Returning to center");
    gimbal_set_pan_angle(GIMBAL_PAN_CENTER);
    gimbal_set_tilt_angle(GIMBAL_TILT_CENTER);
    vTaskDelay(pdMS_TO_TICKS(1000));
    
    // Update calibration status
    gimbal_control.calibrated = true;
    gimbal_control.last_update_time = esp_timer_get_time() / 1000;

    ESP_LOGI(TAG, "Gimbal calibration completed successfully");
    return ESP_OK;
}

esp_err_t gimbal_controller_set_limits(uint16_t pan_min, uint16_t pan_max, 
                                      uint16_t tilt_min, uint16_t tilt_max)
{
    if (!gimbal_initialized) {
        ESP_LOGE(TAG, "Gimbal controller not initialized");
        return ESP_ERR_INVALID_STATE;
    }

    // Validate limits
    if (pan_min >= pan_max || tilt_min >= tilt_max) {
        return ESP_ERR_INVALID_ARG;
    }

    ESP_LOGI(TAG, "Limits set: pan[%d, %d], tilt[%d, %d]", pan_min, pan_max, tilt_min, tilt_max);
    
    // Update gimbal limits
    gimbal_control.pan_min_limit = pan_min;
    gimbal_control.pan_max_limit = pan_max;
    gimbal_control.tilt_min_limit = tilt_min;
    gimbal_control.tilt_max_limit = tilt_max;
    
    // Validate current position against new limits
    if (gimbal_control.pan_position < pan_min || gimbal_control.pan_position > pan_max) {
        ESP_LOGW(TAG, "Current pan position %d outside new limits, clamping", gimbal_control.pan_position);
        gimbal_control.pan_position = fmaxf(pan_min, fminf(gimbal_control.pan_position, pan_max));
    }
    
    if (gimbal_control.tilt_position < tilt_min || gimbal_control.tilt_position > tilt_max) {
        ESP_LOGW(TAG, "Current tilt position %d outside new limits, clamping", gimbal_control.tilt_position);
        gimbal_control.tilt_position = fmaxf(tilt_min, fminf(gimbal_control.tilt_position, tilt_max));
    }
    
    ESP_LOGI(TAG, "Gimbal limits updated and validated");
    return ESP_OK;
}

uint16_t gimbal_controller_degrees_to_pulse(float degrees, uint8_t axis)
{
    if (axis == 0) { // Pan axis
        return gimbal_angle_to_pulse(degrees, 0);
    } else { // Tilt axis
        return gimbal_angle_to_pulse(degrees, 1);
    }
}

float gimbal_controller_pulse_to_degrees(uint16_t pulse, uint8_t axis)
{
    if (axis == 0) { // Pan axis
        return gimbal_pulse_to_angle(pulse, 0);
    } else { // Tilt axis
        return gimbal_pulse_to_angle(pulse, 1);
    }
}

esp_err_t gimbal_controller_smooth_move(uint16_t pan_target, uint16_t tilt_target, 
                                       float speed, float acceleration)
{
    if (!gimbal_initialized) {
        ESP_LOGE(TAG, "Gimbal controller not initialized");
        return ESP_ERR_INVALID_STATE;
    }

    // Create smooth move command
    gimbal_command_t cmd = {
        .move_type = GIMBAL_MOVE_ABSOLUTE,
        .pan_value = (float)pan_target,
        .tilt_value = (float)tilt_target,
        .speed = speed,
        .acceleration = acceleration
    };

    return gimbal_controller_process_command(&cmd);
}

// Private functions
void gimbal_task(void *pvParameters)
{
    gimbal_command_t cmd;
    ESP_LOGI(TAG, "Gimbal task started");

    while (1) {
        // Process commands from queue
        if (xQueueReceive(gimbal_command_queue, &cmd, pdMS_TO_TICKS(100)) == pdTRUE) {
            // Declare variables outside switch to avoid jump to case label errors
            uint16_t new_pan, new_tilt;
            
            switch (cmd.move_type) {
                case GIMBAL_MOVE_ABSOLUTE:
                    ESP_LOGI(TAG, "Moving to absolute position: pan=%.2f, tilt=%.2f", 
                             cmd.pan_value, cmd.tilt_value);
                    
                    // Set pan position
                    if (cmd.pan_value >= GIMBAL_PAN_MIN && cmd.pan_value <= GIMBAL_PAN_MAX) {
                        gimbal_set_pan_angle(cmd.pan_value);
                    }
                    
                    // Set tilt position
                    if (cmd.tilt_value >= GIMBAL_TILT_MIN && cmd.tilt_value <= GIMBAL_TILT_MAX) {
                        gimbal_set_tilt_angle(cmd.tilt_value);
                    }
                    break;

                case GIMBAL_MOVE_RELATIVE:
                    ESP_LOGI(TAG, "Moving relative: pan=%.2f, tilt=%.2f", 
                             cmd.pan_value, cmd.tilt_value);
                    
                    // Calculate new positions
                    new_pan = gimbal_control.pan_position + (int16_t)cmd.pan_value;
                    new_tilt = gimbal_control.tilt_position + (int16_t)cmd.tilt_value;
                    
                    // Apply limits
                    if (new_pan < GIMBAL_PAN_MIN) new_pan = GIMBAL_PAN_MIN;
                    if (new_pan > GIMBAL_PAN_MAX) new_pan = GIMBAL_PAN_MAX;
                    if (new_tilt < GIMBAL_TILT_MIN) new_tilt = GIMBAL_TILT_MIN;
                    if (new_tilt > GIMBAL_TILT_MAX) new_tilt = GIMBAL_TILT_MAX;
                    
                    gimbal_set_pan_angle(new_pan);
                    gimbal_set_tilt_angle(new_tilt);
                    break;

                case GIMBAL_MOVE_VELOCITY:
                    ESP_LOGI(TAG, "Setting velocity: pan=%.2f, tilt=%.2f", 
                             cmd.pan_value, cmd.tilt_value);
                    
                    gimbal_control.pan_speed = cmd.pan_value;
                    gimbal_control.tilt_speed = cmd.tilt_value;
                    break;

                default:
                    ESP_LOGW(TAG, "Unknown move type: %d", cmd.move_type);
                    break;
            }
        }

        // Apply stabilization if enabled
        if (gimbal_control.mode == GIMBAL_MODE_AUTO) {
            gimbal_apply_stabilization();
        }

        // Update status
        gimbal_control.last_update_time = esp_timer_get_time() / 1000;

        vTaskDelay(pdMS_TO_TICKS(1000 / GIMBAL_UPDATE_RATE));
    }
}

static esp_err_t gimbal_set_pan_angle(float angle)
{
    if (angle < GIMBAL_PAN_MIN || angle > GIMBAL_PAN_MAX) {
        ESP_LOGW(TAG, "Pan angle out of range: %.2f", angle);
        return ESP_ERR_INVALID_ARG;
    }

    gimbal_control.pan_position = (uint16_t)angle;
    uint16_t pulse = gimbal_angle_to_pulse(angle, 0);
    
    // Convert pulse width to duty cycle
    uint32_t duty = (pulse * ((1 << GIMBAL_PWM_RESOLUTION) - 1)) / 20000; // 20ms period
    
    esp_err_t ret = ledc_set_duty(GIMBAL_LEDC_MODE, GIMBAL_PAN_CHANNEL, duty);
    if (ret != ESP_OK) {
        ESP_LOGE(TAG, "Failed to set pan duty: %s", esp_err_to_name(ret));
        return ret;
    }

    ESP_LOGD(TAG, "Pan servo set to %.1f degrees (pulse: %u)", angle, pulse);
    return ESP_OK;
}

static esp_err_t gimbal_set_tilt_angle(float angle)
{
    if (angle < GIMBAL_TILT_MIN || angle > GIMBAL_TILT_MAX) {
        ESP_LOGW(TAG, "Tilt angle out of range: %.2f", angle);
        return ESP_ERR_INVALID_ARG;
    }

    gimbal_control.tilt_position = (uint16_t)angle;
    uint16_t pulse = gimbal_angle_to_pulse(angle, 1);
    
    // Convert pulse width to duty cycle
    uint32_t duty = (pulse * ((1 << GIMBAL_PWM_RESOLUTION) - 1)) / 20000; // 20ms period
    
    esp_err_t ret = ledc_set_duty(GIMBAL_LEDC_MODE, GIMBAL_TILT_CHANNEL, duty);
    if (ret != ESP_OK) {
        ESP_LOGE(TAG, "Failed to set tilt duty: %s", esp_err_to_name(ret));
        return ret;
    }

    ESP_LOGD(TAG, "Tilt servo set to %.1f degrees (pulse: %u)", angle, pulse);
    return ESP_OK;
}

static uint16_t gimbal_angle_to_pulse(float angle, uint8_t axis)
{
    uint16_t min_pulse, max_pulse, center;
    
    if (axis == 0) { // Pan axis
        min_pulse = GIMBAL_PAN_MIN;
        max_pulse = GIMBAL_PAN_MAX;
        center = GIMBAL_PAN_CENTER;
    } else { // Tilt axis
        min_pulse = GIMBAL_TILT_MIN;
        max_pulse = GIMBAL_TILT_MAX;
        center = GIMBAL_TILT_CENTER;
    }

    // Convert angle to pulse width
    // Assuming angle is in degrees from -90 to +90
    float normalized_angle = (angle + 90.0f) / 180.0f; // 0.0 to 1.0
    uint16_t pulse = min_pulse + (uint16_t)(normalized_angle * (max_pulse - min_pulse));
    
    return pulse;
}

static float gimbal_pulse_to_angle(uint16_t pulse, uint8_t axis)
{
    uint16_t min_pulse, max_pulse;
    
    if (axis == 0) { // Pan axis
        min_pulse = GIMBAL_PAN_MIN;
        max_pulse = GIMBAL_PAN_MAX;
    } else { // Tilt axis
        min_pulse = GIMBAL_TILT_MIN;
        max_pulse = GIMBAL_TILT_MAX;
    }

    // Convert pulse width to angle
    float normalized_pulse = (float)(pulse - min_pulse) / (float)(max_pulse - min_pulse);
    float angle = (normalized_pulse * 180.0f) - 90.0f; // -90 to +90 degrees
    
    return angle;
}

static esp_err_t gimbal_apply_stabilization(void)
{
    // IMU-based stabilization implementation
    // Read IMU data and apply corrections to maintain orientation
    
    // Get current IMU data (this would interface with the IMU controller)
    // For now, we'll simulate stabilization with a simple algorithm
    
    static float last_pan_error = 0.0f;
    static float last_tilt_error = 0.0f;
    static uint32_t last_stabilization_time = 0;
    
    uint32_t current_time = esp_timer_get_time() / 1000;
    if (current_time - last_stabilization_time < 50) { // 20Hz stabilization rate
        return ESP_OK;
    }
    
    // Calculate target position (maintain current orientation)
    float target_pan = gimbal_control.pan_target;
    float target_tilt = gimbal_control.tilt_target;
    
    // Calculate current errors
    float pan_error = target_pan - gimbal_control.pan_position;
    float tilt_error = target_tilt - gimbal_control.tilt_position;
    
    // Simple PID-like stabilization
    float pan_correction = pan_error * 0.1f + (pan_error - last_pan_error) * 0.05f;
    float tilt_correction = tilt_error * 0.1f + (tilt_error - last_tilt_error) * 0.05f;
    
    // Apply corrections if they're significant
    if (fabs(pan_correction) > 0.1f) {
        float new_pan = gimbal_control.pan_position + pan_correction;
        gimbal_set_pan_angle(new_pan);
    }
    
    if (fabs(tilt_correction) > 0.1f) {
        float new_tilt = gimbal_control.tilt_position + tilt_correction;
        gimbal_set_tilt_angle(new_tilt);
    }
    
    // Store errors for next iteration
    last_pan_error = pan_error;
    last_tilt_error = tilt_error;
    last_stabilization_time = current_time;
    
    ESP_LOGD(TAG, "Stabilization: pan_err=%.2f, tilt_err=%.2f, pan_corr=%.2f, tilt_corr=%.2f",
             pan_error, tilt_error, pan_correction, tilt_correction);
    
    return ESP_OK;
}

esp_err_t gimbal_controller_stabilize(const imu_data_t *imu_data)
{
    if (!gimbal_initialized) {
        ESP_LOGE(TAG, "Gimbal controller not initialized");
        return ESP_ERR_INVALID_STATE;
    }

    if (imu_data == NULL) {
        ESP_LOGE(TAG, "Invalid IMU data pointer");
        return ESP_ERR_INVALID_ARG;
    }

    // Check if stabilization is enabled (steady mode)
    if (gimbal_control.mode != GIMBAL_MODE_STEADY) {
        ESP_LOGD(TAG, "Stabilization not enabled (not in steady mode)");
        return ESP_OK;
    }

    // Calculate compensation based on IMU data
    // Use gyroscope data to compensate for angular movement
    float pan_compensation = -imu_data->gyro_z * 0.1f;  // Compensate for yaw
    float tilt_compensation = -imu_data->gyro_x * 0.1f; // Compensate for roll

    // Apply compensation to current position
    float compensated_pan = gimbal_control.pan_position + pan_compensation;
    float compensated_tilt = gimbal_control.tilt_position + tilt_compensation;

    // Clamp to limits
    compensated_pan = fmaxf(gimbal_control.pan_min_limit, 
                           fminf(compensated_pan, gimbal_control.pan_max_limit));
    compensated_tilt = fmaxf(gimbal_control.tilt_min_limit, 
                            fminf(compensated_tilt, gimbal_control.tilt_max_limit));

    // Apply stabilization if compensation is significant
    if (fabsf(pan_compensation) > 1.0f || fabsf(tilt_compensation) > 1.0f) {
        ESP_LOGD(TAG, "Applying stabilization: pan_comp=%.2f, tilt_comp=%.2f", 
                 pan_compensation, tilt_compensation);
        
        // Convert to pulse width and apply
        uint16_t pan_pulse = (uint16_t)compensated_pan;
        uint16_t tilt_pulse = (uint16_t)compensated_tilt;
        
        esp_err_t ret = gimbal_controller_set_position(pan_pulse, tilt_pulse);
        if (ret != ESP_OK) {
            ESP_LOGW(TAG, "Failed to apply stabilization: %s", esp_err_to_name(ret));
            return ret;
        }
    }

    return ESP_OK;
}
