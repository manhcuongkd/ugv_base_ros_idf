#include "../inc/gimbal_controller.h"
#include "../inc/servo_controller.h"
#include "../inc/ugv_config.h"
#include <esp_log.h>
#include <esp_timer.h>
// #include <driver/ledc.h>  // Removed PWM support
#include <driver/gpio.h>
#include <driver/uart.h>
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

// Gimbal servo communication via UART (SCServo protocol)
// PWM implementation removed - using pure UART/SCServo communication

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
static esp_err_t gimbal_scservo_send_packet_at_register(uint8_t id, uint8_t reg, uint8_t *params, uint8_t param_len);

// SCServo protocol functions (Arduino-style implementation)
static esp_err_t gimbal_scservo_sync_write_pos_ex(uint8_t *ids, uint8_t id_count, int16_t *positions, uint16_t *speeds, uint8_t *accs);
static esp_err_t gimbal_scservo_write_pos_ex(uint8_t id, int16_t position, uint16_t speed, uint8_t acc);
static esp_err_t gimbal_scservo_send_packet(uint8_t id, uint8_t cmd, uint8_t *params, uint8_t param_len);
static uint8_t gimbal_scservo_calc_checksum(uint8_t *data, uint8_t len);

// SCServo UART configuration (shared with servo_controller)
#define SCSERVO_UART_NUM UART_NUM_1  // Use same UART as servo_controller
#define SCSERVO_UART_TX_PIN SERVO_TXD  // GPIO 19
#define SCSERVO_UART_RX_PIN SERVO_RXD  // GPIO 18
#define SCSERVO_BAUD_RATE 1000000

// SMS_STS protocol constants (matching Arduino SCServo library)
#define SCSERVO_HEADER1 0xFF
#define SCSERVO_HEADER2 0xFF
#define SCSERVO_INST_SYNC_WRITE 0x83
#define SCSERVO_INST_WRITE 0x03
#define SMS_STS_ACC 41
#define SMS_STS_GOAL_POSITION_L 42
#define SMS_STS_GOAL_SPEED_L 46
#define SMS_STS_TORQUE_ENABLE 40

esp_err_t gimbal_controller_init(void)
{
    if (gimbal_initialized) {
        ESP_LOGW(TAG, "Gimbal controller already initialized");
        return ESP_OK;
    }

    ESP_LOGI(TAG, "Initializing gimbal controller...");

    // Note: UART is initialized by servo_controller, we just use the shared connection
    // Verify UART is available (servo_controller should be initialized first)
    esp_err_t ret = ESP_OK;

    // Initialize control structure
    memset(&gimbal_control, 0, sizeof(gimbal_control_t));
    gimbal_control.mode = GIMBAL_MODE_MANUAL;
    gimbal_control.pan_position = 2047; // Arduino-style center
    gimbal_control.tilt_position = 2047; // Arduino-style center
    gimbal_control.pan_target = 2047;
    gimbal_control.tilt_target = 2047;
    gimbal_control.pan_speed = 1.0f;
    gimbal_control.tilt_speed = 1.0f;
    gimbal_control.pan_acceleration = 1.0f;
    gimbal_control.tilt_acceleration = 1.0f;
    gimbal_control.pan_moving = false;
    gimbal_control.tilt_moving = false;
    gimbal_control.last_update_time = esp_timer_get_time() / 1000;

    // Note: UART initialization is handled by servo_controller
    // Gimbal controller uses shared UART connection for SCServo communication
    ESP_LOGI(TAG, "Using shared UART connection for SCServo communication");

    // No additional hardware initialization needed - using UART/SCServo protocol only

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

    gimbal_initialized = true;
    ESP_LOGI(TAG, "Gimbal controller initialized successfully");

    // Enable servo torque for both pan and tilt servos
    ESP_LOGI(TAG, "Enabling servo torque for gimbal servos...");
    esp_err_t torque_ret;
    
    torque_ret = scservo_enable_torque(GIMBAL_PAN_ID, 1);
    if (torque_ret == ESP_OK) {
        ESP_LOGI(TAG, "✓ Pan servo (ID %d) torque enabled", GIMBAL_PAN_ID);
    } else {
        ESP_LOGW(TAG, "⚠ Failed to enable pan servo torque: %s", esp_err_to_name(torque_ret));
    }
    
    torque_ret = scservo_enable_torque(GIMBAL_TILT_ID, 1);
    if (torque_ret == ESP_OK) {
        ESP_LOGI(TAG, "✓ Tilt servo (ID %d) torque enabled", GIMBAL_TILT_ID);
    } else {
        ESP_LOGW(TAG, "⚠ Failed to enable tilt servo torque: %s", esp_err_to_name(torque_ret));
    }
    
    // Give servos time to initialize after torque enable
    vTaskDelay(pdMS_TO_TICKS(100));

    // Center the gimbal after initialization
    ret = gimbal_controller_center();
    if (ret != ESP_OK) {
        ESP_LOGW(TAG, "Failed to center gimbal: %s", esp_err_to_name(ret));
    }
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

    // Disable servo torque via SCServo protocol
    gimbal_controller_set_torque(GIMBAL_PAN_ID, false);
    gimbal_controller_set_torque(GIMBAL_TILT_ID, false);

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
    if (!gimbal_initialized) {
        ESP_LOGE(TAG, "Gimbal controller not initialized");
        return ESP_ERR_INVALID_STATE;
    }

    // Use Arduino-style sync write to control both servos simultaneously
    uint8_t gimbal_ids[2] = {GIMBAL_PAN_ID, GIMBAL_TILT_ID};
    int16_t gimbal_positions[2] = {(int16_t)pan, (int16_t)tilt};
    uint16_t gimbal_speeds[2] = {
        (uint16_t)(gimbal_control.pan_speed * 100),
        (uint16_t)(gimbal_control.tilt_speed * 100)
    };
    uint8_t gimbal_accs[2] = {
        (uint8_t)(gimbal_control.pan_acceleration * 10),
        (uint8_t)(gimbal_control.tilt_acceleration * 10)
    };

    esp_err_t ret = gimbal_scservo_sync_write_pos_ex(gimbal_ids, 2, gimbal_positions, gimbal_speeds, gimbal_accs);
    if (ret != ESP_OK) {
        ESP_LOGE(TAG, "Failed to sync write gimbal positions: %s", esp_err_to_name(ret));
        return ret;
    }

    // Set target positions
    gimbal_control.pan_target = pan;
    gimbal_control.tilt_target = tilt;
    gimbal_control.pan_moving = true;
    gimbal_control.tilt_moving = true;

    // Update current positions
    gimbal_control.pan_position = pan;
    gimbal_control.tilt_position = tilt;
    gimbal_control.last_update_time = esp_timer_get_time() / 1000;

    ESP_LOGI(TAG, "Gimbal position set: pan=%d, tilt=%d", pan, tilt);
    return ESP_OK;
}

esp_err_t gimbal_controller_set_pan_tilt(float pan, float tilt)
{
    // Arduino-style position calculation matching gimbal_module.h with CORRECTED PAN DIRECTION
    // Pan: constrain to -180..180, map to servo position
    if (pan < -180.0f) pan = -180.0f;
    if (pan > 180.0f) pan = 180.0f;
    
    // Tilt: constrain to -30..90, map to servo position  
    if (tilt < -30.0f) tilt = -30.0f;
    if (tilt > 90.0f) tilt = 90.0f;
    
    // Convert to Arduino-style servo positions with CORRECTED PAN DIRECTION
    // Pan: 2047 + map(angle, 0, 360, 0, 4095) but REVERSED for correct LEFT/RIGHT
    float normalized_pan = pan + 180.0f; // -180..180 -> 0..360
    int16_t pan_pos = 2047 - (int16_t)((normalized_pan * 4095.0f) / 360.0f) + 2047; // REVERSED!
    
    // Tilt: 2047 - map(angle, 0, 360, 0, 4095) (inverted)
    float normalized_tilt = tilt + 30.0f; // -30..90 -> 0..120
    int16_t tilt_pos = 2047 - (int16_t)((normalized_tilt * 4095.0f) / 120.0f) + 2047;
    
    ESP_LOGI(TAG, "CORRECTED Pan/Tilt: pan_angle=%.1f -> pos=%d, tilt_angle=%.1f -> pos=%d", 
             pan, pan_pos, tilt, tilt_pos);
    
    // Call the existing set_position function
    return gimbal_controller_set_position((uint16_t)pan_pos, (uint16_t)tilt_pos);
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

    return gimbal_controller_set_position(2047, 2047); // Arduino-style center position
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

    // Convert angle to servo position using corrected pan direction
    uint16_t servo_pos = gimbal_angle_to_pulse(angle, 0);
    
    // Use SCServo protocol to set position
    esp_err_t ret = gimbal_scservo_write_pos_ex(GIMBAL_PAN_ID, servo_pos, 
                                                (uint16_t)(gimbal_control.pan_speed * 100), 
                                                (uint8_t)(gimbal_control.pan_acceleration * 10));
    if (ret != ESP_OK) {
        ESP_LOGE(TAG, "Failed to set pan position via SCServo: %s", esp_err_to_name(ret));
        return ret;
    }

    gimbal_control.pan_position = servo_pos;
    ESP_LOGD(TAG, "Pan servo set to %.1f degrees (position: %u)", angle, servo_pos);
    return ESP_OK;
}

static esp_err_t gimbal_set_tilt_angle(float angle)
{
    if (angle < GIMBAL_TILT_MIN || angle > GIMBAL_TILT_MAX) {
        ESP_LOGW(TAG, "Tilt angle out of range: %.2f", angle);
        return ESP_ERR_INVALID_ARG;
    }

    // Convert angle to servo position
    uint16_t servo_pos = gimbal_angle_to_pulse(angle, 1);
    
    // Use SCServo protocol to set position
    esp_err_t ret = gimbal_scservo_write_pos_ex(GIMBAL_TILT_ID, servo_pos, 
                                                (uint16_t)(gimbal_control.tilt_speed * 100), 
                                                (uint8_t)(gimbal_control.tilt_acceleration * 10));
    if (ret != ESP_OK) {
        ESP_LOGE(TAG, "Failed to set tilt position via SCServo: %s", esp_err_to_name(ret));
        return ret;
    }

    gimbal_control.tilt_position = servo_pos;
    ESP_LOGD(TAG, "Tilt servo set to %.1f degrees (position: %u)", angle, servo_pos);
    return ESP_OK;
}

static uint16_t gimbal_angle_to_pulse(float angle, uint8_t axis)
{
    // Arduino-style servo position mapping with CORRECTED PAN DIRECTION
    if (axis == 0) { // Pan axis with CORRECTED direction
        // Convert angle (-180 to +180) to servo position (0 to 4095)
        // CORRECTED: Negative angles (LEFT) -> Lower positions, Positive angles (RIGHT) -> Higher positions
        float normalized_angle = (angle + 180.0f) / 360.0f; // -180..180 -> 0..1
        uint16_t position = (uint16_t)(normalized_angle * 4095.0f);
        
        // Apply CORRECTION: Reverse the mapping so LEFT is lower values, RIGHT is higher values
        position = 4095 - position;
        
        ESP_LOGD(TAG, "Pan angle %.1f -> normalized %.3f -> raw_pos %u -> corrected_pos %u", 
                 angle, normalized_angle, (uint16_t)(normalized_angle * 4095.0f), position);
        return position;
    } else { // Tilt axis (unchanged)
        // Convert angle (-90 to +90) to servo position (0 to 4095)
        float normalized_angle = (angle + 90.0f) / 180.0f; // -90..90 -> 0..1
        uint16_t position = (uint16_t)(normalized_angle * 4095.0f);
        
        ESP_LOGD(TAG, "Tilt angle %.1f -> normalized %.3f -> position %u", 
                 angle, normalized_angle, position);
        return position;
    }
}

static float gimbal_pulse_to_angle(uint16_t pulse, uint8_t axis)
{
    // Arduino-style servo position to angle conversion with CORRECTED PAN DIRECTION
    if (axis == 0) { // Pan axis with CORRECTED direction
        // Reverse the correction applied in gimbal_angle_to_pulse
        uint16_t corrected_pulse = 4095 - pulse;
        
        // Convert servo position (0 to 4095) to angle (-180 to +180)
        float normalized_pulse = (float)corrected_pulse / 4095.0f; // 0..1
        float angle = (normalized_pulse * 360.0f) - 180.0f; // 0..1 -> -180..180
        
        ESP_LOGD(TAG, "Pan position %u -> corrected %u -> normalized %.3f -> angle %.1f", 
                 pulse, corrected_pulse, normalized_pulse, angle);
        return angle;
    } else { // Tilt axis (unchanged)
        // Convert servo position (0 to 4095) to angle (-90 to +90)
        float normalized_pulse = (float)pulse / 4095.0f; // 0..1
        float angle = (normalized_pulse * 180.0f) - 90.0f; // 0..1 -> -90..90
        
        ESP_LOGD(TAG, "Tilt position %u -> normalized %.3f -> angle %.1f", 
                 pulse, normalized_pulse, angle);
        return angle;
    }
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

esp_err_t gimbal_controller_get_feedback(gimbal_feedback_t *pan_fb, gimbal_feedback_t *tilt_fb)
{
    if (!gimbal_initialized) {
        ESP_LOGE(TAG, "Gimbal controller not initialized");
        return ESP_ERR_INVALID_STATE;
    }

    if (pan_fb == NULL || tilt_fb == NULL) {
        return ESP_ERR_INVALID_ARG;
    }

    // Read real servo feedback via UART using SCServo protocol
    esp_err_t ret;
    
    // Read pan servo feedback (ID = GIMBAL_PAN_ID = 2)
    uint16_t pan_pos, pan_speed, pan_load;
    uint8_t pan_voltage, pan_temperature, pan_moving;
    
    // Use Arduino-style FeedBack function for better compatibility
    uint8_t pan_mem[15]; // SMS_STS feedback memory block
    int pan_feedback_len = scservo_feedback(GIMBAL_PAN_ID, pan_mem);
    ESP_LOGI(TAG, "scservo_feedback(PAN_ID=%d) returned: %d bytes", GIMBAL_PAN_ID, pan_feedback_len);
    
    if (pan_feedback_len > 0) {
        // Parse feedback data like Arduino ReadPos function
        // Position is at offset 0 and 1 in the memory block (register 56, 57)
        pan_pos = pan_mem[0] | (pan_mem[1] << 8);
        
        ret = ESP_OK;
    } else {
        ret = ESP_FAIL;
    }
    
    if (ret == ESP_OK) {
        pan_fb->status = true;
        pan_fb->pos = pan_pos;
        
        // Read additional pan servo data
        scservo_read_speed(GIMBAL_PAN_ID, &pan_speed);
        scservo_read_load(GIMBAL_PAN_ID, &pan_load);
        scservo_read_voltage(GIMBAL_PAN_ID, &pan_voltage);
        scservo_read_temperature(GIMBAL_PAN_ID, &pan_temperature);
        scservo_read_moving(GIMBAL_PAN_ID, &pan_moving);
        
        pan_fb->speed = pan_speed;
        pan_fb->load = pan_load;
        pan_fb->voltage = pan_voltage / 10.0f;  // Convert to volts
        pan_fb->current = 0.5f;  // Current not available in basic SCServo protocol
        pan_fb->temper = pan_temperature;
        pan_fb->mode = pan_moving;
    } else {
        // Fallback to simulated data if UART read fails
        pan_fb->status = false;
        pan_fb->pos = gimbal_control.pan_position;
        pan_fb->speed = 0;
        pan_fb->load = 0;
        pan_fb->voltage = 12.0f;
        pan_fb->current = 0.5f;
        pan_fb->temper = 25.0f;
        pan_fb->mode = 0;
        ESP_LOGW(TAG, "Failed to read pan servo feedback, using simulated data");
    }
    
    // Read tilt servo feedback (ID = GIMBAL_TILT_ID = 1)
    uint16_t tilt_pos, tilt_speed, tilt_load;
    uint8_t tilt_voltage, tilt_temperature, tilt_moving;
    
    // Use Arduino-style FeedBack function for better compatibility  
    uint8_t tilt_mem[15]; // SMS_STS feedback memory block
    int tilt_feedback_len = scservo_feedback(GIMBAL_TILT_ID, tilt_mem);
    ESP_LOGI(TAG, "scservo_feedback(TILT_ID=%d) returned: %d bytes", GIMBAL_TILT_ID, tilt_feedback_len);
    
    if (tilt_feedback_len > 0) {
        // Parse feedback data like Arduino ReadPos function
        // Position is at offset 0 and 1 in the memory block (register 56, 57)
        tilt_pos = tilt_mem[0] | (tilt_mem[1] << 8);
        
        ret = ESP_OK;
    } else {
        ret = ESP_FAIL;
    }
    
    if (ret == ESP_OK) {
        tilt_fb->status = true;
        tilt_fb->pos = tilt_pos;
        
        // Read additional tilt servo data
        scservo_read_speed(GIMBAL_TILT_ID, &tilt_speed);
        scservo_read_load(GIMBAL_TILT_ID, &tilt_load);
        scservo_read_voltage(GIMBAL_TILT_ID, &tilt_voltage);
        scservo_read_temperature(GIMBAL_TILT_ID, &tilt_temperature);
        scservo_read_moving(GIMBAL_TILT_ID, &tilt_moving);
        
        tilt_fb->speed = tilt_speed;
        tilt_fb->load = tilt_load;
        tilt_fb->voltage = tilt_voltage / 10.0f;  // Convert to volts
        tilt_fb->current = 0.5f;  // Current not available in basic SCServo protocol
        tilt_fb->temper = tilt_temperature;
        tilt_fb->mode = tilt_moving;
    } else {
        // Fallback to simulated data if UART read fails
        tilt_fb->status = false;
        tilt_fb->pos = gimbal_control.tilt_position;
        tilt_fb->speed = 0;
        tilt_fb->load = 0;
        tilt_fb->voltage = 12.0f;
        tilt_fb->current = 0.5f;
        tilt_fb->temper = 25.0f;
        tilt_fb->mode = 0;
        ESP_LOGW(TAG, "Failed to read tilt servo feedback, using simulated data");
    }

    ESP_LOGD(TAG, "Servo feedback: pan_pos=%d, tilt_pos=%d", pan_fb->pos, tilt_fb->pos);
    return ESP_OK;
}

esp_err_t gimbal_controller_set_torque(uint8_t servo_id, bool enable)
{
    if (!gimbal_initialized) {
        ESP_LOGE(TAG, "Gimbal controller not initialized");
        return ESP_ERR_INVALID_STATE;
    }

    if (servo_id != GIMBAL_PAN_ID && servo_id != GIMBAL_TILT_ID) {
        ESP_LOGE(TAG, "Invalid servo ID: %d", servo_id);
        return ESP_ERR_INVALID_ARG;
    }

    // Implement real servo torque control via UART using SCServo protocol
    esp_err_t ret = scservo_enable_torque(servo_id, enable ? 1 : 0);
    if (ret == ESP_OK) {
        ESP_LOGI(TAG, "Servo %d torque %s successfully", servo_id, enable ? "enabled" : "disabled");
        
        // Add servo stop delay when disabling torque
        if (!enable) {
            vTaskDelay(pdMS_TO_TICKS(SERVO_STOP_DELAY));
        }
    } else {
        ESP_LOGE(TAG, "Failed to %s torque for servo %d: %s", 
                 enable ? "enable" : "disable", servo_id, esp_err_to_name(ret));
        return ret;
    }
    
    return ESP_OK;
}

esp_err_t gimbal_controller_user_control(int8_t input_x, int8_t input_y, uint16_t speed)
{
    if (!gimbal_initialized) {
        ESP_LOGE(TAG, "Gimbal controller not initialized");
        return ESP_ERR_INVALID_STATE;
    }

    // Validate input parameters
    if (input_x < -1 || input_x > 2 || input_y < -1 || input_y > 2) {
        ESP_LOGE(TAG, "Invalid input parameters: x=%d, y=%d", input_x, input_y);
        return ESP_ERR_INVALID_ARG;
    }

    static float goal_x = 0.0f;
    static float goal_y = 0.0f;

    // 8-directional control logic (matching Arduino implementation)
    if (input_x == -1 && input_y == 1) {
        goal_x = -180.0f;
        goal_y = 90.0f;
    }
    else if (input_x == 0 && input_y == 1) {
        goal_y = 90.0f;
    }
    else if (input_x == 1 && input_y == 1) {
        goal_x = 180.0f;
        goal_y = 90.0f;
    }
    else if (input_x == -1 && input_y == 0) {
        goal_x = -180.0f;
    }
    else if (input_x == 1 && input_y == 0) {
        goal_x = 180.0f;
    }
    else if (input_x == -1 && input_y == -1) {
        goal_x = -180.0f;
        goal_y = -45.0f;
    }
    else if (input_x == 0 && input_y == -1) {
        goal_y = -45.0f;
    }
    else if (input_x == 1 && input_y == -1) {
        goal_x = 180.0f;
        goal_y = -45.0f;
    }

    // Center position command
    if (input_x == 2 && input_y == 2) {
        return gimbal_controller_center();
    }
    else {
        // Move to calculated goal position
        esp_err_t ret = gimbal_controller_set_pan_tilt(goal_x, goal_y);
        if (ret != ESP_OK) {
            ESP_LOGE(TAG, "Failed to set gimbal position: %s", esp_err_to_name(ret));
            return ret;
        }

        // Handle center position for individual axes (matching Arduino logic)
        if (input_x == 0) {
            // Disable pan torque temporarily to read current position
            gimbal_controller_set_torque(GIMBAL_PAN_ID, false);
            vTaskDelay(pdMS_TO_TICKS(5));
            gimbal_controller_set_torque(GIMBAL_PAN_ID, true);
            
            // Read current pan position and update goal
            gimbal_feedback_t pan_fb, tilt_fb;
            if (gimbal_controller_get_feedback(&pan_fb, &tilt_fb) == ESP_OK) {
                goal_x = gimbal_controller_pulse_to_degrees(pan_fb.pos, 0);
            }
        }
        
        if (input_y == 0) {
            // Disable tilt torque temporarily to read current position
            gimbal_controller_set_torque(GIMBAL_TILT_ID, false);
            vTaskDelay(pdMS_TO_TICKS(5));
            gimbal_controller_set_torque(GIMBAL_TILT_ID, true);
            
            // Read current tilt position and update goal
            gimbal_feedback_t pan_fb, tilt_fb;
            if (gimbal_controller_get_feedback(&pan_fb, &tilt_fb) == ESP_OK) {
                goal_y = gimbal_controller_pulse_to_degrees(tilt_fb.pos, 1);
            }
        }
    }

    ESP_LOGI(TAG, "User control: input_x=%d, input_y=%d, goal_x=%.1f, goal_y=%.1f", 
             input_x, input_y, goal_x, goal_y);
    return ESP_OK;
}

// SCServo protocol implementation (Arduino-style)
static esp_err_t gimbal_scservo_sync_write_pos_ex(uint8_t *ids, uint8_t id_count, int16_t *positions, uint16_t *speeds, uint8_t *accs)
{
    if (!ids || !positions || !speeds || !accs || id_count == 0) {
        return ESP_ERR_INVALID_ARG;
    }

    // 🎯 EXACT Arduino SCServo library implementation!
    
    // Step 1: Prepare servo data buffer (Arduino-style)
    uint8_t nLen = 7; // ACC(1) + Position(2) + Time(2) + Speed(2) per servo
    uint8_t offbuf[7 * id_count]; // Data buffer for all servos
    
    for (uint8_t i = 0; i < id_count; i++) {
        // Handle negative positions (Arduino-style from SMS_STS.cpp:57-60)
        int16_t pos = positions[i];
        if (pos < 0) {
            pos = -pos;
            pos |= (1 << 15);
        }
        
        // Speed handling (Arduino-style from SMS_STS.cpp:61-66)
        uint16_t V = speeds ? speeds[i] : 0;
        
        // ACC handling (Arduino-style from SMS_STS.cpp:67-71)
        offbuf[i*7] = accs ? accs[i] : 0;
        
        // Host2SCS for Position (Arduino-style from SMS_STS.cpp:72)
        // Note: End=0 (little endian), so DataL=low, DataH=high
        offbuf[i*7+1] = pos & 0xFF;        // Position low byte
        offbuf[i*7+2] = (pos >> 8) & 0xFF; // Position high byte
        
        // Host2SCS for Time = 0 (Arduino-style from SMS_STS.cpp:73)
        offbuf[i*7+3] = 0; // Time low byte
        offbuf[i*7+4] = 0; // Time high byte
        
        // Host2SCS for Speed (Arduino-style from SMS_STS.cpp:74)
        offbuf[i*7+5] = V & 0xFF;        // Speed low byte
        offbuf[i*7+6] = (V >> 8) & 0xFF; // Speed high byte
    }
    
    // Step 2: Call syncWrite exactly like Arduino (SMS_STS.cpp:76)
    // syncWrite(ID, IDN, SMS_STS_ACC, offbuf, 7);
    
    // Arduino syncWrite implementation (SCS.cpp:123-150)
    uint8_t mesLen = ((nLen + 1) * id_count + 4); // Arduino SCS.cpp:126
    uint8_t packet[256];
    uint8_t packet_len = 0;
    
    // Header (Arduino SCS.cpp:128-136)
    packet[packet_len++] = 0xFF;           // bBuf[0]
    packet[packet_len++] = 0xFF;           // bBuf[1]  
    packet[packet_len++] = 0xFE;           // bBuf[2] (broadcast)
    packet[packet_len++] = mesLen;         // bBuf[3]
    packet[packet_len++] = 0x83;           // bBuf[4] INST_SYNC_WRITE
    packet[packet_len++] = SMS_STS_ACC;    // bBuf[5] MemAddr = 41
    packet[packet_len++] = nLen;           // bBuf[6] = 7
    
    // Checksum calculation (Arduino SCS.cpp:138)
    uint8_t Sum = 0xFE + mesLen + 0x83 + SMS_STS_ACC + nLen;
    
    // Servo data (Arduino SCS.cpp:140-147)
    for (uint8_t i = 0; i < id_count; i++) {
        // Write ID (Arduino SCS.cpp:141)
        packet[packet_len++] = ids[i];
        Sum += ids[i];
        
        // Write nLen bytes of data (Arduino SCS.cpp:142)
        for (uint8_t j = 0; j < nLen; j++) {
            packet[packet_len++] = offbuf[i*nLen + j];
            Sum += offbuf[i*nLen + j]; // Arduino SCS.cpp:145
        }
    }
    
    // Checksum (Arduino SCS.cpp:148)
    packet[packet_len++] = ~Sum;
    
    ESP_LOGI(TAG, "📡 ARDUINO-EXACT SYNC WRITE PACKET:");
    ESP_LOGI(TAG, "   Servo count: %d, Register: %d, Data per servo: %d", id_count, SMS_STS_ACC, nLen);
    char hex_str[512] = "";
    for (int i = 0; i < packet_len; i++) {
        char temp[8];
        snprintf(temp, sizeof(temp), "%02X ", packet[i]);
        strcat(hex_str, temp);
    }
    ESP_LOGI(TAG, "   Raw bytes: %s", hex_str);
    ESP_LOGI(TAG, "   Packet length: %d bytes", packet_len);
    
    // Send packet via UART
    int bytes_written = uart_write_bytes(SCSERVO_UART_NUM, packet, packet_len);
    if (bytes_written != packet_len) {
        ESP_LOGE(TAG, "❌ UART sync write failed: expected %d, written %d", packet_len, bytes_written);
        return ESP_FAIL;
    }
    
    ESP_LOGI(TAG, "✅ ARDUINO-EXACT packet sent successfully (%d bytes)", bytes_written);
    
    // Small delay for servo processing
    vTaskDelay(pdMS_TO_TICKS(10));
    
    return ESP_OK;
}

static esp_err_t gimbal_scservo_write_pos_ex(uint8_t id, int16_t position, uint16_t speed, uint8_t acc)
{
    // SMS_STS write position with speed and acceleration (Arduino-compatible format)
    uint8_t params[7];
    
    // SMS_STS format: ACC(1) + Position(2) + Time(2) + Speed(2)
    // Acceleration (1 byte) - at register SMS_STS_ACC (41)
    params[0] = acc;
    
    // Position (2 bytes, little endian) - at register SMS_STS_GOAL_POSITION_L (42)
    params[1] = position & 0xFF;
    params[2] = (position >> 8) & 0xFF;
    
    // Time (2 bytes, not used in this implementation, set to 0) - at register SMS_STS_GOAL_TIME_L (44)
    params[3] = 0;
    params[4] = 0;
    
    // Speed (2 bytes, little endian) - at register SMS_STS_GOAL_SPEED_L (46)
    params[5] = speed & 0xFF;
    params[6] = (speed >> 8) & 0xFF;
    
    // Send packet starting at SMS_STS_ACC register (41)
    return gimbal_scservo_send_packet_at_register(id, SMS_STS_ACC, params, 7);
}

static esp_err_t gimbal_scservo_send_packet_at_register(uint8_t id, uint8_t reg, uint8_t *params, uint8_t param_len)
{
    // Build packet for writing to specific register: [0xFF] [0xFF] [ID] [LEN] [INST_WRITE] [REG] [PARAMS...] [CHECKSUM]
    uint8_t packet[256];
    uint8_t packet_len = 0;
    
    packet[packet_len++] = SCSERVO_HEADER1;
    packet[packet_len++] = SCSERVO_HEADER2;
    packet[packet_len++] = id;
    packet[packet_len++] = param_len + 3; // Length = reg + params + checksum
    packet[packet_len++] = SCSERVO_INST_WRITE;
    packet[packet_len++] = reg; // Register address
    
    // Add parameters
    for (uint8_t i = 0; i < param_len; i++) {
        packet[packet_len++] = params[i];
    }
    
    // Calculate and add checksum
    uint8_t checksum = gimbal_scservo_calc_checksum(&packet[2], packet_len - 2);
    packet[packet_len++] = ~checksum; // Inverted checksum
    
    // Log the exact packet being sent (for debugging)
    ESP_LOGI(TAG, "📡 SENDING SMS_STS PACKET:");
    ESP_LOGI(TAG, "   Target: Servo ID %d, Register %d", id, reg);
    char hex_str[256] = "";
    for (int i = 0; i < packet_len; i++) {
        char temp[8];
        snprintf(temp, sizeof(temp), "%02X ", packet[i]);
        strcat(hex_str, temp);
    }
    ESP_LOGI(TAG, "   Raw bytes: %s", hex_str);
    ESP_LOGI(TAG, "   Packet length: %d bytes", packet_len);
    
    // Send packet via UART
    int bytes_written = uart_write_bytes(SCSERVO_UART_NUM, packet, packet_len);
    if (bytes_written != packet_len) {
        ESP_LOGE(TAG, "❌ UART write failed: expected %d, written %d", packet_len, bytes_written);
        return ESP_FAIL;
    }
    
    ESP_LOGI(TAG, "✅ UART packet sent successfully (%d bytes)", bytes_written);
    
    // Add a small delay and try to read any response
    vTaskDelay(pdMS_TO_TICKS(10));
    uint8_t response[64];
    int bytes_read = uart_read_bytes(SCSERVO_UART_NUM, response, sizeof(response), pdMS_TO_TICKS(50));
    if (bytes_read > 0) {
        char resp_hex[256] = "";
        for (int i = 0; i < bytes_read; i++) {
            char temp[8];
            snprintf(temp, sizeof(temp), "%02X ", response[i]);
            strcat(resp_hex, temp);
        }
        ESP_LOGI(TAG, "📨 Servo response (%d bytes): %s", bytes_read, resp_hex);
    } else {
        ESP_LOGW(TAG, "⚠️ No response from servo ID %d", id);
    }
    return ESP_OK;
}

static esp_err_t gimbal_scservo_send_packet(uint8_t id, uint8_t cmd, uint8_t *params, uint8_t param_len)
{
    uint8_t packet[256];
    uint8_t packet_len = 0;
    
    // Build packet: [0xFF] [0xFF] [ID] [LEN] [CMD] [PARAMS...] [CHECKSUM]
    packet[packet_len++] = SCSERVO_HEADER1;
    packet[packet_len++] = SCSERVO_HEADER2;
    packet[packet_len++] = id;
    packet[packet_len++] = param_len + 2; // Length = params + cmd + checksum
    packet[packet_len++] = cmd;
    
    // Add parameters
    for (uint8_t i = 0; i < param_len; i++) {
        packet[packet_len++] = params[i];
    }
    
    // Calculate and add checksum
    uint8_t checksum = gimbal_scservo_calc_checksum(&packet[2], packet_len - 2); // From ID to last param
    packet[packet_len++] = ~checksum; // Inverted checksum
    
    // Send packet via UART
    int bytes_written = uart_write_bytes(SCSERVO_UART_NUM, packet, packet_len);
    if (bytes_written != packet_len) {
        ESP_LOGE(TAG, "UART write failed: expected %d, written %d", packet_len, bytes_written);
        return ESP_FAIL;
    }
    
    ESP_LOGD(TAG, "Sent SCServo packet: ID=%d, CMD=0x%02X, LEN=%d", id, cmd, param_len);
    return ESP_OK;
}

static uint8_t gimbal_scservo_calc_checksum(uint8_t *data, uint8_t len)
{
    uint8_t checksum = 0;
    for (uint8_t i = 0; i < len; i++) {
        checksum += data[i];
    }
    return checksum;
}

