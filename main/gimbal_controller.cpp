#include "../inc/gimbal_controller.h"
#include "../inc/servo_controller.h"
#include "../inc/ugv_config.h"
#include <esp_log.h>
#include <esp_timer.h>
#include <driver/uart.h>
#include <freertos/FreeRTOS.h>
#include <freertos/task.h>
#include <freertos/queue.h>
#include <math.h>
#include <string.h>

static const char *TAG = "Gimbal_Controller";

// Global State
static bool gimbal_initialized = false;
static gimbal_control_t gimbal_control;
static QueueHandle_t gimbal_command_queue = NULL;
static TaskHandle_t gimbal_task_handle = NULL;

// Private Function Prototypes
static void gimbal_task(void *pvParameters);
static esp_err_t gimbal_send_sync_write_command(uint8_t *ids, uint8_t id_count, 
                                               int16_t *positions, uint16_t *speeds, uint8_t *accs);
static esp_err_t gimbal_validate_position(uint16_t pan, uint16_t tilt);
static esp_err_t gimbal_validate_speed(float speed);
static esp_err_t gimbal_validate_acceleration(float acceleration);
static uint16_t gimbal_angle_to_position(float angle, uint8_t axis);
static float gimbal_position_to_angle(uint16_t position, uint8_t axis);
static void gimbal_log_movement(const char* direction, uint16_t pan, uint16_t tilt);
static esp_err_t gimbal_apply_stabilization_compensation(const imu_data_t *imu_data);

// =============================================================================
// PUBLIC API FUNCTIONS
// =============================================================================

esp_err_t gimbal_controller_init(void)
{
    if (gimbal_initialized) {
        ESP_LOGW(TAG, "Gimbal controller already initialized");
        return ESP_OK;
    }

    ESP_LOGI(TAG, "Initializing gimbal controller...");

    // Initialize control structure with safe defaults
    memset(&gimbal_control, 0, sizeof(gimbal_control_t));
    gimbal_control.mode = GIMBAL_MODE_MANUAL;
    gimbal_control.pan_position = GIMBAL_CENTER_POSITION;
    gimbal_control.tilt_position = GIMBAL_CENTER_POSITION;
    gimbal_control.pan_target = GIMBAL_CENTER_POSITION;
    gimbal_control.tilt_target = GIMBAL_CENTER_POSITION;
    gimbal_control.pan_speed = 1.0f;
    gimbal_control.tilt_speed = 1.0f;
    gimbal_control.pan_acceleration = 1.0f;
    gimbal_control.tilt_acceleration = 1.0f;
    gimbal_control.pan_moving = false;
    gimbal_control.tilt_moving = false;
    gimbal_control.last_update_time = esp_timer_get_time() / 1000;

    // Create command queue
    gimbal_command_queue = xQueueCreate(GIMBAL_QUEUE_SIZE, sizeof(gimbal_command_t));
    if (gimbal_command_queue == NULL) {
        ESP_LOGE(TAG, "Failed to create gimbal command queue");
        return ESP_ERR_NO_MEM;
    }

    // Create gimbal processing task
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
        gimbal_command_queue = NULL;
        return ESP_ERR_NO_MEM;
    }

    gimbal_initialized = true;
    ESP_LOGI(TAG, "Gimbal controller initialized successfully");

    // Center the gimbal as part of initialization
    esp_err_t ret = gimbal_controller_center();
    if (ret != ESP_OK) {
        ESP_LOGW(TAG, "Failed to center gimbal during initialization: %s", esp_err_to_name(ret));
    }

    return ESP_OK;
}

esp_err_t gimbal_controller_deinit(void)
{
    if (!gimbal_initialized) {
        return ESP_OK;
    }

    ESP_LOGI(TAG, "Deinitializing gimbal controller...");

    // Clean up task
    if (gimbal_task_handle != NULL) {
        vTaskDelete(gimbal_task_handle);
        gimbal_task_handle = NULL;
    }

    // Clean up queue
    if (gimbal_command_queue != NULL) {
        vQueueDelete(gimbal_command_queue);
        gimbal_command_queue = NULL;
    }

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
        ESP_LOGE(TAG, "Invalid gimbal mode: %d", mode);
        return ESP_ERR_INVALID_ARG;
    }

    gimbal_control.mode = mode;
    ESP_LOGI(TAG, "Gimbal mode set to %d", mode);
    return ESP_OK;
}

esp_err_t gimbal_controller_center(void)
{
    if (!gimbal_initialized) {
        ESP_LOGE(TAG, "Gimbal controller not initialized");
        return ESP_ERR_INVALID_STATE;
    }

    ESP_LOGI(TAG, "🎯 Centering gimbal to position %d, %d", 
             GIMBAL_CENTER_POSITION, GIMBAL_CENTER_POSITION);
    
    return gimbal_controller_set_position(GIMBAL_CENTER_POSITION, GIMBAL_CENTER_POSITION);
}

esp_err_t gimbal_controller_set_position(uint16_t pan, uint16_t tilt)
{
    if (!gimbal_initialized) {
        ESP_LOGE(TAG, "Gimbal controller not initialized");
        return ESP_ERR_INVALID_STATE;
    }

    // Validate position limits
    esp_err_t ret = gimbal_validate_position(pan, tilt);
    if (ret != ESP_OK) {
        return ret;
    }

    // Prepare servo command data
    uint8_t servo_ids[2] = {GIMBAL_PAN_ID, GIMBAL_TILT_ID};
    int16_t positions[2] = {(int16_t)pan, (int16_t)tilt};
    uint16_t speeds[2] = {
        (uint16_t)(gimbal_control.pan_speed * 100),
        (uint16_t)(gimbal_control.tilt_speed * 100)
    };
    uint8_t accelerations[2] = {
        (uint8_t)(gimbal_control.pan_acceleration * 10),
        (uint8_t)(gimbal_control.tilt_acceleration * 10)
    };

    // Send command to servos
    ret = gimbal_send_sync_write_command(servo_ids, 2, positions, speeds, accelerations);
    if (ret != ESP_OK) {
        ESP_LOGE(TAG, "Failed to send position command: %s", esp_err_to_name(ret));
        return ret;
    }

    // Update state
    gimbal_control.pan_target = pan;
    gimbal_control.tilt_target = tilt;
    gimbal_control.pan_position = pan;
    gimbal_control.tilt_position = tilt;
    gimbal_control.pan_moving = true;
    gimbal_control.tilt_moving = true;
    gimbal_control.last_update_time = esp_timer_get_time() / 1000;

    ESP_LOGI(TAG, "Gimbal position set: pan=%d, tilt=%d", pan, tilt);
    return ESP_OK;
}

esp_err_t gimbal_controller_set_pan_tilt(float pan_angle, float tilt_angle)
{
    // Clamp angles to valid ranges
    pan_angle = fmaxf(PAN_ANGLE_MIN, fminf(pan_angle, PAN_ANGLE_MAX));
    tilt_angle = fmaxf(TILT_ANGLE_MIN, fminf(tilt_angle, TILT_ANGLE_MAX));

    // Convert angles to servo positions
    uint16_t pan_pos = gimbal_angle_to_position(pan_angle, 0);   // 0 = pan axis
    uint16_t tilt_pos = gimbal_angle_to_position(tilt_angle, 1); // 1 = tilt axis

    ESP_LOGI(TAG, "🎯 Angle to position: pan=%.1f° -> %d, tilt=%.1f° -> %d", 
             pan_angle, pan_pos, tilt_angle, tilt_pos);

    return gimbal_controller_set_position(pan_pos, tilt_pos);
}

esp_err_t gimbal_controller_set_speed(float pan_speed, float tilt_speed)
{
    if (!gimbal_initialized) {
        ESP_LOGE(TAG, "Gimbal controller not initialized");
        return ESP_ERR_INVALID_STATE;
    }

    // Validate speeds
    if (gimbal_validate_speed(pan_speed) != ESP_OK || gimbal_validate_speed(tilt_speed) != ESP_OK) {
        ESP_LOGE(TAG, "Invalid speed values: pan=%.2f, tilt=%.2f", pan_speed, tilt_speed);
        return ESP_ERR_INVALID_ARG;
    }

    gimbal_control.pan_speed = pan_speed;
    gimbal_control.tilt_speed = tilt_speed;

    ESP_LOGI(TAG, "Speed set: pan=%.2f, tilt=%.2f", pan_speed, tilt_speed);
    return ESP_OK;
}

esp_err_t gimbal_controller_set_acceleration(float pan_acc, float tilt_acc)
{
    if (!gimbal_initialized) {
        ESP_LOGE(TAG, "Gimbal controller not initialized");
        return ESP_ERR_INVALID_STATE;
    }

    // Validate accelerations
    if (gimbal_validate_acceleration(pan_acc) != ESP_OK || 
        gimbal_validate_acceleration(tilt_acc) != ESP_OK) {
        ESP_LOGE(TAG, "Invalid acceleration values: pan=%.2f, tilt=%.2f", pan_acc, tilt_acc);
        return ESP_ERR_INVALID_ARG;
    }

    gimbal_control.pan_acceleration = pan_acc;
    gimbal_control.tilt_acceleration = tilt_acc;

    ESP_LOGI(TAG, "Acceleration set: pan=%.2f, tilt=%.2f", pan_acc, tilt_acc);
    return ESP_OK;
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

esp_err_t gimbal_controller_get_position(uint16_t *pan, uint16_t *tilt)
{
    if (!gimbal_initialized) {
        ESP_LOGE(TAG, "Gimbal controller not initialized");
        return ESP_ERR_INVALID_STATE;
    }

    if (pan == NULL || tilt == NULL) {
        ESP_LOGE(TAG, "Invalid output parameters");
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
        ESP_LOGE(TAG, "Invalid status parameter");
        return ESP_ERR_INVALID_ARG;
    }

    memcpy(status, &gimbal_control, sizeof(gimbal_control_t));
    return ESP_OK;
}

// =============================================================================
// MOVEMENT FUNCTIONS
// =============================================================================

esp_err_t gimbal_controller_move_left(float speed)
{
    gimbal_log_movement("LEFT", GIMBAL_LEFT_POSITION, gimbal_control.tilt_position);
    return gimbal_controller_set_position(GIMBAL_LEFT_POSITION, gimbal_control.tilt_position);
}

esp_err_t gimbal_controller_move_right(float speed)
{
    gimbal_log_movement("RIGHT", GIMBAL_RIGHT_POSITION, gimbal_control.tilt_position);
    return gimbal_controller_set_position(GIMBAL_RIGHT_POSITION, gimbal_control.tilt_position);
}

esp_err_t gimbal_controller_move_up(float speed)
{
    gimbal_log_movement("UP", gimbal_control.pan_position, GIMBAL_UP_POSITION);
    return gimbal_controller_set_position(gimbal_control.pan_position, GIMBAL_UP_POSITION);
}

esp_err_t gimbal_controller_move_down(float speed)
{
    gimbal_log_movement("DOWN", gimbal_control.pan_position, GIMBAL_DOWN_POSITION);
    return gimbal_controller_set_position(gimbal_control.pan_position, GIMBAL_DOWN_POSITION);
}

esp_err_t gimbal_controller_stop(void)
{
    if (!gimbal_initialized) {
        ESP_LOGE(TAG, "Gimbal controller not initialized");
        return ESP_ERR_INVALID_STATE;
    }

    gimbal_control.pan_moving = false;
    gimbal_control.tilt_moving = false;

    ESP_LOGI(TAG, "Gimbal movement stopped");
    return ESP_OK;
}

// =============================================================================
// ADVANCED CONTROL FUNCTIONS
// =============================================================================

esp_err_t gimbal_controller_smooth_move(uint16_t pan_target, uint16_t tilt_target, 
                                       float speed, float acceleration)
{
    if (!gimbal_initialized) {
        ESP_LOGE(TAG, "Gimbal controller not initialized");
        return ESP_ERR_INVALID_STATE;
    }

    gimbal_command_t cmd = {
        .move_type = GIMBAL_MOVE_ABSOLUTE,
        .pan_value = (float)pan_target,
        .tilt_value = (float)tilt_target,
        .speed = speed,
        .acceleration = acceleration
    };

    return gimbal_controller_process_command(&cmd);
}

esp_err_t gimbal_controller_process_command(gimbal_command_t *cmd)
{
    if (!gimbal_initialized) {
        ESP_LOGE(TAG, "Gimbal controller not initialized");
        return ESP_ERR_INVALID_STATE;
    }

    if (cmd == NULL) {
        ESP_LOGE(TAG, "Invalid command parameter");
        return ESP_ERR_INVALID_ARG;
    }

    // Queue command for processing by gimbal task
    if (xQueueSend(gimbal_command_queue, cmd, pdMS_TO_TICKS(100)) != pdTRUE) {
        ESP_LOGW(TAG, "Failed to queue command - queue full");
        return ESP_ERR_TIMEOUT;
    }

    ESP_LOGD(TAG, "Command queued: type=%d, pan=%.2f, tilt=%.2f", 
             cmd->move_type, cmd->pan_value, cmd->tilt_value);
    return ESP_OK;
}

esp_err_t gimbal_controller_stabilize(const imu_data_t *imu_data)
{
    if (!gimbal_initialized) {
        return ESP_ERR_INVALID_STATE;
    }

    if (imu_data == NULL) {
        return ESP_ERR_INVALID_ARG;
    }

    // Only stabilize in steady mode
    if (gimbal_control.mode != GIMBAL_MODE_STEADY) {
        return ESP_OK;
    }

    return gimbal_apply_stabilization_compensation(imu_data);
}

esp_err_t gimbal_controller_enable_steady_mode(bool enable)
{
    if (!gimbal_initialized) {
        ESP_LOGE(TAG, "Gimbal controller not initialized");
        return ESP_ERR_INVALID_STATE;
    }

    if (enable) {
        gimbal_control.mode = GIMBAL_MODE_STEADY;
        gimbal_control.pan_moving = false;
        gimbal_control.tilt_moving = false;
        ESP_LOGI(TAG, "Steady mode enabled - position holding active");
    } else {
        if (gimbal_control.mode == GIMBAL_MODE_STEADY) {
            gimbal_control.mode = GIMBAL_MODE_MANUAL;
            ESP_LOGI(TAG, "Steady mode disabled - manual control enabled");
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

    if (gimbal_control.mode != GIMBAL_MODE_STEADY) {
        ESP_LOGW(TAG, "Steady goal ignored - not in steady mode");
        return ESP_OK;
    }

    uint16_t pan_pulse = (uint16_t)pan_goal;
    uint16_t tilt_pulse = (uint16_t)tilt_goal;

    ESP_LOGI(TAG, "Moving to steady goal: pan=%d, tilt=%d", pan_pulse, tilt_pulse);
    return gimbal_controller_set_position(pan_pulse, tilt_pulse);
}

// =============================================================================
// SERVO FEEDBACK AND CONTROL
// =============================================================================

esp_err_t gimbal_controller_get_feedback(gimbal_feedback_t *pan_fb, gimbal_feedback_t *tilt_fb)
{
    if (!gimbal_initialized) {
        ESP_LOGE(TAG, "Gimbal controller not initialized");
        return ESP_ERR_INVALID_STATE;
    }

    if (pan_fb == NULL || tilt_fb == NULL) {
        ESP_LOGE(TAG, "Invalid feedback parameters");
        return ESP_ERR_INVALID_ARG;
    }

    // Read pan servo feedback
    uint16_t pan_pos, pan_speed, pan_load;
    uint8_t pan_voltage, pan_temperature, pan_moving;
    
    esp_err_t ret = scservo_read_pos(GIMBAL_PAN_ID, &pan_pos);
    if (ret == ESP_OK) {
        pan_fb->status = true;
        pan_fb->pos = pan_pos;
        
        // Read additional servo parameters
        scservo_read_speed(GIMBAL_PAN_ID, &pan_speed);
        scservo_read_load(GIMBAL_PAN_ID, &pan_load);
        scservo_read_voltage(GIMBAL_PAN_ID, &pan_voltage);
        scservo_read_temperature(GIMBAL_PAN_ID, &pan_temperature);
        scservo_read_moving(GIMBAL_PAN_ID, &pan_moving);
        
        pan_fb->speed = pan_speed;
        pan_fb->load = pan_load;
        pan_fb->voltage = pan_voltage / 10.0f;
        pan_fb->current = 0.5f; // Not available in basic protocol
        pan_fb->temper = pan_temperature;
        pan_fb->mode = pan_moving;
    } else {
        // Use fallback data
        pan_fb->status = false;
        pan_fb->pos = gimbal_control.pan_position;
        pan_fb->speed = 0;
        pan_fb->load = 0;
        pan_fb->voltage = 12.0f;
        pan_fb->current = 0.5f;
        pan_fb->temper = 25.0f;
        pan_fb->mode = 0;
        ESP_LOGW(TAG, "Using simulated pan servo feedback");
    }

    // Read tilt servo feedback
    uint16_t tilt_pos, tilt_speed, tilt_load;
    uint8_t tilt_voltage, tilt_temperature, tilt_moving;
    
    ret = scservo_read_pos(GIMBAL_TILT_ID, &tilt_pos);
    if (ret == ESP_OK) {
        tilt_fb->status = true;
        tilt_fb->pos = tilt_pos;
        
        // Read additional servo parameters
        scservo_read_speed(GIMBAL_TILT_ID, &tilt_speed);
        scservo_read_load(GIMBAL_TILT_ID, &tilt_load);
        scservo_read_voltage(GIMBAL_TILT_ID, &tilt_voltage);
        scservo_read_temperature(GIMBAL_TILT_ID, &tilt_temperature);
        scservo_read_moving(GIMBAL_TILT_ID, &tilt_moving);
        
        tilt_fb->speed = tilt_speed;
        tilt_fb->load = tilt_load;
        tilt_fb->voltage = tilt_voltage / 10.0f;
        tilt_fb->current = 0.5f; // Not available in basic protocol
        tilt_fb->temper = tilt_temperature;
        tilt_fb->mode = tilt_moving;
    } else {
        // Use fallback data
        tilt_fb->status = false;
        tilt_fb->pos = gimbal_control.tilt_position;
        tilt_fb->speed = 0;
        tilt_fb->load = 0;
        tilt_fb->voltage = 12.0f;
        tilt_fb->current = 0.5f;
        tilt_fb->temper = 25.0f;
        tilt_fb->mode = 0;
        ESP_LOGW(TAG, "Using simulated tilt servo feedback");
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

    esp_err_t ret = scservo_enable_torque(servo_id, enable ? 1 : 0);
    if (ret == ESP_OK) {
        ESP_LOGI(TAG, "Servo %d torque %s", servo_id, enable ? "enabled" : "disabled");
        
        if (!enable) {
            vTaskDelay(pdMS_TO_TICKS(50)); // Servo stop delay
        }
    } else {
        ESP_LOGE(TAG, "Failed to %s torque for servo %d: %s", 
                 enable ? "enable" : "disable", servo_id, esp_err_to_name(ret));
    }

    return ret;
}

// =============================================================================
// UTILITY FUNCTIONS
// =============================================================================

uint16_t gimbal_controller_degrees_to_pulse(float degrees, uint8_t axis)
{
    return gimbal_angle_to_position(degrees, axis);
}

float gimbal_controller_pulse_to_degrees(uint16_t pulse, uint8_t axis)
{
    return gimbal_position_to_angle(pulse, axis);
}

// =============================================================================
// PRIVATE FUNCTIONS
// =============================================================================

static void gimbal_task(void *pvParameters)
{
    gimbal_command_t cmd;
    ESP_LOGI(TAG, "Gimbal task started");

    while (1) {
        // Process commands from queue
        if (xQueueReceive(gimbal_command_queue, &cmd, pdMS_TO_TICKS(100)) == pdTRUE) {
            uint16_t new_pan, new_tilt;
            
            switch (cmd.move_type) {
                case GIMBAL_MOVE_ABSOLUTE:
                    ESP_LOGD(TAG, "Absolute move: pan=%.2f, tilt=%.2f", 
                             cmd.pan_value, cmd.tilt_value);
                    
                    if (cmd.pan_value >= GIMBAL_PAN_MIN && cmd.pan_value <= GIMBAL_PAN_MAX &&
                        cmd.tilt_value >= GIMBAL_TILT_MIN && cmd.tilt_value <= GIMBAL_TILT_MAX) {
                        gimbal_controller_set_position((uint16_t)cmd.pan_value, (uint16_t)cmd.tilt_value);
                    }
                    break;

                case GIMBAL_MOVE_RELATIVE:
                    ESP_LOGD(TAG, "Relative move: pan=%.2f, tilt=%.2f", 
                             cmd.pan_value, cmd.tilt_value);
                    
                    new_pan = gimbal_control.pan_position + (int16_t)cmd.pan_value;
                    new_tilt = gimbal_control.tilt_position + (int16_t)cmd.tilt_value;
                    
                    // Clamp to limits
                    new_pan = (new_pan < GIMBAL_PAN_MIN) ? GIMBAL_PAN_MIN : 
                              (new_pan > GIMBAL_PAN_MAX) ? GIMBAL_PAN_MAX : new_pan;
                    new_tilt = (new_tilt < GIMBAL_TILT_MIN) ? GIMBAL_TILT_MIN : 
                               (new_tilt > GIMBAL_TILT_MAX) ? GIMBAL_TILT_MAX : new_tilt;
                    
                    gimbal_controller_set_position(new_pan, new_tilt);
                    break;

                case GIMBAL_MOVE_VELOCITY:
                    ESP_LOGD(TAG, "Velocity move: pan=%.2f, tilt=%.2f", 
                             cmd.pan_value, cmd.tilt_value);
                    
                    gimbal_control.pan_speed = cmd.pan_value;
                    gimbal_control.tilt_speed = cmd.tilt_value;
                    break;

                default:
                    ESP_LOGW(TAG, "Unknown move type: %d", cmd.move_type);
                    break;
            }
        }

        // Update timestamp
        gimbal_control.last_update_time = esp_timer_get_time() / 1000;

        vTaskDelay(pdMS_TO_TICKS(1000 / GIMBAL_UPDATE_RATE_HZ));
    }
}

static esp_err_t gimbal_send_sync_write_command(uint8_t *ids, uint8_t id_count, 
                                               int16_t *positions, uint16_t *speeds, uint8_t *accs)
{
    if (!ids || !positions || !speeds || !accs || id_count == 0) {
        return ESP_ERR_INVALID_ARG;
    }

    // Prepare servo data buffer
    uint8_t data_buffer[SCSERVO_DATA_LENGTH_PER_SERVO * id_count];
    
    for (uint8_t i = 0; i < id_count; i++) {
        int16_t pos = positions[i];
        
        // Handle negative positions
        if (pos < 0) {
            pos = -pos;
            pos |= (1 << 15);
        }
        
        uint16_t speed = speeds[i];
        uint8_t acc = accs[i];
        
        // Pack data: ACC(1) + Position(2) + Time(2) + Speed(2)
        uint8_t *servo_data = &data_buffer[i * SCSERVO_DATA_LENGTH_PER_SERVO];
        servo_data[0] = acc;
        servo_data[1] = pos & 0xFF;
        servo_data[2] = (pos >> 8) & 0xFF;
        servo_data[3] = 0; // Time low
        servo_data[4] = 0; // Time high
        servo_data[5] = speed & 0xFF;
        servo_data[6] = (speed >> 8) & 0xFF;
    }
    
    // Build packet
    uint8_t packet_length = ((SCSERVO_DATA_LENGTH_PER_SERVO + 1) * id_count + 4);
    uint8_t packet[256];
    uint8_t packet_index = 0;
    
    // Header
    packet[packet_index++] = SCSERVO_HEADER1;
    packet[packet_index++] = SCSERVO_HEADER2;
    packet[packet_index++] = SCSERVO_BROADCAST_ID;
    packet[packet_index++] = packet_length;
    packet[packet_index++] = SCSERVO_INST_SYNC_WRITE;
    packet[packet_index++] = SMS_STS_ACC_REGISTER;
    packet[packet_index++] = SCSERVO_DATA_LENGTH_PER_SERVO;
    
    // Calculate checksum
    uint8_t checksum = SCSERVO_BROADCAST_ID + packet_length + SCSERVO_INST_SYNC_WRITE + 
                       SMS_STS_ACC_REGISTER + SCSERVO_DATA_LENGTH_PER_SERVO;
    
    // Add servo data
    for (uint8_t i = 0; i < id_count; i++) {
        packet[packet_index++] = ids[i];
        checksum += ids[i];
        
        for (uint8_t j = 0; j < SCSERVO_DATA_LENGTH_PER_SERVO; j++) {
            uint8_t data_byte = data_buffer[i * SCSERVO_DATA_LENGTH_PER_SERVO + j];
            packet[packet_index++] = data_byte;
            checksum += data_byte;
        }
    }
    
    // Add checksum
    packet[packet_index++] = ~checksum;
    
    ESP_LOGI(TAG, "📡 Sending sync write: %d servos, %d bytes", id_count, packet_index);
    
    // Send packet
    int bytes_written = uart_write_bytes(SCSERVO_UART_NUM, packet, packet_index);
    if (bytes_written != packet_index) {
        ESP_LOGE(TAG, "UART write failed: expected %d, written %d", packet_index, bytes_written);
        return ESP_FAIL;
    }
    
    ESP_LOGI(TAG, "✅ Sync write command sent successfully (%d bytes)", bytes_written);
    
    // Processing delay
    vTaskDelay(pdMS_TO_TICKS(10));
    
    return ESP_OK;
}

static esp_err_t gimbal_validate_position(uint16_t pan, uint16_t tilt)
{
    if (pan < GIMBAL_PAN_MIN || pan > GIMBAL_PAN_MAX) {
        ESP_LOGE(TAG, "Pan position out of range: %d (valid: %d-%d)", 
                 pan, GIMBAL_PAN_MIN, GIMBAL_PAN_MAX);
        return ESP_ERR_INVALID_ARG;
    }
    
    if (tilt < GIMBAL_TILT_MIN || tilt > GIMBAL_TILT_MAX) {
        ESP_LOGE(TAG, "Tilt position out of range: %d (valid: %d-%d)", 
                 tilt, GIMBAL_TILT_MIN, GIMBAL_TILT_MAX);
        return ESP_ERR_INVALID_ARG;
    }
    
    return ESP_OK;
}

static esp_err_t gimbal_validate_speed(float speed)
{
    return (speed >= GIMBAL_MIN_SPEED && speed <= GIMBAL_MAX_SPEED) ? ESP_OK : ESP_ERR_INVALID_ARG;
}

static esp_err_t gimbal_validate_acceleration(float acceleration)
{
    return (acceleration >= GIMBAL_MIN_ACCELERATION && acceleration <= GIMBAL_MAX_ACCELERATION) ? 
           ESP_OK : ESP_ERR_INVALID_ARG;
}

static uint16_t gimbal_angle_to_position(float angle, uint8_t axis)
{
    if (axis == 0) { // Pan axis
        float normalized = angle + 180.0f; // -180..180 -> 0..360
        normalized = fmaxf(0.0f, fminf(normalized, PAN_ANGLE_RANGE));
        
        // Apply direction correction for pan
        uint16_t pos = (uint16_t)((normalized * SERVO_POSITION_RANGE) / PAN_ANGLE_RANGE);
        return GIMBAL_CENTER_POSITION - pos + GIMBAL_CENTER_POSITION; // Invert direction
    } else { // Tilt axis
        float normalized = angle + 30.0f; // -30..90 -> 0..120
        normalized = fmaxf(0.0f, fminf(normalized, TILT_ANGLE_RANGE));
        return GIMBAL_CENTER_POSITION - (uint16_t)((normalized * SERVO_POSITION_RANGE) / TILT_ANGLE_RANGE) + GIMBAL_CENTER_POSITION;
    }
}

static float gimbal_position_to_angle(uint16_t position, uint8_t axis)
{
    if (axis == 0) { // Pan axis
        float normalized = (float)position * PAN_ANGLE_RANGE / SERVO_POSITION_RANGE;
        return normalized - 180.0f;
    } else { // Tilt axis
        float normalized = (float)position * TILT_ANGLE_RANGE / SERVO_POSITION_RANGE;
        return normalized - 30.0f;
    }
}

static void gimbal_log_movement(const char* direction, uint16_t pan, uint16_t tilt)
{
    ESP_LOGI(TAG, "🎯 Moving gimbal %s: pan=%d, tilt=%d", direction, pan, tilt);
}

static esp_err_t gimbal_apply_stabilization_compensation(const imu_data_t *imu_data)
{
    // Calculate compensation based on gyroscope data
    float pan_compensation = -imu_data->gyro_z * GIMBAL_GYRO_COMPENSATION_GAIN;
    float tilt_compensation = -imu_data->gyro_x * GIMBAL_GYRO_COMPENSATION_GAIN;

    // Apply compensation if significant
    if (fabsf(pan_compensation) > GIMBAL_STABILIZATION_THRESHOLD || 
        fabsf(tilt_compensation) > GIMBAL_STABILIZATION_THRESHOLD) {
        
        float compensated_pan = gimbal_control.pan_position + pan_compensation;
        float compensated_tilt = gimbal_control.tilt_position + tilt_compensation;

        // Clamp to limits
        compensated_pan = fmaxf(GIMBAL_PAN_MIN, fminf(compensated_pan, GIMBAL_PAN_MAX));
        compensated_tilt = fmaxf(GIMBAL_TILT_MIN, fminf(compensated_tilt, GIMBAL_TILT_MAX));

        ESP_LOGD(TAG, "Stabilization: pan_comp=%.2f, tilt_comp=%.2f", 
                 pan_compensation, tilt_compensation);

        return gimbal_controller_set_position((uint16_t)compensated_pan, (uint16_t)compensated_tilt);
    }

    return ESP_OK;
}