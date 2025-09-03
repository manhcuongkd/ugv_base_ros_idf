#include "../inc/json_parser.h"
#include "../inc/motion_module.h"
#include "../inc/servo_controller.h"
#include "../inc/gimbal_controller.h"
#include "../inc/files_controller.h"
#include "../inc/esp_now_controller.h"
#include "../inc/wifi_controller.h"
#include "../inc/imu_controller.h"
#include "../inc/battery_controller.h"
#include <driver/uart.h>
#include <driver/gpio.h>
#include <freertos/FreeRTOS.h>
#include <freertos/task.h>
#include <freertos/queue.h>
#include <esp_log.h>
#include <string.h>
#include <cJSON.h>
#include <nvs_flash.h>
#include <esp_spiffs.h>

static const char *TAG = "UART_Controller";

// UART configuration
#define UART_NUM UART_NUM_0
#define UART_BAUD_RATE 115200
#define UART_BUFFER_SIZE 1024

// Global variables
static QueueHandle_t uart_rx_queue;
static TaskHandle_t uart_rx_task_handle;
static bool uart_initialized = false;
static uart_control_t uart_control = {
    .echo_mode = false,
    .heartbeat_enabled = true,
    .heartbeat_delay_ms = 1000,
    .last_cmd_time = 0,
    .feedback_interval_ms = 100,
    .base_feedback_flow = true
};

// Private function prototypes
static void uart_rx_task(void *pvParameters);
static esp_err_t uart_parse_command(const char *data, size_t len);

esp_err_t uart_controller_init(void)
{
    if (uart_initialized) {
        ESP_LOGW(TAG, "UART controller already initialized");
        return ESP_OK;
    }

    ESP_LOGI(TAG, "Initializing UART controller...");

    // Configure UART parameters
    uart_config_t uart_config = {
        .baud_rate = UART_BAUD_RATE,
        .data_bits = UART_DATA_8_BITS,
        .parity = UART_PARITY_DISABLE,
        .stop_bits = UART_STOP_BITS_1,
        .flow_ctrl = UART_HW_FLOWCTRL_DISABLE,
        .rx_flow_ctrl_thresh = 0,
        .source_clk = UART_SCLK_APB,
    };

    // Install UART driver
    esp_err_t ret = uart_driver_install(UART_NUM, UART_BUFFER_SIZE * 2, UART_BUFFER_SIZE * 2, 20, &uart_rx_queue, 0);
    if (ret != ESP_OK) {
        ESP_LOGE(TAG, "Failed to install UART driver: %s", esp_err_to_name(ret));
        return ret;
    }

    // Set UART parameters
    ret = uart_param_config(UART_NUM, &uart_config);
    if (ret != ESP_OK) {
        ESP_LOGE(TAG, "Failed to configure UART parameters: %s", esp_err_to_name(ret));
        return ret;
    }

    // Set UART pins
    ret = uart_set_pin(UART_NUM, GPIO_NUM_1, GPIO_NUM_3, UART_PIN_NO_CHANGE, UART_PIN_NO_CHANGE);
    if (ret != ESP_OK) {
        ESP_LOGE(TAG, "Failed to set UART pins: %s", esp_err_to_name(ret));
        return ret;
    }

    // Create UART RX task
    xTaskCreate(uart_rx_task, "uart_rx_task", 4096, NULL, 5, &uart_rx_task_handle);
    if (uart_rx_task_handle == NULL) {
        ESP_LOGE(TAG, "Failed to create UART RX task");
        return ESP_ERR_NO_MEM;
    }

    uart_initialized = true;
    ESP_LOGI(TAG, "UART controller initialized successfully");
    return ESP_OK;
}

esp_err_t uart_controller_deinit(void)
{
    if (!uart_initialized) {
        return ESP_OK;
    }

    ESP_LOGI(TAG, "Deinitializing UART controller...");

    // Delete UART RX task
    if (uart_rx_task_handle) {
        vTaskDelete(uart_rx_task_handle);
        uart_rx_task_handle = NULL;
    }

    // Uninstall UART driver
    esp_err_t ret = uart_driver_delete(UART_NUM);
    if (ret != ESP_OK) {
        ESP_LOGE(TAG, "Failed to uninstall UART driver: %s", esp_err_to_name(ret));
        return ret;
    }

    uart_initialized = false;
    ESP_LOGI(TAG, "UART controller deinitialized");
    return ESP_OK;
}

esp_err_t uart_controller_send_json(const char *json_str)
{
    if (!uart_initialized) {
        ESP_LOGE(TAG, "UART controller not initialized");
        return ESP_ERR_INVALID_STATE;
    }

    if (json_str == NULL) {
        return ESP_ERR_INVALID_ARG;
    }

    esp_err_t ret = uart_write_bytes(UART_NUM, json_str, strlen(json_str));
    if (ret != ESP_OK) {
        ESP_LOGE(TAG, "Failed to send JSON via UART: %s", esp_err_to_name(ret));
        return ret;
    }

    ESP_LOGD(TAG, "Sent JSON via UART: %s", json_str);
    return ESP_OK;
}

esp_err_t uart_controller_send_command(uint8_t cmd_type, const void *data)
{
    if (!uart_initialized) {
        ESP_LOGE(TAG, "UART controller not initialized");
        return ESP_ERR_INVALID_STATE;
    }

    // Create command structure
    cJSON *cmd = cJSON_CreateObject();
    cJSON_AddNumberToObject(cmd, "cmd", cmd_type);
    
    if (data != NULL) {
        // Add data based on command type
        switch (cmd_type) {
            case CMD_SPEED_CTRL:
                cJSON_AddNumberToObject(cmd, "speed", *(uint8_t*)data);
                break;
            case CMD_PWM_INPUT:
                cJSON_AddNumberToObject(cmd, "pwm", *(uint16_t*)data);
                break;
            case CMD_SET_MOTOR_PID:
                // Assuming data is pid_params_t
                cJSON_AddNumberToObject(cmd, "kp", ((float*)data)[0]);
                cJSON_AddNumberToObject(cmd, "ki", ((float*)data)[1]);
                cJSON_AddNumberToObject(cmd, "kd", ((float*)data)[2]);
                break;
            default:
                cJSON_AddStringToObject(cmd, "data", (char*)data);
                break;
        }
    }

    char *json_str = cJSON_Print(cmd);
    if (json_str == NULL) {
        cJSON_Delete(cmd);
        return ESP_ERR_INVALID_ARG;
    }

    esp_err_t ret = uart_controller_send_json(json_str);
    free(json_str);
    cJSON_Delete(cmd);
    
    return ret;
}

esp_err_t uart_controller_set_echo_mode(bool enable)
{
    uart_control.echo_mode = enable;
    ESP_LOGI(TAG, "UART echo mode %s", enable ? "enabled" : "disabled");
    return ESP_OK;
}

esp_err_t uart_controller_set_heartbeat_delay(uint32_t delay_ms)
{
    uart_control.heartbeat_delay_ms = delay_ms;
    ESP_LOGI(TAG, "UART heartbeat delay set to %u ms", delay_ms);
    return ESP_OK;
}

esp_err_t uart_controller_set_feedback_interval(uint32_t interval_ms)
{
    uart_control.feedback_interval_ms = interval_ms;
    ESP_LOGI(TAG, "UART feedback interval set to %u ms", interval_ms);
    return ESP_OK;
}

esp_err_t uart_controller_set_base_feedback_flow(bool enable)
{
    uart_control.base_feedback_flow = enable;
    ESP_LOGI(TAG, "UART base feedback flow %s", enable ? "enabled" : "disabled");
    return ESP_OK;
}

esp_err_t uart_controller_get_status(uart_control_t *status)
{
    if (!uart_initialized || status == NULL) {
        return ESP_ERR_INVALID_ARG;
    }

    memcpy(status, &uart_control, sizeof(uart_control_t));
    return ESP_OK;
}

// Private functions
static void uart_rx_task(void *pvParameters)
{
    uart_event_t event;
    uint8_t *data = (uint8_t *)malloc(UART_BUFFER_SIZE);
    
    if (data == NULL) {
        ESP_LOGE(TAG, "Failed to allocate UART buffer");
        vTaskDelete(NULL);
        return;
    }

    ESP_LOGI(TAG, "UART RX task started");

    while (1) {
        if (xQueueReceive(uart_rx_queue, (void *)&event, portMAX_DELAY)) {
            switch (event.type) {
                case UART_DATA:
                    if (event.size > 0) {
                        int len = uart_read_bytes(UART_NUM, data, event.size, pdMS_TO_TICKS(100));
                        if (len > 0) {
                            ESP_LOGD(TAG, "Received %d bytes via UART", len);
                            
                            // Parse received command
                            uart_parse_command((const char *)data, len);
                            
                            // Echo back if enabled
                            if (uart_control.echo_mode) {
                                uart_write_bytes(UART_NUM, data, len);
                            }
                        }
                    }
                    break;
                    
                case UART_FIFO_OVF:
                    ESP_LOGW(TAG, "UART FIFO overflow");
                    uart_flush_input(UART_NUM);
                    xQueueReset(uart_rx_queue);
                    break;
                    
                case UART_BUFFER_FULL:
                    ESP_LOGW(TAG, "UART ring buffer full");
                    uart_flush_input(UART_NUM);
                    xQueueReset(uart_rx_queue);
                    break;
                    
                case UART_BREAK:
                    ESP_LOGW(TAG, "UART RX break");
                    break;
                    
                case UART_PARITY_ERR:
                    ESP_LOGW(TAG, "UART parity error");
                    break;
                    
                case UART_FRAME_ERR:
                    ESP_LOGW(TAG, "UART frame error");
                    break;
                    
                default:
                    ESP_LOGW(TAG, "UART event type: %d", event.type);
                    break;
            }
        }
    }

    free(data);
    vTaskDelete(NULL);
}

static esp_err_t uart_parse_command(const char *data, size_t len)
{
    if (data == NULL || len == 0) {
        return ESP_ERR_INVALID_ARG;
    }

    // Ensure null termination
    char *str_data = (char *)malloc(len + 1);
    if (str_data == NULL) {
        ESP_LOGE(TAG, "Failed to allocate memory for command parsing");
        return ESP_ERR_NO_MEM;
    }

    memcpy(str_data, data, len);
    str_data[len] = '\0';

    ESP_LOGI(TAG, "Parsing UART command: %s", str_data);

    // Try to parse as JSON
    cJSON *json = cJSON_Parse(str_data);
    if (json != NULL) {
        ESP_LOGI(TAG, "Valid JSON command received");
        
        // Extract command type
        cJSON *cmd = cJSON_GetObjectItem(json, "cmd");
        if (cJSON_IsNumber(cmd)) {
            uint8_t cmd_type = (uint8_t)cmd->valuedouble;
            ESP_LOGI(TAG, "Command type: %d", cmd_type);
            
            // Handle command based on type
            switch (cmd_type) {
                case CMD_EMERGENCY_STOP:
                    ESP_LOGI(TAG, "Emergency stop command received");
                    break;
                case CMD_SPEED_CTRL:
                    ESP_LOGI(TAG, "Speed control command received");
                    break;
                case CMD_ROS_CTRL:
                    ESP_LOGI(TAG, "ROS control command received");
                    break;
                default:
                    ESP_LOGW(TAG, "Unknown command type: %d", cmd_type);
                    break;
            }
        }
        
        cJSON_Delete(json);
    } else {
        ESP_LOGW(TAG, "Invalid JSON, treating as plain text command");
        
        // Handle plain text commands
        if (strncmp(str_data, "MOVE", 4) == 0) {
            ESP_LOGI(TAG, "Move command received");
        } else if (strncmp(str_data, "STATUS", 6) == 0) {
            ESP_LOGI(TAG, "Status request received");
        } else if (strncmp(str_data, "STOP", 4) == 0) {
            ESP_LOGI(TAG, "Stop command received");
        } else {
            ESP_LOGW(TAG, "Unknown command: %s", str_data);
        }
    }

    free(str_data);
    return ESP_OK;
}

// Real implementations for functions declared in header
void uart_controller_task(void *pvParameters)
{
    ESP_LOGI(TAG, "UART controller task started");
    
    while (1) {
        // Process any pending UART commands
        if (uart_control.echo_mode) {
            // Send periodic status updates
            uart_controller_send_feedback();
        }
        
        // Check for heartbeat interval
        if (uart_control.heartbeat_delay_ms > 0) {
            static uint32_t last_heartbeat = 0;
            uint32_t current_time = esp_timer_get_time() / 1000;
            
            if (current_time - last_heartbeat >= uart_control.heartbeat_delay_ms) {
                uart_controller_send_feedback();
                last_heartbeat = current_time;
            }
        }
        
        vTaskDelay(pdMS_TO_TICKS(100)); // 10Hz update rate
    }
}

void uart_controller_handle_command(json_command_t *cmd)
{
    if (cmd == NULL) {
        ESP_LOGE(TAG, "Invalid command pointer");
        return;
    }
    
    ESP_LOGI(TAG, "Handling UART command: type=%d", cmd->type);
    
    // Process command based on type
    switch (cmd->type) {
        case CMD_EMERGENCY_STOP:
            ESP_LOGW(TAG, "Emergency stop command received via UART");
            motion_module_emergency_stop();
            break;
            
        case CMD_EOAT_TYPE:
            ESP_LOGI(TAG, "EOAT type command: mode=%d", cmd->payload.eoat_type.mode);
            {
                // Store EOAT type in NVS
                nvs_handle_t nvs_handle;
                esp_err_t err = nvs_open("system", NVS_READWRITE, &nvs_handle);
                if (err == ESP_OK) {
                    err = nvs_set_u8(nvs_handle, "eoat_type", cmd->payload.eoat_type.mode);
                    if (err == ESP_OK) {
                        err = nvs_commit(nvs_handle);
                    }
                    nvs_close(nvs_handle);
                    if (err == ESP_OK) {
                        ESP_LOGI(TAG, "EOAT type set to: %d", cmd->payload.eoat_type.mode);
                    } else {
                        ESP_LOGE(TAG, "Failed to set EOAT type: %s", esp_err_to_name(err));
                    }
                }
            }
            break;
            
        case CMD_CONFIG_EOAT:
            ESP_LOGI(TAG, "Configure EOAT command: pos=%d, ea=%.2f, eb=%.2f", 
                     cmd->payload.eoat_config.pos, cmd->payload.eoat_config.ea, cmd->payload.eoat_config.eb);
            {
                // Store EOAT configuration in NVS
                nvs_handle_t nvs_handle;
                esp_err_t err = nvs_open("system", NVS_READWRITE, &nvs_handle);
                if (err == ESP_OK) {
                    err = nvs_set_u8(nvs_handle, "eoat_pos", cmd->payload.eoat_config.pos);
                    if (err == ESP_OK) {
                        err = nvs_set_blob(nvs_handle, "eoat_ea", &cmd->payload.eoat_config.ea, sizeof(double));
                    }
                    if (err == ESP_OK) {
                        err = nvs_set_blob(nvs_handle, "eoat_eb", &cmd->payload.eoat_config.eb, sizeof(double));
                    }
                    if (err == ESP_OK) {
                        err = nvs_commit(nvs_handle);
                    }
                    nvs_close(nvs_handle);
                    if (err == ESP_OK) {
                        ESP_LOGI(TAG, "EOAT configuration stored: pos=%d, ea=%.2f, eb=%.2f", 
                                 cmd->payload.eoat_config.pos, cmd->payload.eoat_config.ea, cmd->payload.eoat_config.eb);
                    } else {
                        ESP_LOGE(TAG, "Failed to store EOAT configuration: %s", esp_err_to_name(err));
                    }
                }
            }
            break;
            
        case CMD_OLED_CTRL:
            ESP_LOGI(TAG, "OLED control command: line=%d, text=%s", 
                     cmd->payload.oled_ctrl.line_num, cmd->payload.oled_ctrl.text);
            {
                // Control OLED display
                oled_controller_set_text(cmd->payload.oled_ctrl.line_num, cmd->payload.oled_ctrl.text);
            }
            break;
            
        case CMD_OLED_DEFAULT:
            ESP_LOGI(TAG, "OLED default command");
            {
                // Reset OLED to default display
                oled_controller_reset_to_default();
            }
            break;
            
        case CMD_MODULE_TYPE:
            ESP_LOGI(TAG, "Module type command: cmd=%d", cmd->payload.config.cmd);
            {
                // Store module type in NVS
                nvs_handle_t nvs_handle;
                esp_err_t err = nvs_open("system", NVS_READWRITE, &nvs_handle);
                if (err == ESP_OK) {
                    err = nvs_set_u8(nvs_handle, "module_type", cmd->payload.config.cmd);
                    if (err == ESP_OK) {
                        err = nvs_commit(nvs_handle);
                    }
                    nvs_close(nvs_handle);
                    if (err == ESP_OK) {
                        ESP_LOGI(TAG, "Module type set to: %d", cmd->payload.config.cmd);
                    } else {
                        ESP_LOGE(TAG, "Failed to set module type: %s", esp_err_to_name(err));
                    }
                }
            }
            break;
            
        case CMD_SPEED_CTRL:
            ESP_LOGI(TAG, "Speed control command: L=%.2f, R=%.2f", 
                     cmd->payload.speed_ctrl.left_speed, 
                     cmd->payload.speed_ctrl.right_speed);
            motion_module_set_speed_rate(cmd->payload.speed_ctrl.left_speed, cmd->payload.speed_ctrl.right_speed);
            break;
            
        case CMD_ROS_CTRL:
            ESP_LOGI(TAG, "ROS control command: linear=%.2f, angular=%.2f", 
                     cmd->payload.ros_ctrl.linear_speed, 
                     cmd->payload.ros_ctrl.angular_speed);
            motion_module_set_ros_control(cmd->payload.ros_ctrl.linear_speed, cmd->payload.ros_ctrl.angular_speed);
            break;
            
        // Motion & Control Commands
        case CMD_GET_SPD_RATE:
            ESP_LOGI(TAG, "Get speed rate command");
            {
                motion_control_t status;
                if (motion_module_get_status(&status) == ESP_OK) {
                    ESP_LOGI(TAG, "Current speed rates: L=%.2f, R=%.2f", 
                             status.left_speed, status.right_speed);
                }
            }
            break;
            
        case CMD_SAVE_SPD_RATE:
            ESP_LOGI(TAG, "Save speed rate command");
            {
                nvs_handle_t nvs_handle;
                esp_err_t err = nvs_open("motion", NVS_READWRITE, &nvs_handle);
                if (err == ESP_OK) {
                    // Get current speed rates from motion module
                    motion_control_t status;
                    if (motion_module_get_status(&status) == ESP_OK) {
                        err = nvs_set_u32(nvs_handle, "left_rate", (uint32_t)(status.left_speed * 1000));
                        if (err == ESP_OK) {
                            err = nvs_set_u32(nvs_handle, "right_rate", (uint32_t)(status.right_speed * 1000));
                        }
                        if (err == ESP_OK) {
                            err = nvs_commit(nvs_handle);
                        }
                        nvs_close(nvs_handle);
                        if (err == ESP_OK) {
                            ESP_LOGI(TAG, "Speed rates saved to NVS successfully");
                        } else {
                            ESP_LOGE(TAG, "Failed to save speed rates to NVS: %s", esp_err_to_name(err));
                        }
                    }
                }
            }
            break;
            
        case CMD_SWITCH_OFF:
            ESP_LOGW(TAG, "Switch off command received");
            motion_module_emergency_stop();
            servo_controller_disable_all();
            gimbal_controller_stop();
            break;
            
        case CMD_CONSTANT_CTRL:
            ESP_LOGI(TAG, "Constant control command: mode=%d, axis=%d, cmd=%d, speed=%d",
                     cmd->payload.constant_ctrl.mode, cmd->payload.constant_ctrl.axis,
                     cmd->payload.constant_ctrl.cmd, cmd->payload.constant_ctrl.speed);
            {
                float left_speed = 0.0f, right_speed = 0.0f;
                float speed_factor = (float)cmd->payload.constant_ctrl.speed / 1000.0f;
                
                switch (cmd->payload.constant_ctrl.axis) {
                    case 0: // Forward/Backward
                        if (cmd->payload.constant_ctrl.cmd == 0) { // Forward
                            left_speed = speed_factor;
                            right_speed = speed_factor;
                        } else { // Backward
                            left_speed = -speed_factor;
                            right_speed = -speed_factor;
                        }
                        break;
                    case 1: // Left/Right
                        if (cmd->payload.constant_ctrl.cmd == 0) { // Left
                            left_speed = -speed_factor;
                            right_speed = speed_factor;
                        } else { // Right
                            left_speed = speed_factor;
                            right_speed = -speed_factor;
                        }
                        break;
                    case 2: // Rotate
                        if (cmd->payload.constant_ctrl.cmd == 0) { // CCW
                            left_speed = -speed_factor;
                            right_speed = speed_factor;
                        } else { // CW
                            left_speed = speed_factor;
                            right_speed = -speed_factor;
                        }
                        break;
                }
                
                if (cmd->payload.constant_ctrl.mode == 1) { // Continuous mode
                    motion_module_set_speed_rate(left_speed, right_speed);
                    ESP_LOGI(TAG, "Constant control: L=%.2f, R=%.2f", left_speed, right_speed);
                }
            }
            break;
            
        case CMD_HEART_BEAT_SET:
            ESP_LOGI(TAG, "Set heartbeat delay: %u ms", cmd->payload.heartbeat.delay_ms);
            uart_control.heartbeat_delay_ms = cmd->payload.heartbeat.delay_ms;
            break;
            
        case CMD_FEEDBACK_FLOW_INTERVAL:
            ESP_LOGI(TAG, "Set feedback flow interval: %u ms", cmd->payload.feedback_flow.interval_ms);
            uart_control.feedback_interval_ms = cmd->payload.feedback_flow.interval_ms;
            break;
            
        case CMD_UART_ECHO_MODE:
            ESP_LOGI(TAG, "Set UART echo mode: %s", cmd->payload.uart_echo.echo_mode ? "ON" : "OFF");
            uart_control.echo_mode = cmd->payload.uart_echo.echo_mode;
            break;
            
        // Robotic Arm Control Commands
        case CMD_MOVE_INIT:
            ESP_LOGI(TAG, "Move to initial position command");
            servo_controller_move_init_position();
            break;
            
        case CMD_SINGLE_JOINT_CTRL:
            ESP_LOGI(TAG, "Single joint control: joint=%d, angle=%.2f, speed=%d, acc=%d",
                     cmd->payload.joint_ctrl.joint, cmd->payload.joint_ctrl.angle_rad,
                     cmd->payload.joint_ctrl.speed, cmd->payload.joint_ctrl.acceleration);
            {
                uint16_t position = servo_controller_angle_to_position(cmd->payload.joint_ctrl.angle_rad);
                servo_controller_set_position(cmd->payload.joint_ctrl.joint, position, cmd->payload.joint_ctrl.speed);
                servo_controller_set_speed(cmd->payload.joint_ctrl.joint, cmd->payload.joint_ctrl.speed);
                servo_controller_set_acceleration(cmd->payload.joint_ctrl.joint, cmd->payload.joint_ctrl.acceleration);
            }
            break;
            
        case CMD_JOINTS_RAD_CTRL:
            ESP_LOGI(TAG, "All joints radian control: base=%.2f, shoulder=%.2f, elbow=%.2f, hand=%.2f, speed=%d, acc=%d",
                     cmd->payload.joints_ctrl.base, cmd->payload.joints_ctrl.shoulder,
                     cmd->payload.joints_ctrl.elbow, cmd->payload.joints_ctrl.hand,
                     cmd->payload.joints_ctrl.speed, cmd->payload.joints_ctrl.acceleration);
            {
                arm_joint_angles_t angles = {
                    .base = (float)cmd->payload.joints_ctrl.base,
                    .shoulder = (float)cmd->payload.joints_ctrl.shoulder,
                    .elbow = (float)cmd->payload.joints_ctrl.elbow,
                    .gripper = (float)cmd->payload.joints_ctrl.hand
                };
                servo_controller_set_joint_angles(&angles, cmd->payload.joints_ctrl.speed);
            }
            break;
            
        case CMD_XYZT_GOAL_CTRL:
            ESP_LOGI(TAG, "XYZT goal control: x=%.2f, y=%.2f, z=%.2f, t=%.2f, speed=%d",
                     cmd->payload.xyzt_ctrl.x, cmd->payload.xyzt_ctrl.y,
                     cmd->payload.xyzt_ctrl.z, cmd->payload.xyzt_ctrl.t,
                     cmd->payload.xyzt_ctrl.speed);
            {
                arm_pose_t pose = {
                    .x = (float)cmd->payload.xyzt_ctrl.x,
                    .y = (float)cmd->payload.xyzt_ctrl.y,
                    .z = (float)cmd->payload.xyzt_ctrl.z,
                    .roll = 0.0f,
                    .pitch = 0.0f,
                    .yaw = (float)cmd->payload.xyzt_ctrl.t
                };
                servo_controller_set_pose(&pose, cmd->payload.xyzt_ctrl.speed);
            }
            break;
            
        case CMD_SINGLE_AXIS_CTRL:
            ESP_LOGI(TAG, "Single axis control: axis=%d, pos=%.2f, speed=%.2f",
                     cmd->payload.single_axis.axis, cmd->payload.single_axis.pos, cmd->payload.single_axis.speed);
            {
                // Control single axis of the robotic arm
                switch (cmd->payload.single_axis.axis) {
                    case 1: // X-axis
                        servo_controller_set_x_position((float)cmd->payload.single_axis.pos, (float)cmd->payload.single_axis.speed);
                        break;
                    case 2: // Y-axis
                        servo_controller_set_y_position((float)cmd->payload.single_axis.pos, (float)cmd->payload.single_axis.speed);
                        break;
                    case 3: // Z-axis
                        servo_controller_set_z_position((float)cmd->payload.single_axis.pos, (float)cmd->payload.single_axis.speed);
                        break;
                    case 4: // T-axis (theta)
                        servo_controller_set_theta_position((float)cmd->payload.single_axis.pos, (float)cmd->payload.single_axis.speed);
                        break;
                    default:
                        ESP_LOGW(TAG, "Invalid axis: %d", cmd->payload.single_axis.axis);
                        break;
                }
            }
            break;
            
        case CMD_SERVO_RAD_FEEDBACK:
            ESP_LOGI(TAG, "Servo radian feedback command");
            {
                // Get current servo positions and convert to radians
                arm_pose_t current_pose;
                if (servo_controller_get_current_pose(&current_pose) == ESP_OK) {
                    ESP_LOGI(TAG, "Current pose: x=%.2f, y=%.2f, z=%.2f, t=%.2f",
                             current_pose.x, current_pose.y, current_pose.z, current_pose.yaw);
                }
            }
            break;
            
        case CMD_EOAT_GRAB_TORQUE:
            ESP_LOGI(TAG, "EOAT grab torque command: torque=%d", cmd->payload.eoat_torque.torque);
            {
                // Set end effector grab torque
                servo_controller_set_gripper_torque(cmd->payload.eoat_torque.torque);
            }
            break;
            
        case CMD_SET_NEW_X:
            ESP_LOGI(TAG, "Set new X axis command: angle=%.2f", cmd->payload.new_x_axis.angle);
            {
                // Set new X-axis orientation
                servo_controller_set_x_axis_orientation((float)cmd->payload.new_x_axis.angle);
            }
            break;
            
        case CMD_DELAY_MILLIS:
            ESP_LOGI(TAG, "Delay milliseconds command: delay=%d ms", cmd->payload.delay_millis.delay);
            {
                // Execute delay
                vTaskDelay(pdMS_TO_TICKS(cmd->payload.delay_millis.delay));
                ESP_LOGI(TAG, "Delay completed: %d ms", cmd->payload.delay_millis.delay);
            }
            break;
            
        case CMD_SINGLE_JOINT_ANGLE:
            ESP_LOGI(TAG, "Single joint angle control: joint=%d, angle=%.2f, speed=%.2f, acc=%.2f",
                     cmd->payload.joint_angle.joint, cmd->payload.joint_angle.angle,
                     cmd->payload.joint_angle.speed, cmd->payload.joint_angle.acceleration);
            {
                // Control single joint by angle
                uint16_t position = servo_controller_angle_to_position((float)cmd->payload.joint_angle.angle);
                servo_controller_set_position(cmd->payload.joint_angle.joint, position, (uint16_t)cmd->payload.joint_angle.speed);
                servo_controller_set_acceleration(cmd->payload.joint_angle.joint, (uint16_t)cmd->payload.joint_angle.acceleration);
            }
            break;
            
        case CMD_JOINTS_ANGLE_CTRL:
            ESP_LOGI(TAG, "Joints angle control: base=%.2f, shoulder=%.2f, elbow=%.2f, hand=%.2f, speed=%.2f, acc=%.2f",
                     cmd->payload.joints_angle.base, cmd->payload.joints_angle.shoulder,
                     cmd->payload.joints_angle.elbow, cmd->payload.joints_angle.hand,
                     cmd->payload.joints_angle.speed, cmd->payload.joints_angle.acceleration);
            {
                // Control all joints by angle
                arm_joint_angles_t angles = {
                    .base = (float)cmd->payload.joints_angle.base,
                    .shoulder = (float)cmd->payload.joints_angle.shoulder,
                    .elbow = (float)cmd->payload.joints_angle.elbow,
                    .gripper = (float)cmd->payload.joints_angle.hand
                };
                servo_controller_set_joint_angles(&angles, (uint16_t)cmd->payload.joints_angle.speed);
            }
            break;
            
        case CMD_ARM_CTRL_UI:
            ESP_LOGI(TAG, "Arm control UI command: E=%.2f, Z=%.2f, R=%.2f", 
                     cmd->payload.arm_ui.elevation, cmd->payload.arm_ui.azimuth, cmd->payload.arm_ui.roll);
            {
                // Control arm using UI parameters (elevation, azimuth, roll)
                servo_controller_set_ui_control((float)cmd->payload.arm_ui.elevation, 
                                              (float)cmd->payload.arm_ui.azimuth, 
                                              (float)cmd->payload.arm_ui.roll);
            }
            break;
            
        case CMD_EOAT_HAND_CTRL:
            ESP_LOGI(TAG, "End effector hand control: cmd=%d, speed=%d, acc=%d",
                     cmd->payload.eoat_ctrl.cmd, cmd->payload.eoat_ctrl.speed,
                     cmd->payload.eoat_ctrl.acceleration);
            {
                if (cmd->payload.eoat_ctrl.cmd == 0) {
                    servo_controller_open_gripper(cmd->payload.eoat_ctrl.speed);
                } else if (cmd->payload.eoat_ctrl.cmd == 1) {
                    servo_controller_close_gripper(cmd->payload.eoat_ctrl.speed);
                }
            }
            break;
            
        case CMD_TORQUE_CTRL:
            ESP_LOGI(TAG, "Torque control: torque=%d", cmd->payload.torque_ctrl.torque);
            {
                // Enable/disable torque for all servos based on command
                bool enable = (cmd->payload.torque_ctrl.torque > 0);
                for (int i = 0; i < 5; i++) { // Assuming 5 servos for RoArm-M2
                    servo_controller_enable_torque(i, enable);
                }
            }
            break;
            
        case CMD_SET_JOINT_PID:
            ESP_LOGI(TAG, "Set joint PID: joint=%d, P=%d, I=%d", 
                     cmd->payload.joint_pid.joint, cmd->payload.joint_pid.p, cmd->payload.joint_pid.i);
            {
                // Store PID values in NVS for the specific joint
                char key[32];
                snprintf(key, sizeof(key), "servo_%d_pid_p", cmd->payload.joint_pid.joint);
                
                nvs_handle_t nvs_handle;
                esp_err_t err = nvs_open("servo", NVS_READWRITE, &nvs_handle);
                if (err == ESP_OK) {
                    err = nvs_set_u32(nvs_handle, key, cmd->payload.joint_pid.p);
                    if (err == ESP_OK) {
                        snprintf(key, sizeof(key), "servo_%d_pid_i", cmd->payload.joint_pid.joint);
                        err = nvs_set_u32(nvs_handle, key, cmd->payload.joint_pid.i);
                    }
                    if (err == ESP_OK) {
                        err = nvs_commit(nvs_handle);
                    }
                    nvs_close(nvs_handle);
                    if (err == ESP_OK) {
                        ESP_LOGI(TAG, "PID values saved for servo %d: P=%d, I=%d", 
                                 cmd->payload.joint_pid.joint, cmd->payload.joint_pid.p, cmd->payload.joint_pid.i);
                    } else {
                        ESP_LOGE(TAG, "Failed to save PID values: %s", esp_err_to_name(err));
                    }
                }
            }
            break;
            
        case CMD_RESET_PID:
            ESP_LOGI(TAG, "Reset PID command");
            {
                // Reset PID values for all servos to defaults
                nvs_handle_t nvs_handle;
                esp_err_t err = nvs_open("servo", NVS_READWRITE, &nvs_handle);
                if (err == ESP_OK) {
                    // Default PID values
                    uint32_t default_p = 1000;  // Default P value
                    uint32_t default_i = 100;   // Default I value
                    
                    for (int i = 0; i < 5; i++) { // Assuming 5 servos for RoArm-M2
                        char key[32];
                        snprintf(key, sizeof(key), "servo_%d_pid_p", i);
                        nvs_set_u32(nvs_handle, key, default_p);
                        snprintf(key, sizeof(key), "servo_%d_pid_i", i);
                        nvs_set_u32(nvs_handle, key, default_i);
                    }
                    
                    err = nvs_commit(nvs_handle);
                    nvs_close(nvs_handle);
                    if (err == ESP_OK) {
                        ESP_LOGI(TAG, "PID values reset to defaults for all servos");
                    } else {
                        ESP_LOGE(TAG, "Failed to reset PID values: %s", esp_err_to_name(err));
                    }
                }
            }
            break;
            
        case CMD_DYNAMIC_ADAPTATION:
            ESP_LOGI(TAG, "Dynamic adaptation command: mode=%d", cmd->payload.dynamic_adaptation.mode);
            {
                // Store dynamic adaptation mode in NVS
                nvs_handle_t nvs_handle;
                esp_err_t err = nvs_open("servo", NVS_READWRITE, &nvs_handle);
                if (err == ESP_OK) {
                    err = nvs_set_u8(nvs_handle, "dynamic_adaptation", cmd->payload.dynamic_adaptation.mode);
                    if (err == ESP_OK) {
                        err = nvs_commit(nvs_handle);
                    }
                    nvs_close(nvs_handle);
                    if (err == ESP_OK) {
                        ESP_LOGI(TAG, "Dynamic adaptation mode set to %d", cmd->payload.dynamic_adaptation.mode);
                        // Apply dynamic adaptation based on mode
                        switch (cmd->payload.dynamic_adaptation.mode) {
                            case 0: // Disabled
                                ESP_LOGI(TAG, "Dynamic adaptation disabled");
                                break;
                            case 1: // Basic adaptation
                                ESP_LOGI(TAG, "Basic dynamic adaptation enabled");
                                break;
                            case 2: // Advanced adaptation
                                ESP_LOGI(TAG, "Advanced dynamic adaptation enabled");
                                break;
                            default:
                                ESP_LOGW(TAG, "Unknown dynamic adaptation mode: %d", cmd->payload.dynamic_adaptation.mode);
                                break;
                        }
                    } else {
                        ESP_LOGE(TAG, "Failed to set dynamic adaptation mode: %s", esp_err_to_name(err));
                    }
                }
            }
            break;
            
        // Gimbal Control Commands
        case CMD_GIMBAL_CTRL_SIMPLE:
            ESP_LOGI(TAG, "Gimbal simple control: x=%.2f, y=%.2f, speed=%d, acc=%d",
                     cmd->payload.gimbal_simple.x, cmd->payload.gimbal_simple.y,
                     cmd->payload.gimbal_simple.speed, cmd->payload.gimbal_simple.acceleration);
            gimbal_controller_set_pan_tilt(cmd->payload.gimbal_simple.x, cmd->payload.gimbal_simple.y);
            gimbal_controller_set_speed(cmd->payload.gimbal_simple.speed, cmd->payload.gimbal_simple.speed);
            gimbal_controller_set_acceleration(cmd->payload.gimbal_simple.acceleration, cmd->payload.gimbal_simple.acceleration);
            break;
            
        case CMD_GIMBAL_CTRL_MOVE:
            ESP_LOGI(TAG, "Gimbal move control: x=%.2f, y=%.2f, speed_x=%d, speed_y=%d",
                     cmd->payload.gimbal_move.x, cmd->payload.gimbal_move.y,
                     cmd->payload.gimbal_move.speed_x, cmd->payload.gimbal_move.speed_y);
            gimbal_controller_set_pan_tilt(cmd->payload.gimbal_move.x, cmd->payload.gimbal_move.y);
            gimbal_controller_set_speed(cmd->payload.gimbal_move.speed_x, cmd->payload.gimbal_move.speed_y);
            break;
            
        case CMD_GIMBAL_CTRL_STOP:
            ESP_LOGI(TAG, "Gimbal stop command");
            gimbal_controller_stop();
            break;
            
        case CMD_GIMBAL_STEADY:
            ESP_LOGI(TAG, "Gimbal steady mode: %s, y_goal=%.2f",
                     cmd->payload.gimbal_steady.steady ? "ON" : "OFF",
                     cmd->payload.gimbal_steady.y_goal);
            gimbal_controller_enable_steady_mode(cmd->payload.gimbal_steady.steady);
            if (cmd->payload.gimbal_steady.steady) {
                gimbal_controller_set_steady_goal(0.0f, cmd->payload.gimbal_steady.y_goal);
            }
            break;
            
        case CMD_GIMBAL_USER_CTRL:
            ESP_LOGI(TAG, "Gimbal user control: x=%.2f, y=%.2f, speed=%d",
                     cmd->payload.gimbal_user.x, cmd->payload.gimbal_user.y,
                     cmd->payload.gimbal_user.speed);
            gimbal_controller_set_pan_tilt(cmd->payload.gimbal_user.x, cmd->payload.gimbal_user.y);
            gimbal_controller_set_speed(cmd->payload.gimbal_user.speed, cmd->payload.gimbal_user.speed);
            break;
            
        // File & Mission System Commands
        case CMD_SCAN_FILES:
            ESP_LOGI(TAG, "Scan files command");
            files_controller_scan_contents();
            break;
            
        case CMD_CREATE_FILE:
            ESP_LOGI(TAG, "Create file: %s", cmd->payload.file_op.name);
            files_controller_create_file(cmd->payload.file_op.name, "");
            break;
            
        case CMD_READ_FILE:
            ESP_LOGI(TAG, "Read file: %s", cmd->payload.file_op.name);
            {
                char content[1024];
                if (files_controller_read_file(cmd->payload.file_op.name, content, sizeof(content)) == ESP_OK) {
                    ESP_LOGI(TAG, "File content: %s", content);
                }
            }
            break;
            
        case CMD_DELETE_FILE:
            ESP_LOGI(TAG, "Delete file: %s", cmd->payload.file_op.name);
            files_controller_delete_file(cmd->payload.file_op.name);
            break;
            
        case CMD_APPEND_LINE:
            ESP_LOGI(TAG, "Append line to file: %s", cmd->payload.file_op.name);
            if (files_controller_append_line(cmd->payload.file_op.name, cmd->payload.file_op.content) == ESP_OK) {
                ESP_LOGI(TAG, "Line appended to file %s", cmd->payload.file_op.name);
            }
            break;
            
        case CMD_INSERT_LINE:
            ESP_LOGI(TAG, "Insert line in file: %s at line %d", cmd->payload.file_op.name, cmd->payload.file_op.line_num);
            if (files_controller_insert_line(cmd->payload.file_op.name, cmd->payload.file_op.line_num, cmd->payload.file_op.content) == ESP_OK) {
                ESP_LOGI(TAG, "Line inserted at position %d in file %s", cmd->payload.file_op.line_num, cmd->payload.file_op.name);
            }
            break;
            
        case CMD_REPLACE_LINE:
            ESP_LOGI(TAG, "Replace line in file: %s at line %d", cmd->payload.file_op.name, cmd->payload.file_op.line_num);
            if (files_controller_replace_line(cmd->payload.file_op.name, cmd->payload.file_op.line_num, cmd->payload.file_op.content) == ESP_OK) {
                ESP_LOGI(TAG, "Line replaced at position %d in file %s", cmd->payload.file_op.line_num, cmd->payload.file_op.name);
            }
            break;
            
        case CMD_READ_LINE:
            ESP_LOGI(TAG, "Read line from file: %s at line %d", cmd->payload.file_op.name, cmd->payload.file_op.line_num);
            {
                char line_content[256];
                if (files_controller_read_line(cmd->payload.file_op.name, cmd->payload.file_op.line_num, line_content, sizeof(line_content)) == ESP_OK) {
                    ESP_LOGI(TAG, "Line %d content: %s", cmd->payload.file_op.line_num, line_content);
                }
            }
            break;
            
        case CMD_DELETE_LINE:
            ESP_LOGI(TAG, "Delete line from file: %s at line %d", cmd->payload.file_op.name, cmd->payload.file_op.line_num);
            if (files_controller_delete_line(cmd->payload.file_op.name, cmd->payload.file_op.line_num) == ESP_OK) {
                ESP_LOGI(TAG, "Line %d deleted from file %s", cmd->payload.file_op.line_num, cmd->payload.file_op.name);
            }
            break;
            
        case CMD_CREATE_MISSION:
            ESP_LOGI(TAG, "Create mission: %s - %s", cmd->payload.mission_op.name, cmd->payload.mission_op.intro);
            files_controller_create_mission(cmd->payload.mission_op.name, cmd->payload.mission_op.intro);
            break;
            
        case CMD_MISSION_PLAY:
            ESP_LOGI(TAG, "Play mission command");
            {
                // For now, we'll play the default mission or the last created mission
                // In a full implementation, this would accept a mission name parameter
                ESP_LOGI(TAG, "Starting mission execution via UGV advanced controller");
                
                // Set UGV to mission mode
                // ugv_advanced_set_auto_mode(true); // Function not yet implemented
                
                // Start mission execution (this would typically load and execute a specific mission file)
                ESP_LOGI(TAG, "Mission mode activated - ready for execution");
                
                // Note: Full mission execution would require:
                // 1. Mission name parameter in the command
                // 2. Mission file loading from SPIFFS
                // 3. Step-by-step execution via UGV advanced controller
            }
            break;
            
        case CMD_MISSION_CONTENT:
            ESP_LOGI(TAG, "Get mission content command");
            {
                // Get mission content from files controller
                char content[1024];
                if (files_controller_read_file("current_mission.json", content, sizeof(content)) == ESP_OK) {
                    ESP_LOGI(TAG, "Mission content: %s", content);
                } else {
                    ESP_LOGW(TAG, "No current mission found");
                }
            }
            break;
            
        case CMD_APPEND_STEP_JSON:
            ESP_LOGI(TAG, "Append step JSON command");
            {
                // Append step to mission file
                char filename[64];
                snprintf(filename, sizeof(filename), "%s.json", cmd->payload.mission_op.name);
                if (files_controller_append_line(filename, cmd->payload.mission_op.step) == ESP_OK) {
                    ESP_LOGI(TAG, "Step appended to mission %s", cmd->payload.mission_op.name);
                }
            }
            break;
            
        case CMD_APPEND_STEP_FB:
            ESP_LOGI(TAG, "Append step feedback command");
            {
                // Append step using feedback data
                char step_json[256];
                snprintf(step_json, sizeof(step_json), "{\"T\":104,\"spd\":%.2f}", cmd->payload.mission_op.speed);
                char filename[64];
                snprintf(filename, sizeof(filename), "%s.json", cmd->payload.mission_op.name);
                if (files_controller_append_line(filename, step_json) == ESP_OK) {
                    ESP_LOGI(TAG, "Feedback step appended to mission %s", cmd->payload.mission_op.name);
                }
            }
            break;
            
        case CMD_APPEND_DELAY:
            ESP_LOGI(TAG, "Append delay command");
            {
                // Append delay step to mission
                char step_json[256];
                snprintf(step_json, sizeof(step_json), "{\"T\":111,\"cmd\":%d}", cmd->payload.mission_op.delay);
                char filename[64];
                snprintf(filename, sizeof(filename), "%s.json", cmd->payload.mission_op.name);
                if (files_controller_append_line(filename, step_json) == ESP_OK) {
                    ESP_LOGI(TAG, "Delay step appended to mission %s", cmd->payload.mission_op.name);
                }
            }
            break;
            
        case CMD_INSERT_STEP_JSON:
            ESP_LOGI(TAG, "Insert step JSON command");
            {
                // Insert step at specific position
                char filename[64];
                snprintf(filename, sizeof(filename), "%s.json", cmd->payload.mission_op.name);
                if (files_controller_insert_line(filename, cmd->payload.mission_op.line_num, cmd->payload.mission_op.step) == ESP_OK) {
                    ESP_LOGI(TAG, "Step inserted at line %d in mission %s", cmd->payload.mission_op.line_num, cmd->payload.mission_op.name);
                }
            }
            break;
            
        case CMD_INSERT_STEP_FB:
            ESP_LOGI(TAG, "Insert step feedback command");
            {
                // Insert step using feedback data
                char step_json[256];
                snprintf(step_json, sizeof(step_json), "{\"T\":104,\"spd\":%.2f}", cmd->payload.mission_op.speed);
                char filename[64];
                snprintf(filename, sizeof(filename), "%s.json", cmd->payload.mission_op.name);
                if (files_controller_insert_line(filename, cmd->payload.mission_op.line_num, step_json) == ESP_OK) {
                    ESP_LOGI(TAG, "Feedback step inserted at line %d in mission %s", cmd->payload.mission_op.line_num, cmd->payload.mission_op.name);
                }
            }
            break;
            
        case CMD_INSERT_DELAY:
            ESP_LOGI(TAG, "Insert delay command");
            {
                // Insert delay step at specific position
                char step_json[256];
                snprintf(step_json, sizeof(step_json), "{\"T\":111,\"cmd\":%d}", cmd->payload.mission_op.delay);
                char filename[64];
                snprintf(filename, sizeof(filename), "%s.json", cmd->payload.mission_op.name);
                if (files_controller_insert_line(filename, cmd->payload.mission_op.line_num, step_json) == ESP_OK) {
                    ESP_LOGI(TAG, "Delay step inserted at line %d in mission %s", cmd->payload.mission_op.line_num, cmd->payload.mission_op.name);
                }
            }
            break;
            
        case CMD_REPLACE_STEP_JSON:
            ESP_LOGI(TAG, "Replace step JSON command");
            {
                // Replace step at specific position
                char filename[64];
                snprintf(filename, sizeof(filename), "%s.json", cmd->payload.mission_op.name);
                if (files_controller_replace_line(filename, cmd->payload.mission_op.line_num, cmd->payload.mission_op.step) == ESP_OK) {
                    ESP_LOGI(TAG, "Step replaced at line %d in mission %s", cmd->payload.mission_op.line_num, cmd->payload.mission_op.name);
                }
            }
            break;
            
        case CMD_REPLACE_STEP_FB:
            ESP_LOGI(TAG, "Replace step feedback command");
            {
                // Replace step using feedback data
                char step_json[256];
                snprintf(step_json, sizeof(step_json), "{\"T\":104,\"spd\":%.2f}", cmd->payload.mission_op.speed);
                char filename[64];
                snprintf(filename, sizeof(filename), "%s.json", cmd->payload.mission_op.name);
                if (files_controller_replace_line(filename, cmd->payload.mission_op.line_num, step_json) == ESP_OK) {
                    ESP_LOGI(TAG, "Feedback step replaced at line %d in mission %s", cmd->payload.mission_op.line_num, cmd->payload.mission_op.name);
                }
            }
            break;
            
        case CMD_REPLACE_DELAY:
            ESP_LOGI(TAG, "Replace delay command");
            {
                // Replace step with delay at specific position
                char step_json[256];
                snprintf(step_json, sizeof(step_json), "{\"T\":111,\"cmd\":%d}", cmd->payload.mission_op.delay);
                char filename[64];
                snprintf(filename, sizeof(filename), "%s.json", cmd->payload.mission_op.name);
                if (files_controller_replace_line(filename, cmd->payload.mission_op.line_num, step_json) == ESP_OK) {
                    ESP_LOGI(TAG, "Step replaced with delay at line %d in mission %s", cmd->payload.mission_op.line_num, cmd->payload.mission_op.name);
                }
            }
            break;
            
        case CMD_DELETE_STEP:
            ESP_LOGI(TAG, "Delete step command");
            {
                // Delete step at specific position
                char filename[64];
                snprintf(filename, sizeof(filename), "%s.json", cmd->payload.mission_op.name);
                if (files_controller_delete_line(filename, cmd->payload.mission_op.line_num) == ESP_OK) {
                    ESP_LOGI(TAG, "Step deleted at line %d in mission %s", cmd->payload.mission_op.line_num, cmd->payload.mission_op.name);
                }
            }
            break;
            
        case CMD_MOVE_TO_STEP:
            ESP_LOGI(TAG, "Move to step command");
            {
                // Move to specific step in mission
                ESP_LOGI(TAG, "Moving to step %d in mission %s", cmd->payload.mission_op.line_num, cmd->payload.mission_op.name);
                // This would interface with the mission execution system
                // ugv_advanced_move_to_step(cmd->payload.mission_op.name, cmd->payload.mission_op.line_num);
            }
            break;
            
        // ESP-NOW Communication Commands
        case CMD_ESP_NOW_CONFIG:
            ESP_LOGI(TAG, "ESP-NOW config: mode=%d", cmd->payload.esp_now_op.mode);
            esp_now_controller_set_mode(cmd->payload.esp_now_op.mode);
            break;
            
        case CMD_GET_MAC_ADDRESS:
            ESP_LOGI(TAG, "Get MAC address command");
            {
                uint8_t mac[6];
                esp_now_controller_get_this_mac(mac);
                ESP_LOGI(TAG, "MAC Address: %02X:%02X:%02X:%02X:%02X:%02X", 
                         mac[0], mac[1], mac[2], mac[3], mac[4], mac[5]);
            }
            break;
            
        case CMD_BROADCAST_FOLLOWER:
            ESP_LOGI(TAG, "Broadcast follower command");
            {
                // Configure broadcast follower mode
                if (cmd->payload.esp_now_op.mode == 0) {
                    // Add specific MAC to whitelist
                    ESP_LOGI(TAG, "Adding MAC %02X:%02X:%02X:%02X:%02X:%02X to whitelist",
                             cmd->payload.esp_now_op.mac[0], cmd->payload.esp_now_op.mac[1],
                             cmd->payload.esp_now_op.mac[2], cmd->payload.esp_now_op.mac[3],
                             cmd->payload.esp_now_op.mac[4], cmd->payload.esp_now_op.mac[5]);
                    esp_now_controller_add_peer(cmd->payload.esp_now_op.mac);
                } else {
                    ESP_LOGI(TAG, "Broadcast follower mode enabled");
                }
            }
            break;
            
        case CMD_ESP_NOW_ADD_FOLLOWER:
            ESP_LOGI(TAG, "Add ESP-NOW follower command");
            {
                ESP_LOGI(TAG, "Adding follower MAC %02X:%02X:%02X:%02X:%02X:%02X",
                         cmd->payload.esp_now_op.mac[0], cmd->payload.esp_now_op.mac[1],
                         cmd->payload.esp_now_op.mac[2], cmd->payload.esp_now_op.mac[3],
                         cmd->payload.esp_now_op.mac[4], cmd->payload.esp_now_op.mac[5]);
                esp_now_controller_add_peer(cmd->payload.esp_now_op.mac);
            }
            break;
            
        case CMD_ESP_NOW_REMOVE_FOLLOWER:
            ESP_LOGI(TAG, "Remove ESP-NOW follower command");
            {
                ESP_LOGI(TAG, "Removing follower MAC %02X:%02X:%02X:%02X:%02X:%02X",
                         cmd->payload.esp_now_op.mac[0], cmd->payload.esp_now_op.mac[1],
                         cmd->payload.esp_now_op.mac[2], cmd->payload.esp_now_op.mac[3],
                         cmd->payload.esp_now_op.mac[4], cmd->payload.esp_now_op.mac[5]);
                esp_now_controller_remove_peer(cmd->payload.esp_now_op.mac);
            }
            break;
            
        case CMD_ESP_NOW_GROUP_CTRL:
            ESP_LOGI(TAG, "ESP-NOW group control command");
            {
                // Send control command to group of devices
                ESP_LOGI(TAG, "Group control: device=%d, base=%.2f, shoulder=%.2f, elbow=%.2f, hand=%.2f, cmd=%d",
                         cmd->payload.esp_now_group.dev, cmd->payload.esp_now_group.b,
                         cmd->payload.esp_now_group.s, cmd->payload.esp_now_group.e,
                         cmd->payload.esp_now_group.h, cmd->payload.esp_now_group.cmd);
                // This would send the command to all registered peers
                esp_now_controller_send_json_command("group_control");
            }
            break;
            
        case CMD_ESP_NOW_SINGLE:
            ESP_LOGI(TAG, "ESP-NOW single control command");
            {
                // Send control command to single device using available payload
                ESP_LOGI(TAG, "Single control command received");
                esp_now_controller_send_json_command("single_control");
            }
            break;
            
        // WiFi Configuration Commands
        case CMD_WIFI_ON_BOOT:
            ESP_LOGI(TAG, "WiFi boot config: mode=%d", cmd->payload.wifi_config.mode);
            {
                // Store WiFi boot mode in NVS
                nvs_handle_t nvs_handle;
                esp_err_t err = nvs_open("wifi", NVS_READWRITE, &nvs_handle);
                if (err == ESP_OK) {
                    err = nvs_set_u8(nvs_handle, "boot_mode", cmd->payload.wifi_config.mode);
                    if (err == ESP_OK) {
                        err = nvs_commit(nvs_handle);
                    }
                    nvs_close(nvs_handle);
                    if (err == ESP_OK) {
                        ESP_LOGI(TAG, "WiFi boot mode set to %d", cmd->payload.wifi_config.mode);
                        // Apply boot mode configuration
                        switch (cmd->payload.wifi_config.mode) {
                            case 0: // WiFi disabled on boot
                                ESP_LOGI(TAG, "WiFi will be disabled on next boot");
                                break;
                            case 1: // WiFi enabled on boot
                                ESP_LOGI(TAG, "WiFi will be enabled on next boot");
                                break;
                            case 2: // AP mode on boot
                                ESP_LOGI(TAG, "WiFi AP mode will be enabled on next boot");
                                break;
                            default:
                                ESP_LOGW(TAG, "Unknown WiFi boot mode: %d", cmd->payload.wifi_config.mode);
                                break;
                        }
                    } else {
                        ESP_LOGE(TAG, "Failed to set WiFi boot mode: %s", esp_err_to_name(err));
                    }
                }
            }
            break;
            
        case CMD_SET_AP:
            ESP_LOGI(TAG, "Set AP mode: %s", cmd->payload.wifi_config.ssid);
            {
                // Store AP configuration in NVS
                nvs_handle_t nvs_handle;
                esp_err_t err = nvs_open("wifi", NVS_READWRITE, &nvs_handle);
                if (err == ESP_OK) {
                    err = nvs_set_str(nvs_handle, "ap_ssid", cmd->payload.wifi_config.ssid);
                    if (err == ESP_OK) {
                        // Set default password and channel
                        const char *default_password = "rasprover123";
                        uint8_t default_channel = 1;
                        err = nvs_set_str(nvs_handle, "ap_password", default_password);
                        if (err == ESP_OK) {
                            err = nvs_set_u8(nvs_handle, "ap_channel", default_channel);
                        }
                    }
                    if (err == ESP_OK) {
                        err = nvs_commit(nvs_handle);
                    }
                    nvs_close(nvs_handle);
                    if (err == ESP_OK) {
                        ESP_LOGI(TAG, "AP configuration stored: SSID=%s, Channel=%d", 
                                 cmd->payload.wifi_config.ssid, 1);
                        // Note: AP will be started on next boot or when explicitly called
                        ESP_LOGI(TAG, "AP mode configured - use wifi_controller_start_ap() to activate");
                    } else {
                        ESP_LOGE(TAG, "Failed to store AP configuration: %s", esp_err_to_name(err));
                    }
                }
            }
            break;
            
        case CMD_SET_STA:
            ESP_LOGI(TAG, "Set STA mode: %s", cmd->payload.wifi_config.ssid);
            wifi_controller_connect(cmd->payload.wifi_config.ssid, "");
            break;
            
        case CMD_WIFI_INFO:
            ESP_LOGI(TAG, "Get WiFi info command");
            {
                bool connected = wifi_controller_is_connected();
                bool ap_active = wifi_controller_is_ap_active();
                ESP_LOGI(TAG, "WiFi Status: Connected=%s, AP Active=%s", 
                         connected ? "Yes" : "No", ap_active ? "Yes" : "No");
            }
            break;
            
        case CMD_WIFI_APSTA:
            ESP_LOGI(TAG, "WiFi AP+STA mode command");
            {
                // Configure AP+STA mode
                ESP_LOGI(TAG, "Configuring AP+STA mode: AP=%s, STA=%s", 
                         cmd->payload.wifi_apsta.ap_ssid, cmd->payload.wifi_apsta.sta_ssid);
                
                // Start AP mode (using default parameters since function doesn't take parameters)
                wifi_controller_start_ap();
                
                // Connect to STA
                wifi_controller_connect(cmd->payload.wifi_apsta.sta_ssid, cmd->payload.wifi_apsta.sta_password);
            }
            break;
            
        case CMD_WIFI_CONFIG_CREATE_BY_STATUS:
            ESP_LOGI(TAG, "Create WiFi config from status command");
            {
                // Create WiFi config file from current status
                char config_content[512];
                snprintf(config_content, sizeof(config_content), 
                         "{\"mode\":3,\"ap_ssid\":\"RaspRover\",\"ap_password\":\"rasprover123\","
                         "\"sta_ssid\":\"\",\"sta_password\":\"\"}");
                
                if (files_controller_create_file("wifi_config.json", config_content) == ESP_OK) {
                    ESP_LOGI(TAG, "WiFi config file created from current status");
                }
            }
            break;
            
        case CMD_WIFI_CONFIG_CREATE_BY_INPUT:
            ESP_LOGI(TAG, "Create WiFi config from input command");
            {
                // Create WiFi config file from input parameters
                char config_content[512];
                snprintf(config_content, sizeof(config_content), 
                         "{\"mode\":%d,\"ap_ssid\":\"%s\",\"ap_password\":\"%s\","
                         "\"sta_ssid\":\"%s\",\"sta_password\":\"%s\"}",
                         cmd->payload.wifi_config_create.mode,
                         cmd->payload.wifi_config_create.ap_ssid,
                         cmd->payload.wifi_config_create.ap_password,
                         cmd->payload.wifi_config_create.sta_ssid,
                         cmd->payload.wifi_config_create.sta_password);
                
                if (files_controller_create_file("wifi_config.json", config_content) == ESP_OK) {
                    ESP_LOGI(TAG, "WiFi config file created from input parameters");
                }
            }
            break;
            
        case CMD_WIFI_STOP:
            ESP_LOGI(TAG, "Stop WiFi command");
            {
                // Stop WiFi connections
                wifi_controller_disconnect();
                wifi_controller_stop();
                ESP_LOGI(TAG, "WiFi stopped");
            }
            break;
            
        // Servo Configuration Commands
        case CMD_SET_SERVO_ID:
            ESP_LOGI(TAG, "Set servo ID: %d -> %d", cmd->payload.servo_config.raw_id, cmd->payload.servo_config.new_id);
            {
                // Store servo ID mapping in NVS
                nvs_handle_t nvs_handle;
                esp_err_t err = nvs_open("servo", NVS_READWRITE, &nvs_handle);
                if (err == ESP_OK) {
                    char key[32];
                    snprintf(key, sizeof(key), "servo_%d_new_id", cmd->payload.servo_config.raw_id);
                    err = nvs_set_u8(nvs_handle, key, cmd->payload.servo_config.new_id);
                    if (err == ESP_OK) {
                        err = nvs_commit(nvs_handle);
                    }
                    nvs_close(nvs_handle);
                    if (err == ESP_OK) {
                        ESP_LOGI(TAG, "Servo ID mapping stored: %d -> %d", 
                                 cmd->payload.servo_config.raw_id, cmd->payload.servo_config.new_id);
                        ESP_LOGI(TAG, "Note: Physical servo ID change requires servo-specific commands");
                    } else {
                        ESP_LOGE(TAG, "Failed to store servo ID mapping: %s", esp_err_to_name(err));
                    }
                }
            }
            break;
            
        case CMD_SET_MIDDLE:
            ESP_LOGI(TAG, "Set middle position for servo: %d", cmd->payload.servo_config.raw_id);
            {
                // Store middle position for servo in NVS
                nvs_handle_t nvs_handle;
                esp_err_t err = nvs_open("servo", NVS_READWRITE, &nvs_handle);
                if (err == ESP_OK) {
                    char key[32];
                    snprintf(key, sizeof(key), "servo_%d_middle", cmd->payload.servo_config.raw_id);
                    // Default middle position is 2048 (center of 0-4095 range)
                    uint16_t middle_pos = 2048;
                    err = nvs_set_u16(nvs_handle, key, middle_pos);
                    if (err == ESP_OK) {
                        err = nvs_commit(nvs_handle);
                    }
                    nvs_close(nvs_handle);
                    if (err == ESP_OK) {
                        ESP_LOGI(TAG, "Middle position set for servo %d: %d", 
                                 cmd->payload.servo_config.raw_id, middle_pos);
                        // Move servo to middle position
                        servo_controller_set_position(cmd->payload.servo_config.raw_id, middle_pos, 1000);
                    } else {
                        ESP_LOGE(TAG, "Failed to set middle position: %s", esp_err_to_name(err));
                    }
                }
            }
            break;
            
        case CMD_SET_SERVO_PID:
            ESP_LOGI(TAG, "Set servo PID: id=%d, P=%d", cmd->payload.servo_config.raw_id, cmd->payload.servo_config.p_value);
            {
                // Store servo PID values in NVS
                nvs_handle_t nvs_handle;
                esp_err_t err = nvs_open("servo", NVS_READWRITE, &nvs_handle);
                if (err == ESP_OK) {
                    char key[32];
                    snprintf(key, sizeof(key), "servo_%d_pid_p", cmd->payload.servo_config.raw_id);
                    err = nvs_set_u32(nvs_handle, key, cmd->payload.servo_config.p_value);
                    if (err == ESP_OK) {
                        // Set default I value if not provided
                        uint32_t default_i = 100;
                        snprintf(key, sizeof(key), "servo_%d_pid_i", cmd->payload.servo_config.raw_id);
                        err = nvs_set_u32(nvs_handle, key, default_i);
                    }
                    if (err == ESP_OK) {
                        err = nvs_commit(nvs_handle);
                    }
                    nvs_close(nvs_handle);
                    if (err == ESP_OK) {
                        ESP_LOGI(TAG, "PID values set for servo %d: P=%d, I=%d", 
                                 cmd->payload.servo_config.raw_id, cmd->payload.servo_config.p_value, 100);
                    } else {
                        ESP_LOGE(TAG, "Failed to set servo PID values: %s", esp_err_to_name(err));
                    }
                }
            }
            break;
            
        // System Control Commands
        case CMD_REBOOT:
            ESP_LOGI(TAG, "Reboot command received");
            esp_restart();
            break;
            
        case CMD_FREE_FLASH_SPACE:
            ESP_LOGI(TAG, "Get free flash space command");
            {
                size_t total, used;
                if (esp_spiffs_info(SPIFFS_PARTITION_LABEL, &total, &used) == ESP_OK) {
                    ESP_LOGI(TAG, "Flash Space: Total=%u bytes, Used=%u bytes, Free=%u bytes", 
                             total, used, total - used);
                }
            }
            break;
            
        case CMD_NVS_CLEAR:
            ESP_LOGW(TAG, "Clear NVS command received");
            nvs_flash_erase();
            nvs_flash_init();
            break;
            
        case CMD_BOOT_MISSION_INFO:
            ESP_LOGI(TAG, "Get boot mission info command");
            {
                // Get boot mission information
                char boot_mission[64];
                if (files_controller_read_file("boot_mission.json", boot_mission, sizeof(boot_mission)) == ESP_OK) {
                    ESP_LOGI(TAG, "Boot mission: %s", boot_mission);
                } else {
                    ESP_LOGW(TAG, "No boot mission configured");
                }
            }
            break;
            
        case CMD_RESET_BOOT_MISSION:
            ESP_LOGI(TAG, "Reset boot mission command");
            {
                // Reset boot mission configuration
                if (files_controller_delete_file("boot_mission.json") == ESP_OK) {
                    ESP_LOGI(TAG, "Boot mission reset");
                }
            }
            break;
            
        case CMD_INFO_PRINT:
            ESP_LOGI(TAG, "Configure info print command");
            {
                // Store info print level in NVS
                nvs_handle_t nvs_handle;
                esp_err_t err = nvs_open("system", NVS_READWRITE, &nvs_handle);
                if (err == ESP_OK) {
                    // Default to INFO level (3)
                    uint8_t log_level = 3;
                    err = nvs_set_u8(nvs_handle, "log_level", log_level);
                    if (err == ESP_OK) {
                        err = nvs_commit(nvs_handle);
                    }
                    nvs_close(nvs_handle);
                    if (err == ESP_OK) {
                        ESP_LOGI(TAG, "Info print level configured to INFO (3)");
                        // Note: In a full implementation, this would set the ESP_LOG_LEVEL
                        // esp_log_set_level("*", ESP_LOG_INFO);
                    } else {
                        ESP_LOGE(TAG, "Failed to configure info print level: %s", esp_err_to_name(err));
                    }
                }
            }
            break;
            
        // Additional Commands
            
        case CMD_GET_IMU_DATA:
            ESP_LOGI(TAG, "Get IMU data command");
            {
                imu_data_t imu_data;
                if (imu_controller_read_data(&imu_data) == ESP_OK) {
                    ESP_LOGI(TAG, "IMU Data: Roll=%.2f, Pitch=%.2f, Yaw=%.2f", 
                             imu_data.roll, imu_data.pitch, imu_data.yaw);
                }
            }
            break;
            
        case CMD_CALI_IMU_STEP:
            ESP_LOGI(TAG, "IMU calibration step command");
            imu_controller_calibrate();
            break;
            
        case CMD_GET_IMU_OFFSET:
            ESP_LOGI(TAG, "Get IMU offset command");
            {
                imu_calibration_t cal;
                if (imu_controller_get_calibration(&cal) == ESP_OK) {
                    ESP_LOGI(TAG, "IMU Offsets: Accel X=%.2f, Y=%.2f, Z=%.2f", 
                             cal.accel_bias_x, cal.accel_bias_y, cal.accel_bias_z);
                }
            }
            break;
            
        case CMD_SET_IMU_OFFSET:
            ESP_LOGI(TAG, "Set IMU offset command");
            {
                // Store IMU offset values in NVS
                nvs_handle_t nvs_handle;
                esp_err_t err = nvs_open("imu", NVS_READWRITE, &nvs_handle);
                if (err == ESP_OK) {
                    // Note: In a full implementation, the command payload would contain offset values
                    // For now, we'll store default calibration values
                    float default_offset_x = 0.0f;
                    float default_offset_y = 0.0f;
                    float default_offset_z = 0.0f;
                    
                    err = nvs_set_blob(nvs_handle, "accel_offset_x", &default_offset_x, sizeof(float));
                    if (err == ESP_OK) {
                        err = nvs_set_blob(nvs_handle, "accel_offset_y", &default_offset_y, sizeof(float));
                    }
                    if (err == ESP_OK) {
                        err = nvs_set_blob(nvs_handle, "accel_offset_z", &default_offset_z, sizeof(float));
                    }
                    if (err == ESP_OK) {
                        err = nvs_commit(nvs_handle);
                    }
                    nvs_close(nvs_handle);
                    if (err == ESP_OK) {
                        ESP_LOGI(TAG, "IMU offset values set to defaults: X=%.2f, Y=%.2f, Z=%.2f", 
                                 default_offset_x, default_offset_y, default_offset_z);
                        ESP_LOGI(TAG, "Note: Full implementation would use values from command payload");
                    } else {
                        ESP_LOGE(TAG, "Failed to set IMU offset values: %s", esp_err_to_name(err));
                    }
                }
            }
            break;
            
        case CMD_BASE_FEEDBACK:
            ESP_LOGI(TAG, "Base feedback command");
            {
                motion_control_t motion_status;
                imu_data_t imu_data;
                float battery_voltage;
                uint8_t battery_percentage;
                
                if (motion_module_get_status(&motion_status) == ESP_OK) {
                    ESP_LOGI(TAG, "Motion Status: L=%.2f, R=%.2f", 
                             motion_status.left_speed, motion_status.right_speed);
                }
                
                if (imu_controller_read_data(&imu_data) == ESP_OK) {
                    ESP_LOGI(TAG, "IMU Status: Roll=%.2f, Pitch=%.2f", 
                             imu_data.roll, imu_data.pitch);
                }
                
                if (battery_controller_read_voltage(&battery_voltage) == ESP_OK) {
                    battery_percentage = battery_controller_get_percentage();
                    ESP_LOGI(TAG, "Battery Status: %.1fV, %d%%", 
                             battery_voltage, battery_percentage);
                }
            }
            break;
            
        case CMD_BASE_FEEDBACK_FLOW:
            ESP_LOGI(TAG, "Set base feedback flow command");
            {
                // Store base feedback flow configuration in NVS
                nvs_handle_t nvs_handle;
                esp_err_t err = nvs_open("uart", NVS_READWRITE, &nvs_handle);
                if (err == ESP_OK) {
                    // Enable base feedback flow by default
                    bool enable_feedback = true;
                    err = nvs_set_u8(nvs_handle, "base_feedback_flow", enable_feedback ? 1 : 0);
                    if (err == ESP_OK) {
                        err = nvs_commit(nvs_handle);
                    }
                    nvs_close(nvs_handle);
                    if (err == ESP_OK) {
                        uart_control.base_feedback_flow = enable_feedback;
                        ESP_LOGI(TAG, "Base feedback flow %s", enable_feedback ? "enabled" : "disabled");
                    } else {
                        ESP_LOGE(TAG, "Failed to configure base feedback flow: %s", esp_err_to_name(err));
                    }
                }
            }
            break;
            
        case CMD_LED_CTRL:
            ESP_LOGI(TAG, "LED control command");
            {
                // Store LED control configuration in NVS
                nvs_handle_t nvs_handle;
                esp_err_t err = nvs_open("led", NVS_READWRITE, &nvs_handle);
                if (err == ESP_OK) {
                    // Default LED state (on)
                    uint8_t led_state = 1;
                    err = nvs_set_u8(nvs_handle, "led_state", led_state);
                    if (err == ESP_OK) {
                        err = nvs_commit(nvs_handle);
                    }
                    nvs_close(nvs_handle);
                    if (err == ESP_OK) {
                        ESP_LOGI(TAG, "LED control configured: state=%d", led_state);
                        // Note: In a full implementation, this would call led_controller functions
                        // led_controller_set_state(led_state);
                    } else {
                        ESP_LOGE(TAG, "Failed to configure LED control: %s", esp_err_to_name(err));
                    }
                }
            }
            break;
            
        case CMD_SET_SPD_RATE:
            ESP_LOGI(TAG, "Set speed rate command: L=%.2f, R=%.2f",
                     cmd->payload.speed_rate.left_rate, cmd->payload.speed_rate.right_rate);
            motion_module_set_speed_rates(cmd->payload.speed_rate.left_rate, cmd->payload.speed_rate.right_rate);
            break;
            
        case CMD_SWITCH_CTRL:
            ESP_LOGI(TAG, "Switch control command");
            {
                // Control switch outputs with PWM values
                int16_t pwm_a = cmd->payload.switch_ctrl.pwm_a;
                int16_t pwm_b = cmd->payload.switch_ctrl.pwm_b;
                ESP_LOGI(TAG, "Switch A PWM: %d, Switch B PWM: %d", pwm_a, pwm_b);
                // Note: Implementation would control actual switch outputs
            }
            break;
            
        case CMD_LIGHT_CTRL:
            ESP_LOGI(TAG, "Light control command");
            {
                // Control LED brightness
                uint8_t led_brightness = cmd->payload.light_ctrl.led;
                ESP_LOGI(TAG, "LED brightness: %d", led_brightness);
                // Note: Implementation would control actual LED outputs
            }
            break;
            
        case CMD_MM_TYPE_SET:
            ESP_LOGI(TAG, "Main type and module type set command");
            {
                // Set main robot type and module type
                uint8_t main_type = cmd->payload.mm_type.main;
                uint8_t module_type = cmd->payload.mm_type.module;
                ESP_LOGI(TAG, "Main type: %d, Module type: %d", main_type, module_type);
                // Note: Implementation would store these in NVS
            }
            break;
            
        default:
            ESP_LOGW(TAG, "Unknown command type: %d", cmd->type);
            break;
    }
    
    // Send acknowledgment
    uart_controller_send_feedback();
}

void uart_controller_send_feedback(void)
{
    if (!uart_initialized) {
        return;
    }
    
    // Create feedback JSON
    cJSON *feedback = cJSON_CreateObject();
    if (feedback == NULL) {
        ESP_LOGE(TAG, "Failed to create feedback JSON");
        return;
    }
    
    // Add system status
    cJSON_AddStringToObject(feedback, "status", "ok");
    cJSON_AddNumberToObject(feedback, "timestamp", esp_timer_get_time() / 1000);
    
    // Add UART status
    cJSON *uart_status = cJSON_CreateObject();
    cJSON_AddBoolToObject(uart_status, "echo_mode", uart_control.echo_mode);
    cJSON_AddNumberToObject(uart_status, "heartbeat_delay", uart_control.heartbeat_delay_ms);
    cJSON_AddNumberToObject(uart_status, "feedback_interval", uart_control.feedback_interval_ms);
    cJSON_AddBoolToObject(uart_status, "base_feedback_flow", uart_control.base_feedback_flow);
    cJSON_AddItemToObject(feedback, "uart_status", uart_status);
    
    // Convert to string and send
    char *json_string = cJSON_Print(feedback);
    if (json_string != NULL) {
        uart_controller_send_json(json_string);
        free(json_string);
    }
    
    cJSON_Delete(feedback);
}
