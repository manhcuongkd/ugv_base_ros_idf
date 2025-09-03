#include "../inc/json_parser.h"
#include <esp_log.h>
#include <cJSON.h>
#include <freertos/FreeRTOS.h>
#include <freertos/queue.h>
#include <freertos/task.h>
#include <string.h>

static const char *TAG = "JSONParser";

// Private variables
static bool json_parser_initialized = false;
static QueueHandle_t feedback_queue = NULL;

// Private function prototypes
static esp_err_t json_parser_parse_motion_command(const cJSON *cmd_data, json_command_t *command);
static esp_err_t json_parser_parse_servo_command(const cJSON *cmd_data, json_command_t *command);
static esp_err_t json_parser_parse_imu_command(const cJSON *cmd_data, json_command_t *command);
static esp_err_t json_parser_parse_system_command(const cJSON *cmd_data, json_command_t *command);
static esp_err_t json_parser_create_feedback_json(const json_feedback_t *feedback, char *buffer, size_t buffer_size);

esp_err_t json_parser_init(void) {
    if (json_parser_initialized) {
        return ESP_OK;
    }

    ESP_LOGI(TAG, "Initializing JSON parser...");

    // Create feedback queue
    feedback_queue = xQueueCreate(10, sizeof(json_feedback_t));
    if (feedback_queue == NULL) {
        ESP_LOGE(TAG, "Failed to create feedback queue");
        return ESP_FAIL;
    }

    json_parser_initialized = true;
    ESP_LOGI(TAG, "JSON parser initialized successfully");
    return ESP_OK;
}

esp_err_t json_parser_process_command(json_command_t *cmd) {
    if (!json_parser_initialized || cmd == NULL) {
        return ESP_ERR_INVALID_STATE;
    }

    ESP_LOGI(TAG, "Processing JSON command type: %d", cmd->type);

    switch (cmd->type) {
        case CMD_EMERGENCY_STOP:
            ESP_LOGI(TAG, "Emergency stop command received");
            // Handle emergency stop
            break;
            
        case CMD_SPEED_CTRL:
            ESP_LOGI(TAG, "Speed control command: left=%.2f, right=%.2f", 
                     cmd->payload.speed_ctrl.left_speed, 
                     cmd->payload.speed_ctrl.right_speed);
            // Handle speed control
            break;
            
        case CMD_PWM_INPUT:
            ESP_LOGI(TAG, "PWM input command: left=%d, right=%d", 
                     cmd->payload.pwm_input.left_pwm, 
                     cmd->payload.pwm_input.right_pwm);
            // Handle PWM input
            break;
            
        case CMD_ROS_CTRL:
            ESP_LOGI(TAG, "ROS control command: linear=%.2f, angular=%.2f", 
                     cmd->payload.ros_ctrl.linear_speed, 
                     cmd->payload.ros_ctrl.angular_speed);
            // Handle ROS control
            break;
            
        case CMD_SET_MOTOR_PID:
            ESP_LOGI(TAG, "Motor PID command: kp=%.2f, ki=%.2f, kd=%.2f, limit=%.2f", 
                     cmd->payload.pid_params.kp, 
                     cmd->payload.pid_params.ki, 
                     cmd->payload.pid_params.kd,
                     cmd->payload.pid_params.limit);
            // Handle motor PID settings
            break;
            
        case CMD_OLED_CTRL:
            ESP_LOGI(TAG, "OLED control command: line=%d, text=%s", 
                     cmd->payload.oled_ctrl.line_num, 
                     cmd->payload.oled_ctrl.text);
            // Handle OLED control
            break;
            
        case CMD_EOAT_TYPE:
            ESP_LOGI(TAG, "EOAT type command");
            // Handle EOAT type
            break;
            
        case CMD_CONFIG_EOAT:
            ESP_LOGI(TAG, "EOAT configuration command");
            // Handle EOAT configuration
            break;
            
        case CMD_SET_ROBOT_CONFIG:
            ESP_LOGI(TAG, "Robot configuration command: main_type=%d, module_type=%d", 
                     cmd->payload.config.main_type, 
                     cmd->payload.config.module_type);
            // Handle robot configuration
            break;
            
        case CMD_ESP_NOW_RECV:
            ESP_LOGI(TAG, "ESP-NOW receive command");
            // Handle ESP-NOW receive
            break;
            
        case CMD_ESP_NOW_SEND:
            ESP_LOGI(TAG, "ESP-NOW send command");
            // Handle ESP-NOW send
            break;
            
        case CMD_BUS_SERVO_ERROR:
            ESP_LOGI(TAG, "Bus servo error command");
            // Handle bus servo error
            break;
            
        case CMD_RESET_EMERGENCY:
            ESP_LOGI(TAG, "Reset emergency command");
            // Handle emergency reset
            break;
            
        default:
            ESP_LOGW(TAG, "Unknown command type: %d", cmd->type);
            return ESP_ERR_INVALID_ARG;
    }

    return ESP_OK;
}

esp_err_t json_parser_send_base_feedback(void) {
    if (!json_parser_initialized) {
        return ESP_ERR_INVALID_STATE;
    }

    // Create base feedback
    json_feedback_t feedback;
    feedback.type = FEEDBACK_BASE_INFO;
    feedback.timestamp = xTaskGetTickCount() * portTICK_PERIOD_MS;
    
    // Fill in base info data
    feedback.payload.base_info.left_speed = 0.0f;
    feedback.payload.base_info.right_speed = 0.0f;
    feedback.payload.base_info.gyro_x = 0.0f;
    feedback.payload.base_info.gyro_y = 0.0f;
    feedback.payload.base_info.gyro_z = 0.0f;
    feedback.payload.base_info.accel_x = 0.0f;
    feedback.payload.base_info.accel_y = 0.0f;
    feedback.payload.base_info.accel_z = 0.0f;
    feedback.payload.base_info.mag_x = 0.0f;
    feedback.payload.base_info.mag_y = 0.0f;
    feedback.payload.base_info.mag_z = 0.0f;
    feedback.payload.base_info.odometry_left = 0;
    feedback.payload.base_info.odometry_right = 0;
    feedback.payload.base_info.voltage = 12.0f;
    
    // Send feedback (in real implementation, this would go to UART/WiFi)
    ESP_LOGI(TAG, "Base feedback sent");
    
    return ESP_OK;
}

esp_err_t json_parser_create_command(json_command_t *cmd, uint8_t type, void *data) {
    if (!json_parser_initialized || cmd == NULL) {
        return ESP_ERR_INVALID_STATE;
    }

    memset(cmd, 0, sizeof(json_command_t));
    cmd->type = type;
    cmd->timestamp = xTaskGetTickCount() * portTICK_PERIOD_MS;

    // Copy data based on command type
    switch (type) {
        case CMD_SPEED_CTRL:
            if (data) {
                memcpy(&cmd->payload.speed_ctrl, data, sizeof(cmd->payload.speed_ctrl));
            }
            break;
            
        case CMD_PWM_INPUT:
            if (data) {
                memcpy(&cmd->payload.pwm_input, data, sizeof(cmd->payload.pwm_input));
            }
            break;
            
        case CMD_ROS_CTRL:
            if (data) {
                memcpy(&cmd->payload.ros_ctrl, data, sizeof(cmd->payload.ros_ctrl));
            }
            break;
            
        case CMD_SET_MOTOR_PID:
            if (data) {
                memcpy(&cmd->payload.pid_params, data, sizeof(cmd->payload.pid_params));
            }
            break;
            
        case CMD_OLED_CTRL:
            if (data) {
                memcpy(&cmd->payload.oled_ctrl, data, sizeof(cmd->payload.oled_ctrl));
            }
            break;
            
        case CMD_SET_ROBOT_CONFIG:
            if (data) {
                memcpy(&cmd->payload.config, data, sizeof(cmd->payload.config));
            }
            break;
            
        case CMD_ESP_NOW_SEND:
            if (data) {
                memcpy(&cmd->payload.esp_now, data, sizeof(cmd->payload.esp_now));
            }
            break;
            
        default:
            ESP_LOGW(TAG, "Unknown command type for creation: %d", type);
            return ESP_ERR_INVALID_ARG;
    }

    ESP_LOGI(TAG, "JSON command created: type=%d", type);
    return ESP_OK;
}

esp_err_t json_parser_parse_string(const char *json_string, json_command_t *command) {
    if (!json_parser_initialized || json_string == NULL || command == NULL) {
        return ESP_ERR_INVALID_STATE;
    }

    cJSON *root = cJSON_Parse(json_string);
    if (root == NULL) {
        ESP_LOGE(TAG, "Failed to parse JSON string");
        return ESP_FAIL;
    }

    // Parse command type
    cJSON *type_json = cJSON_GetObjectItem(root, "type");
    if (!cJSON_IsNumber(type_json)) {
        ESP_LOGE(TAG, "Missing or invalid command type");
        cJSON_Delete(root);
        return ESP_ERR_INVALID_ARG;
    }

    command->type = type_json->valueint;
    command->timestamp = xTaskGetTickCount() * portTICK_PERIOD_MS;

    // Parse command data based on type
    cJSON *data_json = cJSON_GetObjectItem(root, "data");
    if (data_json != NULL) {
        esp_err_t result = ESP_OK;
        
        switch (command->type) {
            case CMD_SPEED_CTRL:
            case CMD_PWM_INPUT:
            case CMD_ROS_CTRL:
            case CMD_SET_MOTOR_PID:
            case CMD_OLED_CTRL:
            case CMD_SET_ROBOT_CONFIG:
            case CMD_ESP_NOW_SEND:
                // Parse specific command data
                result = json_parser_parse_motion_command(data_json, command);
                break;
                
            default:
                ESP_LOGW(TAG, "Unknown command type for parsing: %d", command->type);
                result = ESP_ERR_INVALID_ARG;
                break;
        }
        
        if (result != ESP_OK) {
            cJSON_Delete(root);
            return result;
        }
    }

    cJSON_Delete(root);
    ESP_LOGI(TAG, "JSON string parsed successfully: type=%d", command->type);
    return ESP_OK;
}

// Private functions
static esp_err_t json_parser_parse_motion_command(const cJSON *cmd_data, json_command_t *command) {
    if (cmd_data == NULL || command == NULL) {
        return ESP_ERR_INVALID_ARG;
    }

    switch (command->type) {
        case CMD_SPEED_CTRL: {
            cJSON *left_speed = cJSON_GetObjectItem(cmd_data, "left_speed");
            cJSON *right_speed = cJSON_GetObjectItem(cmd_data, "right_speed");
            
            if (cJSON_IsNumber(left_speed) && cJSON_IsNumber(right_speed)) {
                command->payload.speed_ctrl.left_speed = left_speed->valuedouble;
                command->payload.speed_ctrl.right_speed = right_speed->valuedouble;
            }
            break;
        }
        
        case CMD_PWM_INPUT: {
            cJSON *left_pwm = cJSON_GetObjectItem(cmd_data, "left_pwm");
            cJSON *right_pwm = cJSON_GetObjectItem(cmd_data, "right_pwm");
            
            if (cJSON_IsNumber(left_pwm) && cJSON_IsNumber(right_pwm)) {
                command->payload.pwm_input.left_pwm = left_pwm->valueint;
                command->payload.pwm_input.right_pwm = right_pwm->valueint;
            }
            break;
        }
        
        case CMD_ROS_CTRL: {
            cJSON *linear_speed = cJSON_GetObjectItem(cmd_data, "linear_speed");
            cJSON *angular_speed = cJSON_GetObjectItem(cmd_data, "angular_speed");
            
            if (cJSON_IsNumber(linear_speed) && cJSON_IsNumber(angular_speed)) {
                command->payload.ros_ctrl.linear_speed = linear_speed->valuedouble;
                command->payload.ros_ctrl.angular_speed = angular_speed->valuedouble;
            }
            break;
        }
        
        case CMD_SET_MOTOR_PID: {
            cJSON *kp = cJSON_GetObjectItem(cmd_data, "kp");
            cJSON *ki = cJSON_GetObjectItem(cmd_data, "ki");
            cJSON *kd = cJSON_GetObjectItem(cmd_data, "kd");
            cJSON *limit = cJSON_GetObjectItem(cmd_data, "limit");
            
            if (cJSON_IsNumber(kp) && cJSON_IsNumber(ki) && cJSON_IsNumber(kd) && cJSON_IsNumber(limit)) {
                command->payload.pid_params.kp = kp->valuedouble;
                command->payload.pid_params.ki = ki->valuedouble;
                command->payload.pid_params.kd = kd->valuedouble;
                command->payload.pid_params.limit = limit->valuedouble;
            }
            break;
        }
        
        case CMD_OLED_CTRL: {
            cJSON *line_num = cJSON_GetObjectItem(cmd_data, "line_num");
            cJSON *text = cJSON_GetObjectItem(cmd_data, "text");
            
            if (cJSON_IsNumber(line_num) && cJSON_IsString(text)) {
                command->payload.oled_ctrl.line_num = line_num->valueint;
                strncpy(command->payload.oled_ctrl.text, text->valuestring, sizeof(command->payload.oled_ctrl.text) - 1);
            }
            break;
        }
        
        case CMD_SET_ROBOT_CONFIG: {
            cJSON *main_type = cJSON_GetObjectItem(cmd_data, "main_type");
            cJSON *module_type = cJSON_GetObjectItem(cmd_data, "module_type");
            
            if (cJSON_IsNumber(main_type) && cJSON_IsNumber(module_type)) {
                command->payload.config.main_type = main_type->valueint;
                command->payload.config.module_type = module_type->valueint;
            }
            break;
        }
        
        default:
            ESP_LOGW(TAG, "Unsupported command type for motion parsing: %d", command->type);
            return ESP_ERR_NOT_SUPPORTED;
    }

    return ESP_OK;
}

static esp_err_t json_parser_parse_servo_command(const cJSON *cmd_data, json_command_t *command) {
    if (cmd_data == NULL || command == NULL) {
        return ESP_ERR_INVALID_ARG;
    }
    
    switch (command->type) {
        case CMD_SERVO_CTRL: {
            cJSON *servo_id = cJSON_GetObjectItem(cmd_data, "servo_id");
            cJSON *position = cJSON_GetObjectItem(cmd_data, "position");
            cJSON *speed = cJSON_GetObjectItem(cmd_data, "speed");
            
            if (cJSON_IsNumber(servo_id) && cJSON_IsNumber(position)) {
                command->payload.servo_ctrl.servo_id = servo_id->valueint;
                command->payload.servo_ctrl.position = position->valueint;
                if (cJSON_IsNumber(speed)) {
                    command->payload.servo_ctrl.speed = speed->valueint;
                } else {
                    command->payload.servo_ctrl.speed = 100; // Default speed
                }
            } else {
                return ESP_ERR_INVALID_ARG;
            }
            break;
        }
        
        case CMD_ROBOTIC_ARM: {
            cJSON *base = cJSON_GetObjectItem(cmd_data, "base");
            cJSON *shoulder = cJSON_GetObjectItem(cmd_data, "shoulder");
            cJSON *elbow = cJSON_GetObjectItem(cmd_data, "elbow");
            cJSON *hand = cJSON_GetObjectItem(cmd_data, "hand");
            
            if (cJSON_IsNumber(base) && cJSON_IsNumber(shoulder) && 
                cJSON_IsNumber(elbow) && cJSON_IsNumber(hand)) {
                command->payload.robotic_arm.base = base->valuedouble;
                command->payload.robotic_arm.shoulder = shoulder->valuedouble;
                command->payload.robotic_arm.elbow = elbow->valuedouble;
                command->payload.robotic_arm.hand = hand->valuedouble;
            } else {
                return ESP_ERR_INVALID_ARG;
            }
            break;
        }
        
        default:
            ESP_LOGW(TAG, "Unsupported servo command type: %d", command->type);
            return ESP_ERR_NOT_SUPPORTED;
    }
    
    return ESP_OK;
}

static esp_err_t json_parser_parse_imu_command(const cJSON *cmd_data, json_command_t *command) {
    if (cmd_data == NULL || command == NULL) {
        return ESP_ERR_INVALID_ARG;
    }
    
    switch (command->type) {
        case CMD_IMU_CALIBRATE: {
            cJSON *calibration_type = cJSON_GetObjectItem(cmd_data, "calibration_type");
            if (cJSON_IsNumber(calibration_type)) {
                command->payload.imu_calibrate.calibration_type = calibration_type->valueint;
            } else {
                command->payload.imu_calibrate.calibration_type = 0; // Default: full calibration
            }
            break;
        }
        
        case CMD_IMU_CONFIG: {
            cJSON *gyro_range = cJSON_GetObjectItem(cmd_data, "gyro_range");
            cJSON *accel_range = cJSON_GetObjectItem(cmd_data, "accel_range");
            cJSON *sample_rate = cJSON_GetObjectItem(cmd_data, "sample_rate");
            
            if (cJSON_IsNumber(gyro_range)) {
                command->payload.imu_config.gyro_range = gyro_range->valueint;
            }
            if (cJSON_IsNumber(accel_range)) {
                command->payload.imu_config.accel_range = accel_range->valueint;
            }
            if (cJSON_IsNumber(sample_rate)) {
                command->payload.imu_config.sample_rate = sample_rate->valueint;
            }
            break;
        }
        
        default:
            ESP_LOGW(TAG, "Unsupported IMU command type: %d", command->type);
            return ESP_ERR_NOT_SUPPORTED;
    }
    
    return ESP_OK;
}

static esp_err_t json_parser_parse_system_command(const cJSON *cmd_data, json_command_t *command) {
    if (cmd_data == NULL || command == NULL) {
        return ESP_ERR_INVALID_ARG;
    }
    
    switch (command->type) {
        case CMD_SYSTEM_RESET: {
            cJSON *reset_type = cJSON_GetObjectItem(cmd_data, "reset_type");
            if (cJSON_IsNumber(reset_type)) {
                command->payload.system_reset.reset_type = reset_type->valueint;
            } else {
                command->payload.system_reset.reset_type = 0; // Default: soft reset
            }
            break;
        }
        
        case CMD_SYSTEM_SHUTDOWN: {
            cJSON *shutdown_type = cJSON_GetObjectItem(cmd_data, "shutdown_type");
            if (cJSON_IsNumber(shutdown_type)) {
                command->payload.system_shutdown.shutdown_type = shutdown_type->valueint;
            } else {
                command->payload.system_shutdown.shutdown_type = 0; // Default: normal shutdown
            }
            break;
        }
        
        case CMD_SYSTEM_STATUS: {
            // No additional data needed for status request
            break;
        }
        
        default:
            ESP_LOGW(TAG, "Unsupported system command type: %d", command->type);
            return ESP_ERR_NOT_SUPPORTED;
    }
    
    return ESP_OK;
}

static esp_err_t json_parser_create_feedback_json(const json_feedback_t *feedback, char *buffer, size_t buffer_size) {
    if (feedback == NULL || buffer == NULL || buffer_size == 0) {
        return ESP_ERR_INVALID_ARG;
    }
    
    cJSON *json = cJSON_CreateObject();
    if (json == NULL) {
        return ESP_ERR_NO_MEM;
    }
    
    // Add basic feedback info
    cJSON_AddStringToObject(json, "type", "feedback");
    cJSON_AddNumberToObject(json, "timestamp", feedback->timestamp);
    cJSON_AddStringToObject(json, "status", feedback->status);
    
    // Add motion feedback
    if (feedback->motion_available) {
        cJSON *motion = cJSON_CreateObject();
        cJSON_AddNumberToObject(motion, "left_speed", feedback->motion.left_speed);
        cJSON_AddNumberToObject(motion, "right_speed", feedback->motion.right_speed);
        cJSON_AddNumberToObject(motion, "left_encoder", feedback->motion.left_encoder);
        cJSON_AddNumberToObject(motion, "right_encoder", feedback->motion.right_encoder);
        cJSON_AddItemToObject(json, "motion", motion);
    }
    
    // Add IMU feedback
    if (feedback->imu_available) {
        cJSON *imu = cJSON_CreateObject();
        cJSON_AddNumberToObject(imu, "accel_x", feedback->imu.accel_x);
        cJSON_AddNumberToObject(imu, "accel_y", feedback->imu.accel_y);
        cJSON_AddNumberToObject(imu, "accel_z", feedback->imu.accel_z);
        cJSON_AddNumberToObject(imu, "gyro_x", feedback->imu.gyro_x);
        cJSON_AddNumberToObject(imu, "gyro_y", feedback->imu.gyro_y);
        cJSON_AddNumberToObject(imu, "gyro_z", feedback->imu.gyro_z);
        cJSON_AddNumberToObject(imu, "temperature", feedback->imu.temperature);
        cJSON_AddItemToObject(json, "imu", imu);
    }
    
    // Add battery feedback
    if (feedback->battery_available) {
        cJSON *battery = cJSON_CreateObject();
        cJSON_AddNumberToObject(battery, "voltage", feedback->battery.voltage);
        cJSON_AddNumberToObject(battery, "current", feedback->battery.current);
        cJSON_AddNumberToObject(battery, "power", feedback->battery.power);
        cJSON_AddNumberToObject(battery, "percentage", feedback->battery.percentage);
        cJSON_AddItemToObject(json, "battery", battery);
    }
    
    // Add system feedback
    cJSON *system = cJSON_CreateObject();
    cJSON_AddNumberToObject(system, "uptime", feedback->system.uptime);
    cJSON_AddNumberToObject(system, "free_heap", feedback->system.free_heap);
    cJSON_AddNumberToObject(system, "cpu_usage", feedback->system.cpu_usage);
    cJSON_AddItemToObject(json, "system", system);
    
    // Convert to string
    char *json_string = cJSON_Print(json);
    if (json_string == NULL) {
        cJSON_Delete(json);
        return ESP_ERR_NO_MEM;
    }
    
    // Copy to buffer
    size_t len = strlen(json_string);
    if (len >= buffer_size) {
        free(json_string);
        cJSON_Delete(json);
        return ESP_ERR_INVALID_SIZE;
    }
    
    strcpy(buffer, json_string);
    free(json_string);
    cJSON_Delete(json);
    
    return ESP_OK;
}
