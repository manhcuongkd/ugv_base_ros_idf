#include <stdio.h>
#include <string.h>
#include <math.h>
#include <esp_log.h>
#include <esp_timer.h>
#include <cJSON.h>
#include <freertos/FreeRTOS.h>
#include <freertos/task.h>

#include "../inc/ugv_advanced.h"
#include "../inc/ugv_config.h"
#include "../inc/oled_controller.h"
#include "../inc/led_controller.h"
#include "../inc/mission_system.h"
#include "../inc/files_controller.h"
#include "../inc/json_parser.h"
#include "../inc/uart_controller.h"
#include "../inc/motion_module.h"
#include "../inc/gimbal_controller.h"
#include "../inc/servo_controller.h"
#include "../inc/imu_controller.h"
#include "../inc/battery_controller.h"

static const char *TAG = "UGV_Advanced";

// Global variables for mission system
static cJSON *json_cmd_receive = NULL;
// static bool mission_abort_flag = false;  // Used by mission_system.cpp
static uint32_t feedback_flow_extra_delay = 50; // ms
static bool uart_cmd_echo = true;

// Arm configuration variables (matching Arduino)
static double l3A, l3B, l3, t3rad;
static double l4A, l4B, lEA, lEB, lE, tErad;
static double EoAT_A, EoAT_B;
static double initX, initY, initZ, initT;
static double goalX, goalY, goalZ, goalT;
static double lastX, lastY, lastZ, lastT;
static double radB, radS, radE;

// Speed rates
static float spd_rate_A = 1.0f;
static float spd_rate_B = 1.0f;

// Module type
static uint8_t module_type = 0;

// ============================================================================
// MISSION MANAGEMENT FUNCTIONS (Delegated to mission_system.cpp)
// ============================================================================
// Note: Mission management functions are implemented in mission_system.cpp
// to avoid duplicate definitions. These functions are declared in ugv_advanced.h
// for Arduino compatibility but implemented elsewhere.

// ============================================================================
// JSON COMMAND PROCESSING
// ============================================================================

/**
 * @brief Process received JSON command
 */
void json_cmd_receive_handler(void) {
    if (!json_cmd_receive) {
        ESP_LOGE(TAG, "No command to process");
        return;
    }
    
    // Get command type
    cJSON *cmd_type = cJSON_GetObjectItem(json_cmd_receive, "T");
    if (!cJSON_IsNumber(cmd_type)) {
        ESP_LOGE(TAG, "Invalid command type");
        return;
    }
    
    int type = cmd_type->valueint;
    ESP_LOGI(TAG, "Processing command type: %d", type);
    
    switch (type) {
        case CMD_ARM_POSITION: {
            // Arm position command
            cJSON *x = cJSON_GetObjectItem(json_cmd_receive, "x");
            cJSON *y = cJSON_GetObjectItem(json_cmd_receive, "y");
            cJSON *z = cJSON_GetObjectItem(json_cmd_receive, "z");
            cJSON *t = cJSON_GetObjectItem(json_cmd_receive, "t");
            cJSON *spd = cJSON_GetObjectItem(json_cmd_receive, "spd");
            
            if (cJSON_IsNumber(x) && cJSON_IsNumber(y) && cJSON_IsNumber(z) && cJSON_IsNumber(t)) {
                double pos_x = x->valuedouble;
                double pos_y = y->valuedouble;
                double pos_z = z->valuedouble;
                double pos_t = t->valuedouble;
                float speed = cJSON_IsNumber(spd) ? spd->valuedouble : 1.0f;
                
                ESP_LOGI(TAG, "Moving arm to: x=%.2f, y=%.2f, z=%.2f, t=%.2f, speed=%.2f",
                         pos_x, pos_y, pos_z, pos_t, speed);
                
                // Update last position
                lastX = pos_x;
                lastY = pos_y;
                lastZ = pos_z;
                lastT = pos_t;
                
                // In real implementation, this would control the arm
                // servo_controller_move_arm(pos_x, pos_y, pos_z, pos_t, speed);
            }
            break;
        }
        
        case CMD_DELAY: {
            // Delay command
            cJSON *delay = cJSON_GetObjectItem(json_cmd_receive, "cmd");
            if (cJSON_IsNumber(delay)) {
                int delay_ms = delay->valueint;
                ESP_LOGI(TAG, "Delay: %d ms", delay_ms);
                vTaskDelay(pdMS_TO_TICKS(delay_ms));
            }
            break;
        }
        
        case CMD_SET_SPD_RATE: {
            // Speed rate command
            cJSON *rate_a = cJSON_GetObjectItem(json_cmd_receive, "L");
            cJSON *rate_b = cJSON_GetObjectItem(json_cmd_receive, "R");
            
            if (cJSON_IsNumber(rate_a) && cJSON_IsNumber(rate_b)) {
                spd_rate_A = rate_a->valuedouble;
                spd_rate_B = rate_b->valuedouble;
                ESP_LOGI(TAG, "Speed rates set: A=%.2f, B=%.2f", spd_rate_A, spd_rate_B);
            }
            break;
        }
        
        case CMD_MM_TYPE_SET: {
            // Module type command
            cJSON *main_type = cJSON_GetObjectItem(json_cmd_receive, "main");
            cJSON *mod_type = cJSON_GetObjectItem(json_cmd_receive, "module");
            
            if (cJSON_IsNumber(main_type) && cJSON_IsNumber(mod_type)) {
                uint8_t main = main_type->valueint;
                uint8_t module = mod_type->valueint;
                ESP_LOGI(TAG, "Module type set: main=%d, module=%d", main, module);
                
                module_type = module;
                // In real implementation, this would configure the system
            }
            break;
        }
        
        default:
            ESP_LOGW(TAG, "Unknown command type: %d", type);
            break;
    }
}

// ============================================================================
// CONFIGURATION FUNCTIONS
// ============================================================================

void config_ee_mode_type(uint8_t mode) {
    ESP_LOGI(TAG, "Configuring end effector mode type: %d", mode);
    
    // Initialize arm dimensions based on mode
    if (mode == 0) {
        l3A = 100.0; // ARM_L3_LENGTH_MM_A_0
        l3B = 50.0;  // ARM_L3_LENGTH_MM_B_0
        l3 = sqrt(l3A * l3A + l3B * l3B);
        t3rad = atan2(l3B, l3A);
        
        initX = l3A + 100.0; // l2B
        initY = 0;
        initZ = 100.0 - l3B; // l2A - l3B
        initT = M_PI;
    } else if (mode == 1) {
        l3A = 120.0; // ARM_L3_LENGTH_MM_A_1
        l3B = 60.0;  // ARM_L3_LENGTH_MM_B_1
        l3 = sqrt(l3A * l3A + l3B * l3B);
        t3rad = atan2(l3B, l3A);
        
        EoAT_A = 20.0;
        EoAT_B = 10.0;
        l4A = 67.85; // ARM_L4_LENGTH_MM_A
        l4B = 30.0;  // ARM_L4_LENGTH_MM_B
        lEA = EoAT_A + l4A;
        lEB = EoAT_B + l4B;
        lE = sqrt(lEA * lEA + lEB * lEB);
        tErad = atan2(lEB, lEA);
        
        initX = l3A + 100.0 + l4A + EoAT_A;
        initY = 0;
        initZ = 100.0 - l3B - l4B - EoAT_B;
        initT = M_PI;
    }
    
    goalX = initX;
    goalY = initY;
    goalZ = initZ;
    goalT = initT;
    
    lastX = goalX;
    lastY = goalY;
    lastZ = goalZ;
    lastT = goalT;
    
    ugv_config.eem_mode = mode;
}

void config_eoat(uint8_t mount_pos, double ea, double eb) {
    ESP_LOGI(TAG, "Configuring EoAT: mount_pos=%d, ea=%.2f, eb=%.2f", mount_pos, ea, eb);
    
    // Set mount position specific values
    switch (mount_pos) {
        case 0: l4A = 67.85; break;
        case 1: l4A = 64.16; break;
        case 2: l4A = 59.07; break;
        case 3: l4A = 51.07; break;
        default: l4A = 67.85; break;
    }
    
    EoAT_A = ea;
    EoAT_B = eb;
    
    l4B = 30.0; // ARM_L4_LENGTH_MM_B
    lEA = EoAT_A + l4A;
    lEB = EoAT_B + l4B;
    lE = sqrt(lEA * lEA + lEB * lEB);
    tErad = atan2(lEB, lEA);
    
    initX = l3A + 100.0 + l4A + EoAT_A;
    initY = 0;
    initZ = 100.0 - l3B - l4B - EoAT_B;
    initT = M_PI;
}

void config_info_print(uint8_t cmd) {
    ESP_LOGI(TAG, "Configuring info print level: %d", cmd);
    ugv_config.info_print = cmd;
}

void set_base_info_feedback_mode(bool enable) {
    ESP_LOGI(TAG, "Setting base info feedback mode: %s", enable ? "enabled" : "disabled");
    ugv_config.base_feedback_flow = enable;
}

void set_feedback_flow_interval(int cmd) {
    ESP_LOGI(TAG, "Setting feedback flow interval: %d", cmd);
    feedback_flow_extra_delay = abs(cmd);
}

void set_cmd_echo(bool enable) {
    ESP_LOGI(TAG, "Setting command echo mode: %s", enable ? "enabled" : "disabled");
    uart_cmd_echo = enable;
}

void change_module_type(uint8_t cmd) {
    ESP_LOGI(TAG, "Changing module type: %d", cmd);
    module_type = cmd;
}

// ============================================================================
// SAVE FUNCTIONS
// ============================================================================

void save_speed_rate(void) {
    ESP_LOGI(TAG, "Saving speed rate to boot mission");
    
    cJSON *speed_json = cJSON_CreateObject();
    cJSON_AddNumberToObject(speed_json, "T", CMD_SET_SPD_RATE);
    cJSON_AddNumberToObject(speed_json, "L", spd_rate_A);
    cJSON_AddNumberToObject(speed_json, "R", spd_rate_B);
    
    char *json_string = cJSON_Print(speed_json);
    if (json_string) {
        append_step_json("boot", json_string);
        free(json_string);
    }
    
    cJSON_Delete(speed_json);
}

void save_main_type_module_type(uint8_t main_type, uint8_t module_type) {
    ESP_LOGI(TAG, "Saving main type %d and module type %d to boot mission", main_type, module_type);
    
    // Check if same configuration already exists
    char filename[UGV_MAX_MISSION_NAME_LEN + 20];
    snprintf(filename, sizeof(filename), "boot%s", MISSION_FILE_EXTENSION);
    
    char content[2048];
    esp_err_t ret = files_controller_read_file(filename, content, sizeof(content));
    if (ret == ESP_OK) {
        // Parse and check for existing CMD_MM_TYPE_SET
        cJSON *mission_json = cJSON_Parse(content);
        if (mission_json) {
            // Check if same configuration exists
            // Simplified check - in real implementation would parse all steps
            cJSON_Delete(mission_json);
        }
    }
    
    // Create new configuration
    cJSON *config_json = cJSON_CreateObject();
    cJSON_AddNumberToObject(config_json, "T", CMD_MM_TYPE_SET);
    cJSON_AddNumberToObject(config_json, "main", main_type);
    cJSON_AddNumberToObject(config_json, "module", module_type);
    
    char *json_string = cJSON_Print(config_json);
    if (json_string) {
        append_step_json("boot", json_string);
        free(json_string);
    }
    
    cJSON_Delete(config_json);
}

// ============================================================================
// SYSTEM FUNCTIONS
// ============================================================================

void base_info_feedback(void) {
    static uint32_t last_feedback_time = 0;
    uint32_t current_time = esp_timer_get_time() / 1000;
    
    // Check feedback interval
    if (current_time - last_feedback_time < feedback_flow_extra_delay) {
        return;
    }
    
    last_feedback_time = current_time;
    
    if (!ugv_config.base_feedback_flow) {
        return;
    }
    
    ESP_LOGI(TAG, "Sending base info feedback");
    
    // Create feedback JSON
    cJSON *feedback = cJSON_CreateObject();
    cJSON_AddNumberToObject(feedback, "T", FEEDBACK_BASE_INFO);
    
    // Motion feedback
    cJSON_AddNumberToObject(feedback, "L", 0.0); // Left speed
    cJSON_AddNumberToObject(feedback, "R", 0.0); // Right speed
    
    // IMU feedback (simplified)
    cJSON_AddNumberToObject(feedback, "ax", 0.0);
    cJSON_AddNumberToObject(feedback, "ay", 0.0);
    cJSON_AddNumberToObject(feedback, "az", 9.8);
    cJSON_AddNumberToObject(feedback, "gx", 0.0);
    cJSON_AddNumberToObject(feedback, "gy", 0.0);
    cJSON_AddNumberToObject(feedback, "gz", 0.0);
    cJSON_AddNumberToObject(feedback, "mx", 0.0);
    cJSON_AddNumberToObject(feedback, "my", 0.0);
    cJSON_AddNumberToObject(feedback, "mz", 0.0);
    
    // Odometry (simplified)
    cJSON_AddNumberToObject(feedback, "odl", 0);
    cJSON_AddNumberToObject(feedback, "odr", 0);
    
    // Voltage (simplified)
    cJSON_AddNumberToObject(feedback, "v", 1100); // 11.0V
    
    // Module-specific feedback
    switch (module_type) {
        case 1: // Arm module
            cJSON_AddNumberToObject(feedback, "ax", lastX);
            cJSON_AddNumberToObject(feedback, "ay", lastY);
            cJSON_AddNumberToObject(feedback, "az", lastZ);
            cJSON_AddNumberToObject(feedback, "ab", radB);
            cJSON_AddNumberToObject(feedback, "as", radS);
            cJSON_AddNumberToObject(feedback, "ae", radE);
            cJSON_AddNumberToObject(feedback, "at", lastT);
            cJSON_AddNumberToObject(feedback, "torB", 0);
            cJSON_AddNumberToObject(feedback, "torS", 0);
            cJSON_AddNumberToObject(feedback, "torE", 0);
            cJSON_AddNumberToObject(feedback, "torH", 0);
            break;
            
        case 2: // Gimbal module
            cJSON_AddNumberToObject(feedback, "pan", 0.0);
            cJSON_AddNumberToObject(feedback, "tilt", 0.0);
            break;
    }
    
    char *json_string = cJSON_Print(feedback);
    if (json_string) {
        ESP_LOGI(TAG, "Feedback: %s", json_string);
        free(json_string);
    }
    
    cJSON_Delete(feedback);
}

void heart_beat_ctrl(void) {
    static uint32_t last_heartbeat = 0;
    uint32_t current_time = esp_timer_get_time() / 1000;
    
    // Heartbeat every 3 seconds
    if (current_time - last_heartbeat >= 3000) {
        ESP_LOGI(TAG, "Heartbeat - System running");
        last_heartbeat = current_time;
        
        // Control LED heartbeat
        led_controller_heartbeat();
    }
}