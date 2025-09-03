#include <stdio.h>
#include <string.h>
#include <esp_log.h>
#include <esp_timer.h>
#include <cJSON.h>

#include "../inc/ugv_advanced.h"
#include "../inc/ugv_config.h"
#include "../inc/oled_controller.h"
#include "../inc/led_controller.h"
#include "../inc/mission_system.h"

static const char *TAG = "UGV_Advanced";

// ============================================================================
// ADVANCED UGV FUNCTIONS (Matching Arduino Implementation)
// ============================================================================

void config_ee_mode_type(uint8_t mode)
{
    ESP_LOGI(TAG, "Configuring end effector mode type: %d", mode);
    // Implementation would configure arm dimensions based on mode
    ugv_config.eem_mode = mode;
}

void config_eoat(uint8_t mount_pos, double ea, double eb)
{
    ESP_LOGI(TAG, "Configuring EoAT: mount_pos=%d, ea=%.2f, eb=%.2f", mount_pos, ea, eb);
    // Implementation would configure end effector dimensions
}

void config_info_print(uint8_t cmd)
{
    ESP_LOGI(TAG, "Configuring info print level: %d", cmd);
    ugv_config.info_print = cmd;
}

void set_base_info_feedback_mode(bool enable)
{
    ESP_LOGI(TAG, "Setting base info feedback mode: %s", enable ? "enabled" : "disabled");
    ugv_config.base_feedback_flow = enable;
}

void set_feedback_flow_interval(int cmd)
{
    ESP_LOGI(TAG, "Setting feedback flow interval: %d", cmd);
    // Implementation would set feedback timing
}

void set_cmd_echo(bool enable)
{
    ESP_LOGI(TAG, "Setting command echo mode: %s", enable ? "enabled" : "disabled");
    // Implementation would set UART echo mode
}

void save_speed_rate(void)
{
    ESP_LOGI(TAG, "Saving speed rate to boot mission");
    
    char speed_json[256];
    snprintf(speed_json, sizeof(speed_json), "{\"T\":138,\"L\":%.2f,\"R\":%.2f}", 
             1.0f, 1.0f); // Placeholder values
    
    append_step_json("boot", speed_json);
}

void save_main_type_module_type(uint8_t main_type, uint8_t module_type)
{
    ESP_LOGI(TAG, "Saving main type %d and module type %d to boot mission", main_type, module_type);
    
    char config_json[256];
    snprintf(config_json, sizeof(config_json), "{\"T\":900,\"main\":%d,\"module\":%d}", 
             main_type, module_type);
    
    append_step_json("boot", config_json);
}

void base_info_feedback(void)
{
    static uint32_t last_feedback_time = 0;
    uint32_t current_time = esp_timer_get_time() / 1000;
    
    // Check feedback interval
    if (current_time - last_feedback_time < 100) { // 100ms interval
        return;
    }
    
    last_feedback_time = current_time;
    
    ESP_LOGI(TAG, "Sending base info feedback");
    
    // Create feedback JSON (simplified)
    cJSON *feedback = cJSON_CreateObject();
    cJSON_AddNumberToObject(feedback, "T", 1001); // FEEDBACK_BASE_INFO
    cJSON_AddNumberToObject(feedback, "L", 0);     // Left speed
    cJSON_AddNumberToObject(feedback, "R", 0);     // Right speed
    cJSON_AddNumberToObject(feedback, "v", 1100);  // Voltage (11.0V)
    
    char *json_str = cJSON_Print(feedback);
    if (json_str) {
        ESP_LOGI(TAG, "Feedback: %s", json_str);
        free(json_str);
    }
    
    cJSON_Delete(feedback);
}

void heart_beat_ctrl(void)
{
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
