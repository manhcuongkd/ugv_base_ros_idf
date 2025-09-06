#include "../inc/mission_system.h"
#include "../inc/files_controller.h"
#include "../inc/uart_controller.h"
#include <esp_log.h>
#include <string.h>
#include <stdlib.h>
#include <freertos/FreeRTOS.h>
#include <freertos/task.h>

static const char *TAG = "Mission_System";

// Mission state variables
static bool mission_playing = false;
static bool mission_abort_requested = false;
static char current_mission_name[64] = {0};
static int current_repeat_times = 0;
static int current_step = 0;

// Mission system implementation
bool create_mission(const char *name, const char *intro) {
    if (!name || !intro) {
        ESP_LOGE(TAG, "Invalid parameters for mission creation");
        return false;
    }
    
    ESP_LOGI(TAG, "Creating mission: %s", name);
    
    // Create mission file with introduction
    esp_err_t ret = files_controller_create_file(name, intro);
    if (ret != ESP_OK) {
        ESP_LOGE(TAG, "Failed to create mission file: %s", esp_err_to_name(ret));
        return false;
    }
    
    ESP_LOGI(TAG, "Mission created successfully: %s", name);
    return true;
}

int mission_content(const char *name) {
    if (!name) {
        ESP_LOGE(TAG, "Invalid mission name");
        return -1;
    }
    
    ESP_LOGI(TAG, "Getting mission content: %s", name);
    
    // Read mission file content
    char content[1024];
    esp_err_t ret = files_controller_read_file(name, content, sizeof(content));
    if (ret != ESP_OK) {
        ESP_LOGE(TAG, "Failed to read mission content: %s", esp_err_to_name(ret));
        return -1;
    }
    
    ESP_LOGI(TAG, "Mission content: %s", content);
    return strlen(content);
}

bool append_step_json(const char *name, const char *step) {
    if (!name || !step) {
        ESP_LOGE(TAG, "Invalid parameters for step appending");
        return false;
    }
    
    ESP_LOGI(TAG, "Appending step JSON to mission: %s", name);
    
    // Append step to mission file
    esp_err_t ret = files_controller_append_line(name, step);
    if (ret != ESP_OK) {
        ESP_LOGE(TAG, "Failed to append step: %s", esp_err_to_name(ret));
        return false;
    }
    
    ESP_LOGI(TAG, "Step appended successfully");
    return true;
}

void append_step_feedback(const char *name, float speed) {
    if (!name) {
        ESP_LOGE(TAG, "Invalid mission name");
        return;
    }
    
    ESP_LOGI(TAG, "Appending step feedback to mission: %s, speed: %.2f", name, speed);
    
    // Create feedback step command
    char step[64];
    snprintf(step, sizeof(step), "{\"T\":1,\"L\":%.2f,\"R\":%.2f}", speed, speed);
    
    append_step_json(name, step);
}

void append_delay_cmd(const char *name, int delay_time) {
    if (!name) {
        ESP_LOGE(TAG, "Invalid mission name");
        return;
    }
    
    ESP_LOGI(TAG, "Appending delay to mission: %s, delay: %d", name, delay_time);
    
    // Create delay step command
    char step[64];
    snprintf(step, sizeof(step), "{\"T\":999,\"delay\":%d}", delay_time);
    
    append_step_json(name, step);
}

bool insert_step_json(const char *name, int step_num, const char *step) {
    if (!name || !step) {
        ESP_LOGE(TAG, "Invalid parameters for step insertion");
        return false;
    }
    
    ESP_LOGI(TAG, "Inserting step JSON to mission: %s, step: %d", name, step_num);
    
    // Insert step at specified line number
    esp_err_t ret = files_controller_insert_line(name, step_num, step);
    if (ret != ESP_OK) {
        ESP_LOGE(TAG, "Failed to insert step: %s", esp_err_to_name(ret));
        return false;
    }
    
    ESP_LOGI(TAG, "Step inserted successfully");
    return true;
}

void insert_step_feedback(const char *name, int step_num, float speed) {
    if (!name) {
        ESP_LOGE(TAG, "Invalid mission name");
        return;
    }
    
    ESP_LOGI(TAG, "Inserting step feedback to mission: %s, step: %d, speed: %.2f", name, step_num, speed);
    
    // Create feedback step command
    char step[64];
    snprintf(step, sizeof(step), "{\"T\":1,\"L\":%.2f,\"R\":%.2f}", speed, speed);
    
    insert_step_json(name, step_num, step);
}

void insert_delay_cmd(const char *name, int step_num, int delay_time) {
    if (!name) {
        ESP_LOGE(TAG, "Invalid mission name");
        return;
    }
    
    ESP_LOGI(TAG, "Inserting delay to mission: %s, step: %d, delay: %d", name, step_num, delay_time);
    
    // Create delay step command
    char step[64];
    snprintf(step, sizeof(step), "{\"T\":999,\"delay\":%d}", delay_time);
    
    insert_step_json(name, step_num, step);
}

bool replace_step_json(const char *name, int step_num, const char *step) {
    if (!name || !step) {
        ESP_LOGE(TAG, "Invalid parameters for step replacement");
        return false;
    }
    
    ESP_LOGI(TAG, "Replacing step JSON in mission: %s, step: %d", name, step_num);
    
    // Replace step at specified line number
    esp_err_t ret = files_controller_replace_line(name, step_num, step);
    if (ret != ESP_OK) {
        ESP_LOGE(TAG, "Failed to replace step: %s", esp_err_to_name(ret));
        return false;
    }
    
    ESP_LOGI(TAG, "Step replaced successfully");
    return true;
}

void replace_step_feedback(const char *name, int step_num, float speed) {
    if (!name) {
        ESP_LOGE(TAG, "Invalid mission name");
        return;
    }
    
    ESP_LOGI(TAG, "Replacing step feedback in mission: %s, step: %d, speed: %.2f", name, step_num, speed);
    
    // Create feedback step command
    char step[64];
    snprintf(step, sizeof(step), "{\"T\":1,\"L\":%.2f,\"R\":%.2f}", speed, speed);
    
    replace_step_json(name, step_num, step);
}

void replace_delay_cmd(const char *name, int step_num, int delay_time) {
    if (!name) {
        ESP_LOGE(TAG, "Invalid mission name");
        return;
    }
    
    ESP_LOGI(TAG, "Replacing delay in mission: %s, step: %d, delay: %d", name, step_num, delay_time);
    
    // Create delay step command
    char step[64];
    snprintf(step, sizeof(step), "{\"T\":999,\"delay\":%d}", delay_time);
    
    replace_step_json(name, step_num, step);
}

void delete_step(const char *name, int step_num) {
    if (!name) {
        ESP_LOGE(TAG, "Invalid mission name");
        return;
    }
    
    ESP_LOGI(TAG, "Deleting step from mission: %s, step: %d", name, step_num);
    
    // Delete step at specified line number
    esp_err_t ret = files_controller_delete_line(name, step_num);
    if (ret != ESP_OK) {
        ESP_LOGE(TAG, "Failed to delete step: %s", esp_err_to_name(ret));
        return;
    }
    
    ESP_LOGI(TAG, "Step deleted successfully");
}

bool move_to_step(const char *name, int step_num) {
    if (!name) {
        ESP_LOGE(TAG, "Invalid mission name");
        return false;
    }
    
    ESP_LOGI(TAG, "Moving to step in mission: %s, step: %d", name, step_num);
    
    // Read the specific step from mission file
    char step_content[256];
    esp_err_t ret = files_controller_read_line(name, step_num, step_content, sizeof(step_content));
    if (ret != ESP_OK) {
        ESP_LOGE(TAG, "Failed to read step: %s", esp_err_to_name(ret));
        return false;
    }
    
    ESP_LOGI(TAG, "Step content: %s", step_content);
    
    // Execute the step command by parsing and sending it to UART controller
    ret = uart_controller_parse_command(step_content, strlen(step_content));
    if (ret != ESP_OK) {
        ESP_LOGE(TAG, "Failed to execute step command: %s", esp_err_to_name(ret));
        return false;
    }
    
    ESP_LOGI(TAG, "Step executed successfully");
    return true;
}

void mission_play(const char *name, int repeat_times) {
    if (!name) {
        ESP_LOGE(TAG, "Invalid mission name");
        return;
    }
    
    if (mission_playing) {
        ESP_LOGW(TAG, "Mission already playing, aborting current mission");
        serial_mission_abort();
    }
    
    ESP_LOGI(TAG, "Playing mission: %s, times: %d", name, repeat_times);
    
    // Set mission state
    mission_playing = true;
    mission_abort_requested = false;
    strncpy(current_mission_name, name, sizeof(current_mission_name) - 1);
    current_mission_name[sizeof(current_mission_name) - 1] = '\0';
    current_repeat_times = repeat_times;
    current_step = 1;
    
    // Start mission playback task
    xTaskCreate(mission_playback_task, "mission_playback", 4096, NULL, 5, NULL);
}

bool serial_mission_abort(void) {
    ESP_LOGI(TAG, "Mission abort requested");
    
    // Set abort flag
    mission_abort_requested = true;
    
    // Reset mission state
    mission_playing = false;
    current_mission_name[0] = '\0';
    current_repeat_times = 0;
    current_step = 0;
    
    ESP_LOGI(TAG, "Mission aborted successfully");
    return true;
}

// Mission playback task
void mission_playback_task(void *pvParameters) {
    int repeat_count = 0;
    
    while (repeat_count < current_repeat_times && !mission_abort_requested) {
        ESP_LOGI(TAG, "Starting mission repeat %d/%d", repeat_count + 1, current_repeat_times);
        
        // Read and execute all steps in the mission
        int step_num = 1;
        char step_content[256];
        
        while (!mission_abort_requested) {
            esp_err_t ret = files_controller_read_line(current_mission_name, step_num, step_content, sizeof(step_content));
            if (ret != ESP_OK) {
                // No more steps or error reading step
                break;
            }
            
            ESP_LOGI(TAG, "Executing step %d: %s", step_num, step_content);
            
            // Execute the step
            ret = uart_controller_parse_command(step_content, strlen(step_content));
            if (ret != ESP_OK) {
                ESP_LOGE(TAG, "Failed to execute step %d: %s", step_num, esp_err_to_name(ret));
                break;
            }
            
            // Wait between steps (configurable delay)
            vTaskDelay(pdMS_TO_TICKS(100)); // 100ms delay between steps
            
            step_num++;
        }
        
        repeat_count++;
        
        // Wait between repeats if not the last one
        if (repeat_count < current_repeat_times && !mission_abort_requested) {
            vTaskDelay(pdMS_TO_TICKS(1000)); // 1 second delay between repeats
        }
    }
    
    // Mission completed or aborted
    mission_playing = false;
    ESP_LOGI(TAG, "Mission playback task completed");
    
    vTaskDelete(NULL);
}