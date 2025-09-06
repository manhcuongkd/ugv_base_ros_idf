#include <stdio.h>
#include <string.h>
#include <freertos/FreeRTOS.h>
#include <freertos/task.h>
#include <freertos/queue.h>
#include <esp_system.h>
#include <nvs_flash.h>
#include <esp_log.h>
#include <driver/uart.h>
#include <driver/gpio.h>
#include <esp_wifi.h>
#include <esp_event.h>
#include <esp_http_server.h>
#include <esp_spiffs.h>
#include <esp_timer.h>
#include <cJSON.h>

// Include our custom headers
#include "../inc/ugv_config.h"
#include "../inc/json_parser.h"
#include "../inc/oled_controller.h"
#include "../inc/imu_controller.h"
#include "../inc/motion_module.h"
#include "../inc/mission_system.h"
#include "../inc/ugv_advanced.h"
#include "../inc/module_handlers.h"
#include "../inc/system_manager.h"

static const char *TAG = "Main";

// Global variables
static QueueHandle_t json_cmd_queue;
static TaskHandle_t main_task_handle;
static TaskHandle_t imu_task_handle;
static TaskHandle_t motion_task_handle;
static TaskHandle_t uart_task_handle;

// Configuration
ugv_config_t ugv_config = {
    .main_type = 1,        // RaspRover
    .module_type = 2,      // Gimbal
    .info_print = 1,       // Debug info
    .esp_now_mode = 3,     // Follower mode
    .ctrl_by_broadcast = true,
    .steady_mode = false,
    .base_feedback_flow = true,
    .eem_mode = 0          // Default EEM mode
};

// Task function declarations
static void main_task(void *pvParameters);
static void imu_task(void *pvParameters);
static void motion_task(void *pvParameters);
static void uart_task(void *pvParameters);

extern "C" void app_main(void)
{
    ESP_LOGI(TAG, "Starting RaspRover IDF Application");
    
    // Initialize NVS
    esp_err_t ret = nvs_flash_init();
    if (ret == ESP_ERR_NVS_NO_FREE_PAGES || ret == ESP_ERR_NVS_NEW_VERSION_FOUND) {
        ESP_ERROR_CHECK(nvs_flash_erase());
        ret = nvs_flash_init();
    }
    ESP_ERROR_CHECK(ret);
    
    // Initialize system
    ESP_ERROR_CHECK(system_manager_init());
    
    // Create queues
    json_cmd_queue = xQueueCreate(10, sizeof(json_command_t));
    if (json_cmd_queue == NULL) {
        ESP_LOGE(TAG, "Failed to create JSON command queue");
        return;
    }
    
    // Create tasks
    xTaskCreatePinnedToCore(main_task, "main_task", 8192, NULL, 5, &main_task_handle, 0);
    xTaskCreatePinnedToCore(imu_task, "imu_task", 4096, NULL, 4, &imu_task_handle, 1);
    xTaskCreatePinnedToCore(motion_task, "motion_task", 4096, NULL, 3, &motion_task_handle, 0);
    xTaskCreatePinnedToCore(uart_task, "uart_task", 4096, NULL, 4, &uart_task_handle, 0);
    
    ESP_LOGI(TAG, "RaspRover application started successfully");
}

// Main application task
static void main_task(void *pvParameters)
{
    ESP_LOGI(TAG, "Main task started");
    
    // Wait for system initialization
    while (!system_manager_is_initialized()) {
        vTaskDelay(pdMS_TO_TICKS(100));
    }
    
    // Display startup message
    oled_controller_display_text(0, "RaspRover IDF");
    oled_controller_display_text(1, "Version: 1.0.0");
    oled_controller_display_text(2, "Starting...");
    oled_controller_display_text(3, "");
    oled_controller_update();
    
    vTaskDelay(pdMS_TO_TICKS(1200));
    
    // Main application loop
    while (1) {
        // Process JSON commands
        json_command_t cmd;
        if (xQueueReceive(json_cmd_queue, &cmd, pdMS_TO_TICKS(10)) == pdTRUE) {
            json_parser_process_command(&cmd);
        }
        
        // Update system status
        if (ugv_config.base_feedback_flow) {
            // Send base feedback
            base_info_feedback();
        }
        
        // Module type specific handling (matching Arduino implementation)
        switch (ugv_config.module_type) {
            case 1: // RoArm-M2
                module_type_roarm_m2();
                break;
            case 2: // Gimbal
                module_type_gimbal();
                break;
        }
        
        // Heartbeat control
        heart_beat_ctrl();
        
        // Update OLED display
        oled_controller_update_system_info();
        
        vTaskDelay(pdMS_TO_TICKS(50)); // 20Hz update rate
    }
}

// IMU task
static void imu_task(void *pvParameters)
{
    ESP_LOGI(TAG, "IMU task started");
    
    while (!system_manager_is_initialized()) {
        vTaskDelay(pdMS_TO_TICKS(100));
    }
    
    // IMU calibration
    ESP_LOGI(TAG, "Starting IMU calibration...");
    oled_controller_display_text(3, "IMU Calibrating...");
    oled_controller_update();
    
    imu_controller_calibrate();
    
    oled_controller_display_text(3, "IMU Ready");
    oled_controller_update();
    
    // IMU data processing loop
    while (1) {
        imu_data_t imu_data;
        if (imu_controller_read_data(&imu_data) == ESP_OK) {
            // Process IMU data
            if (ugv_config.module_type == 2) { // Gimbal mode
                // Apply gimbal stabilization
                // gimbal_controller_stabilize(&imu_data);
            }
        }
        
        vTaskDelay(pdMS_TO_TICKS(10)); // 100Hz IMU update rate
    }
}

// Motion task
static void motion_task(void *pvParameters)
{
    ESP_LOGI(TAG, "Motion task started");
    
    while (!system_manager_is_initialized()) {
        vTaskDelay(pdMS_TO_TICKS(100));
    }
    
    // Initialize encoders
    motion_module_init_encoders();
    
    // Initialize PID controllers
    motion_module_init_pid();
    
    // Motion control loop
    while (1) {
        if (!system_manager_is_emergency_stop()) {
            // Update encoder readings
            motion_module_update_encoders();
            
            // Compute PID control
            motion_module_compute_pid();
            
            // Apply motor control
            motion_module_apply_motor_control();
        }
        
        vTaskDelay(pdMS_TO_TICKS(20)); // 50Hz motion control rate
    }
}

// UART task
static void uart_task(void *pvParameters)
{
    ESP_LOGI(TAG, "UART task started");
    
    // UART configuration
    const uart_config_t uart_config = {
        .baud_rate = 115200,
        .data_bits = UART_DATA_8_BITS,
        .parity = UART_PARITY_DISABLE,
        .stop_bits = UART_STOP_BITS_1,
        .flow_ctrl = UART_HW_FLOWCTRL_DISABLE,
        .rx_flow_ctrl_thresh = 0,
        .source_clk = UART_SCLK_APB,
    };
    
    // Install UART driver
    ESP_ERROR_CHECK(uart_driver_install(UART_NUM_0, 1024 * 2, 1024 * 2, 20, NULL, 0));
    ESP_ERROR_CHECK(uart_param_config(UART_NUM_0, &uart_config));
    ESP_ERROR_CHECK(uart_set_pin(UART_NUM_0, GPIO_NUM_1, GPIO_NUM_3, UART_PIN_NO_CHANGE, UART_PIN_NO_CHANGE));
    
    while (!system_manager_is_initialized()) {
        vTaskDelay(pdMS_TO_TICKS(100));
    }
    
    ESP_LOGI(TAG, "UART initialized, starting command processing...");
    
    // UART buffer for accumulating JSON commands (like Arduino implementation)
    static char uart_buffer[2048];
    static size_t buffer_pos = 0;
    static uint32_t last_char_time = 0;
    const uint32_t JSON_TIMEOUT_MS = 100; // 100ms timeout for JSON completion
    
    char* temp_data = (char*) malloc(1024);
    
    // UART command processing loop
    while (1) {
        // Read data from UART
        int len = uart_read_bytes(UART_NUM_0, temp_data, 1023, pdMS_TO_TICKS(10));
        if (len > 0) {
            temp_data[len] = '\0';
            last_char_time = esp_timer_get_time() / 1000; // Convert to milliseconds
            
            // Append to buffer
            for (int i = 0; i < len && buffer_pos < sizeof(uart_buffer) - 1; i++) {
                char c = temp_data[i];
                uart_buffer[buffer_pos++] = c;
                
                // Check for end of JSON command (newline character)
                if (c == '\n') {
                    uart_buffer[buffer_pos] = '\0';
                    ESP_LOGI(TAG, "Complete JSON command received (%d bytes): %s", buffer_pos, uart_buffer);
                    
                    // Parse JSON command
                    json_command_t cmd;
                    esp_err_t result = json_parser_parse_command(uart_buffer, &cmd);
                    if (result == ESP_OK) {
                        ESP_LOGI(TAG, "Successfully parsed command type: %d", cmd.type);
                        
                        // Process the command
                        result = json_parser_process_command(&cmd);
                        if (result != ESP_OK) {
                            ESP_LOGE(TAG, "Failed to process command: %s", esp_err_to_name(result));
                        }
                    } else {
                        ESP_LOGE(TAG, "Failed to parse JSON command: %s", esp_err_to_name(result));
                    }
                    
                    // Reset buffer for next command
                    buffer_pos = 0;
                }
            }
        }
        
        // Handle timeout for incomplete JSON (like Arduino implementation)
        uint32_t current_time = esp_timer_get_time() / 1000;
        if (buffer_pos > 0 && (current_time - last_char_time > JSON_TIMEOUT_MS)) {
            ESP_LOGW(TAG, "JSON timeout - clearing buffer (%d bytes): %.*s", buffer_pos, buffer_pos, uart_buffer);
            buffer_pos = 0;
        }
        
        vTaskDelay(pdMS_TO_TICKS(10)); // 100Hz UART processing rate
    }
    
    free(temp_data);
}
