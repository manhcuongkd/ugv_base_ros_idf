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
#include "../inc/uart_controller.h"
#include "../inc/oled_controller.h"
#include "../inc/imu_controller.h"
#include "../inc/gimbal_controller.h"
#include "../inc/motion_module.h"
#include "../inc/mission_system.h"
#include "../inc/ugv_advanced.h"
#include "../inc/module_handlers.h"
#include "../inc/system_manager.h"
#include "../inc/esp_now_controller.h"
#include "../inc/battery_controller.h"
#include "../inc/led_controller.h"
#include "../inc/wifi_controller.h"
#include "../inc/http_server.h"

static const char *TAG = "Main";

// Helper function to wait for system initialization (eliminates redundancy)
static void wait_for_system_initialization(void)
{
    while (!system_manager_is_initialized()) {
        vTaskDelay(pdMS_TO_TICKS(100));
    }
}

// Global variables
static QueueHandle_t json_cmd_queue;
static TaskHandle_t main_task_handle;
static TaskHandle_t motion_control_task_handle;
static TaskHandle_t esp_now_flow_control_task_handle;
static TaskHandle_t system_monitor_task_handle;
static TaskHandle_t battery_monitor_task_handle;
static TaskHandle_t led_status_task_handle;

// Configuration
ugv_config_t ugv_config = {
    .main_type = 2,        // UGV Rover
    .module_type = 2,      // Gimbal
    .info_print = 1,       // Debug info
    .esp_now_mode = 3,     // Follower mode
    .ctrl_by_broadcast = true,
    .steady_mode = false,
    .base_feedback_flow = false,
    .eem_mode = 0          // Default EEM mode
};

// Task function declarations
static void main_task(void *pvParameters);
static void motion_control_task(void *pvParameters);
static void esp_now_flow_control_task(void *pvParameters);
static void system_monitor_task(void *pvParameters);
static void battery_monitor_task(void *pvParameters);
static void led_status_task(void *pvParameters);

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
    
    // Create main coordination task
    xTaskCreatePinnedToCore(main_task, "main_task", 32768, NULL, 5, &main_task_handle, 0);
    
    // Start controller-specific tasks (only those that have task implementations)
    xTaskCreatePinnedToCore(uart_controller_task, "uart_task", 8192, NULL, 4, NULL, 0);
    
    // NOTE: IMU controller doesn't have a dedicated task function yet
    // It is managed through individual API functions called from other tasks
    // Motion control now has its own dedicated task (motion_control_task) below
    
    // Create critical control tasks
    xTaskCreatePinnedToCore(motion_control_task, "motion_ctrl", 4096, NULL, 6, &motion_control_task_handle, 0);
    
    // Create system-level tasks
    xTaskCreatePinnedToCore(esp_now_flow_control_task, "esp_now_flow", 4096, NULL, 3, &esp_now_flow_control_task_handle, 1);
    xTaskCreatePinnedToCore(system_monitor_task, "sys_monitor", 3072, NULL, 2, &system_monitor_task_handle, 0);
    xTaskCreatePinnedToCore(battery_monitor_task, "battery_mon", 3072, NULL, 2, &battery_monitor_task_handle, 1);
    xTaskCreatePinnedToCore(led_status_task, "led_status", 2048, NULL, 1, &led_status_task_handle, 0);
    
    ESP_LOGI(TAG, "RaspRover application started successfully");
}

// Main application task
static void main_task(void *pvParameters)
{
    ESP_LOGI(TAG, "Main task started");
    
    // Wait for system initialization
    wait_for_system_initialization();
    
    // Check stack usage after initialization
    UBaseType_t stack_remaining = uxTaskGetStackHighWaterMark(NULL);
    ESP_LOGI(TAG, "Main task stack remaining after init: %u bytes", stack_remaining * sizeof(StackType_t));
    
    // Display startup message
    oled_controller_display_text(0, "RaspRover IDF");
    oled_controller_display_text(1, "Version: 1.0.0");
    oled_controller_display_text(2, "Starting...");
    oled_controller_display_text(3, "");
    oled_controller_update();
    
    vTaskDelay(pdMS_TO_TICKS(1200));
    
    // Switch back to default mode for periodic updates
    oled_controller_reset_to_default();
    
    // Main application loop
    while (1) {
        // JSON commands are now processed directly in uart_controller
        
        // Update system status
        if (ugv_config.base_feedback_flow) {
            // Send base feedback
            base_info_feedback();
        }
        
        // Heartbeat control
        heart_beat_ctrl();
        
        // Update OLED display (Arduino-style periodic update)
        oled_controller_info_update();
        
        vTaskDelay(pdMS_TO_TICKS(1000)); // Standard main task frequency
    }
}

// Motion control task (CRITICAL: Continuous PID control loop)
static void motion_control_task(void *pvParameters)
{
    ESP_LOGI(TAG, "Motion control task started");
    
    wait_for_system_initialization();
    
    // Initialize motion control components
    motion_module_init_encoders();
    motion_module_init_pid();
    
    ESP_LOGI(TAG, "Motion control loop starting - 50Hz PID control");
    
    // CRITICAL: Continuous motion control loop
    while (1) {
        if (!system_manager_is_emergency_stop()) {
            // 1. Update encoder readings (get current wheel speeds)
            motion_module_update_encoders();
            
            // 2. Compute PID control (calculate motor commands based on setpoint vs actual)
            motion_module_compute_pid();
            
            // 3. Apply motor control signals (send PWM to motors)
            motion_module_apply_motor_control();
        } else {
            // During emergency stop, ensure motors are stopped
            motion_module_emergency_stop();
        }
        
        // Run at 50Hz (20ms) for smooth motion control
        vTaskDelay(pdMS_TO_TICKS(20));
    }
}

// ESP-NOW flow control task (high-level coordination)
static void esp_now_flow_control_task(void *pvParameters)
{
    ESP_LOGI(TAG, "ESP-NOW flow control task started");
    
    wait_for_system_initialization();
    
    // ESP-NOW flow control coordination
    while (1) {
        if (ugv_config.esp_now_mode == 1) { // Flow leader group
            // Group flow control
            esp_now_controller_group_devs_flow_ctrl();
        } else if (ugv_config.esp_now_mode == 2) { // Flow leader single
            // Single device flow control
            esp_now_controller_single_dev_flow_ctrl();
        }
        
        // Handle mode changes and peer management
        // TODO: Add ESP-NOW peer management logic
        
        vTaskDelay(pdMS_TO_TICKS(1000)); // 1Hz flow control rate
    }
}

// System monitor task (high-level system health)
static void system_monitor_task(void *pvParameters)
{
    ESP_LOGI(TAG, "System monitor task started");
    
    wait_for_system_initialization();
    
    // System monitoring loop
    while (1) {
        // Check system health
        UBaseType_t free_stack = uxTaskGetStackHighWaterMark(NULL);
        size_t free_heap = esp_get_free_heap_size();
        
        // Log system status periodically
        static uint32_t status_counter = 0;
        if (++status_counter >= 30) { // Every 30 seconds
            ESP_LOGI(TAG, "System Status - Free Stack: %d, Free Heap: %d, Uptime: %lld ms", 
                     free_stack, free_heap, esp_timer_get_time() / 1000);
            
            // Update OLED with system info
            char status_line[32];
            snprintf(status_line, sizeof(status_line), "Heap:%dKB", free_heap / 1024);
            oled_controller_display_text(1, status_line);
            oled_controller_update();
            
            status_counter = 0;
        }
        
        // Check for emergency conditions
        if (free_heap < 10240) { // Less than 10KB free
            ESP_LOGW(TAG, "Low memory warning: %d bytes free", free_heap);
        }
        
        // Module-specific monitoring
        if (ugv_config.module_type == 2) { // Gimbal mode
            // Monitor gimbal status
            // TODO: Add gimbal health checks
        }
        
        vTaskDelay(pdMS_TO_TICKS(1000)); // 1Hz monitoring rate
    }
}

// Battery monitor task (high-level power management)
static void battery_monitor_task(void *pvParameters)
{
    ESP_LOGI(TAG, "Battery monitor task started");
    
    wait_for_system_initialization();
    
    // Battery monitoring loop
    while (1) {
        // Check battery status using individual functions (since get_info is not implemented yet)
        float voltage;
        uint8_t percentage;
        if (battery_controller_read_voltage(&voltage) == ESP_OK) {
            percentage = battery_controller_get_percentage();
            
            // Check battery levels
            if (percentage <= 10) {
                ESP_LOGW(TAG, "Battery critical: %d%%", percentage);
                // TODO: Trigger low battery actions
                
                // Update OLED with battery warning
                oled_controller_display_text(2, "BATTERY LOW!");
                oled_controller_update();
            } else if (percentage <= 20) {
                ESP_LOGW(TAG, "Battery low: %d%%", percentage);
            }
            
            // Periodic battery status update
            static uint32_t battery_counter = 0;
            if (++battery_counter >= 10) { // Every 10 seconds
                ESP_LOGI(TAG, "Battery: %.2fV %d%%", voltage, percentage);
                battery_counter = 0;
            }
        }
        
        vTaskDelay(pdMS_TO_TICKS(1000)); // 1Hz battery monitoring rate
    }
}

// LED status task (visual feedback coordination)
static void led_status_task(void *pvParameters)
{
    ESP_LOGI(TAG, "LED status task started");
    
    wait_for_system_initialization();
    
    // LED status coordination loop
    while (1) {
        // System status indication
        if (system_manager_is_emergency_stop()) {
            // Red blinking for emergency stop
            led_controller_set_mode(LED_MODE_BLINK);
            led_controller_set_color(LED_COLOR_RED);
        } else if (system_manager_is_initialized()) {
            // Green for normal operation
            led_controller_set_mode(LED_MODE_ON);
            led_controller_set_color(LED_COLOR_GREEN);
        } else {
            // Blue pulsing for initialization
            led_controller_set_mode(LED_MODE_PULSE);
            led_controller_set_color(LED_COLOR_BLUE);
        }
        
        // Module-specific LED patterns
        if (ugv_config.module_type == 2) { // Gimbal mode
            // TODO: Add gimbal-specific LED patterns
        }
        
        // ESP-NOW status indication
        if (ugv_config.esp_now_mode != 0) { // Not none
            // TODO: Add ESP-NOW status LED patterns
        }
        
        vTaskDelay(pdMS_TO_TICKS(100)); // 10Hz LED update rate
    }
}