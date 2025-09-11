#include <stdio.h>
#include <string.h>
#include <driver/uart.h>
#include <driver/gpio.h>
#include <driver/ledc.h>
#include <driver/i2c.h>
#include <driver/spi_master.h>
#include <esp_system.h>
#include <esp_wifi.h>
#include <esp_event.h>
#include <esp_now.h>
#include <esp_http_server.h>
#include <esp_spiffs.h>
#include <esp_log.h>

#include "../inc/system_manager.h"
#include "../inc/motion_module.h"
#include "../inc/imu_controller.h"
#include "../inc/servo_controller.h"
#include "../inc/gimbal_controller.h"
#include "../inc/oled_controller.h"
#include "../inc/battery_controller.h"
#include "../inc/led_controller.h"
#include "../inc/mission_system.h"
#include "../inc/ugv_advanced.h"
#include "../inc/json_parser.h"
#include "../inc/uart_controller.h"
#include "../inc/esp_now_controller.h"
#include "../inc/files_controller.h"

static const char *TAG = "System_Manager";

// System state
static bool system_initialized = false;
static bool emergency_stop = false;

// ============================================================================
// SYSTEM MANAGEMENT FUNCTIONS
// ============================================================================

esp_err_t system_manager_init(void)
{
    ESP_LOGI(TAG, "Initializing system components...");
    
    // Initialize hardware
    ESP_ERROR_CHECK(system_manager_init_hardware());
    
    // Initialize communication
    ESP_ERROR_CHECK(system_manager_init_communication());
    
    // Initialize file system (SPIFFS - Arduino uses LittleFS but SPIFFS is compatible)
    esp_vfs_spiffs_conf_t conf = {
        .base_path = "/spiffs",
        .partition_label = NULL,
        .max_files = 5,
        .format_if_mount_failed = true
    };
    
    esp_err_t ret = esp_vfs_spiffs_register(&conf);
    if (ret != ESP_OK) {
        ESP_LOGW(TAG, "Failed to mount SPIFFS (%s), continuing without file system", esp_err_to_name(ret));
        // Continue without SPIFFS - file system is non-critical for basic operation  
        // Note: Arduino uses LittleFS, future improvement would migrate to LittleFS
    } else {
        ESP_LOGI(TAG, "SPIFFS mounted successfully");
    }
    
    // Initialize Files Controller (depends on SPIFFS)
    esp_err_t files_result = files_controller_init();
    if (files_result != ESP_OK) {
        ESP_LOGW(TAG, "Files controller initialization failed (%s), continuing without file operations", esp_err_to_name(files_result));
    }
    
    // Create boot mission only if file system is available (matching Arduino implementation)
    if (files_result == ESP_OK) {
        ESP_LOGI(TAG, "SPIFFS available - boot mission creation deferred to reduce stack usage");
        // TODO: Move boot mission creation to a separate task to reduce stack pressure
        // create_mission("boot", "these cmds run automatically at boot.");
        // save_main_type_module_type(1, 2); // RaspRover, Gimbal
        // mission_play("boot", 1);
    } else {
        ESP_LOGI(TAG, "Skipping boot mission creation - no file system available");
    }
    
    system_initialized = true;
    ESP_LOGI(TAG, "System initialization completed");
    return ESP_OK;
}

esp_err_t system_manager_init_hardware(void)
{
    ESP_LOGI(TAG, "Initializing hardware...");
    
    // Initialize I2C
    i2c_config_t i2c_config = {
        .mode = I2C_MODE_MASTER,
        .sda_io_num = S_SDA,
        .scl_io_num = S_SCL,
        .sda_pullup_en = GPIO_PULLUP_ENABLE,
        .scl_pullup_en = GPIO_PULLUP_ENABLE,
        .master = {
            .clk_speed = 400000
        },
        .clk_flags = 0
    };
    ESP_ERROR_CHECK(i2c_param_config(I2C_NUM_0, &i2c_config));
    ESP_ERROR_CHECK(i2c_driver_install(I2C_NUM_0, I2C_MODE_MASTER, 0, 0, 0));
    
    // Initialize motion module
    ESP_ERROR_CHECK(motion_module_init());
    
    // Initialize IMU (non-critical - continue even if IMU is not connected)
    esp_err_t imu_result = imu_controller_init();
    if (imu_result != ESP_OK) {
        ESP_LOGW(TAG, "IMU initialization failed (%s), continuing without IMU", esp_err_to_name(imu_result));
    }
    
    // Initialize OLED (non-critical - continue even if OLED is not connected)
    esp_err_t oled_result = oled_controller_init();
    if (oled_result != ESP_OK) {
        ESP_LOGW(TAG, "OLED initialization failed (%s), continuing without OLED", esp_err_to_name(oled_result));
    }
    
    // Initialize battery controller (non-critical - continue even if INA219 is not connected)
    esp_err_t battery_result = battery_controller_init();
    if (battery_result != ESP_OK) {
        ESP_LOGW(TAG, "Battery controller initialization failed (%s), continuing without battery monitoring", esp_err_to_name(battery_result));
    }
    
    // Initialize LED controller
    ESP_ERROR_CHECK(led_controller_init());
    
    // Initialize servo and gimbal controllers for API testing
    ESP_LOGI(TAG, "Initializing servo and gimbal controllers...");
    ESP_ERROR_CHECK(servo_controller_init());
    esp_err_t gimbal_result = gimbal_controller_init();
    if (gimbal_result != ESP_OK) {
        ESP_LOGW(TAG, "Gimbal controller initialization failed (%s), continuing without gimbal", esp_err_to_name(gimbal_result));
    } else {
        ESP_LOGI(TAG, "✓ Gimbal controller initialized successfully");
    }
    
    ESP_LOGI(TAG, "Hardware initialization completed");
    return ESP_OK;
}

esp_err_t system_manager_init_communication(void)
{
    ESP_LOGI(TAG, "Initializing communication...");
    
    // Initialize JSON parser (for UART command processing)
    ESP_ERROR_CHECK(json_parser_init());
    
    // Initialize WiFi - temporarily disabled due to type conflicts
    // ESP_ERROR_CHECK(wifi_controller_init());
    
    // Initialize HTTP server - temporarily disabled due to type conflicts
    // ESP_ERROR_CHECK(http_server_init());
    
    // Initialize ESP-NOW controller (non-critical)
    esp_err_t esp_now_result = esp_now_controller_init();
    if (esp_now_result != ESP_OK) {
        ESP_LOGW(TAG, "ESP-NOW controller initialization failed (%s), continuing without ESP-NOW", esp_err_to_name(esp_now_result));
    }
    
    // Initialize UART controller
    esp_err_t uart_result = uart_controller_init();
    if (uart_result != ESP_OK) {
        ESP_LOGE(TAG, "UART controller initialization failed (%s)", esp_err_to_name(uart_result));
        return uart_result;  // UART is critical for communication
    }
    
    ESP_LOGI(TAG, "Communication initialization completed");
    return ESP_OK;
}

bool system_manager_is_initialized(void)
{
    return system_initialized;
}

bool system_manager_is_emergency_stop(void)
{
    return emergency_stop;
}

void system_manager_set_emergency_stop(bool stop)
{
    emergency_stop = stop;
    ESP_LOGI(TAG, "Emergency stop %s", stop ? "activated" : "deactivated");
}
