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
#include "../inc/oled_controller.h"
#include "../inc/battery_controller.h"
#include "../inc/led_controller.h"
#include "../inc/mission_system.h"
#include "../inc/ugv_advanced.h"

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
    
    // Initialize file system
    esp_vfs_spiffs_conf_t conf = {
        .base_path = "/spiffs",
        .partition_label = NULL,
        .max_files = 5,
        .format_if_mount_failed = true
    };
    
    esp_err_t ret = esp_vfs_spiffs_register(&conf);
    if (ret != ESP_OK) {
        ESP_LOGE(TAG, "Failed to mount SPIFFS (%s)", esp_err_to_name(ret));
        return ret;
    }
    
    // Create boot mission (matching Arduino implementation)
    ESP_LOGI(TAG, "Creating boot mission...");
    create_mission("boot", "these cmds run automatically at boot.");
    
    // Save main type and module type to boot mission
    save_main_type_module_type(1, 2); // RaspRover, Gimbal
    
    // Play boot mission once
    mission_play("boot", 1);
    
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
    
    // Initialize IMU
    ESP_ERROR_CHECK(imu_controller_init());
    
    // Initialize OLED
    ESP_ERROR_CHECK(oled_controller_init());
    
    // Initialize battery controller
    ESP_ERROR_CHECK(battery_controller_init());
    
    // Initialize LED controller
    ESP_ERROR_CHECK(led_controller_init());
    
    // Initialize servo controller
    ESP_ERROR_CHECK(servo_controller_init());
    
    ESP_LOGI(TAG, "Hardware initialization completed");
    return ESP_OK;
}

esp_err_t system_manager_init_communication(void)
{
    ESP_LOGI(TAG, "Initializing communication...");
    
    // Initialize WiFi - temporarily disabled due to type conflicts
    // ESP_ERROR_CHECK(wifi_controller_init());
    
    // Initialize HTTP server - temporarily disabled due to type conflicts
    // ESP_ERROR_CHECK(http_server_init());
    
    // Initialize ESP-NOW
    ESP_ERROR_CHECK(esp_now_init());
    
    // Initialize UART
    uart_config_t uart_config = {
        .baud_rate = 115200,
        .data_bits = UART_DATA_8_BITS,
        .parity = UART_PARITY_DISABLE,
        .stop_bits = UART_STOP_BITS_1,
        .flow_ctrl = UART_HW_FLOWCTRL_DISABLE,
        .source_clk = UART_SCLK_APB
    };
    ESP_ERROR_CHECK(uart_param_config(UART_NUM_0, &uart_config));
    ESP_ERROR_CHECK(uart_driver_install(UART_NUM_0, 1024, 1024, 0, NULL, 0));
    
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
