#ifndef BATTERY_CONTROLLER_H
#define BATTERY_CONTROLLER_H

#include <stdint.h>
#include <stdbool.h>
#include <esp_err.h>
#include "ugv_config.h"

#ifdef __cplusplus
extern "C" {
#endif

// Battery status
typedef enum {
    BATTERY_STATUS_UNKNOWN = 0,     // Battery status unknown
    BATTERY_STATUS_NORMAL,          // Battery normal
    BATTERY_STATUS_LOW,             // Battery low
    BATTERY_STATUS_CRITICAL,        // Battery critical
    BATTERY_STATUS_CHARGING,        // Battery charging
    BATTERY_STATUS_FULL,            // Battery full
    BATTERY_STATUS_FAULT            // Battery fault
} battery_status_t;

// Battery information
typedef struct {
    float voltage;                  // Battery voltage (V)
    float current;                  // Battery current (A)
    float power;                    // Battery power (W)
    uint8_t percentage;             // Battery percentage (0-100)
    battery_status_t status;        // Battery status
    float temperature;              // Battery temperature (°C)
    uint32_t capacity_mah;         // Battery capacity (mAh)
    uint32_t remaining_mah;        // Remaining capacity (mAh)
    uint32_t timestamp;             // Last update timestamp
    bool valid;                     // Data validity flag
} battery_info_t;

// Battery configuration
typedef struct {
    float min_voltage;              // Minimum voltage threshold (V)
    float low_voltage;              // Low voltage threshold (V)
    float critical_voltage;         // Critical voltage threshold (V)
    float max_voltage;              // Maximum voltage threshold (V)
    float max_current;              // Maximum current (A)
    float max_temperature;          // Maximum temperature (°C)
    uint32_t capacity_mah;         // Nominal capacity (mAh)
    bool enable_low_voltage_protection; // Enable low voltage protection
    bool enable_overcurrent_protection; // Enable overcurrent protection
    bool enable_overtemperature_protection; // Enable overtemperature protection
} battery_config_t;

// Function prototypes

/**
 * @brief Initialize battery controller
 * @return ESP_OK on success, error code otherwise
 */
esp_err_t battery_controller_init(void);

/**
 * @brief Deinitialize battery controller
 * @return ESP_OK on success, error code otherwise
 */
esp_err_t battery_controller_deinit(void);

/**
 * @brief Configure battery settings
 * @param config Pointer to battery configuration
 * @return ESP_OK on success, error code otherwise
 */
esp_err_t battery_controller_configure(battery_config_t *config);

/**
 * @brief Read battery voltage
 * @param voltage Pointer to voltage value
 * @return ESP_OK on success, error code otherwise
 */
esp_err_t battery_controller_read_voltage(float *voltage);

/**
 * @brief Read battery current
 * @param current Pointer to current value
 * @return ESP_OK on success, error code otherwise
 */
esp_err_t battery_controller_read_current(float *current);

/**
 * @brief Read battery power
 * @param power Pointer to power value
 * @return ESP_OK on success, error code otherwise
 */
esp_err_t battery_controller_read_power(float *power);

/**
 * @brief Read battery temperature
 * @param temperature Pointer to temperature value
 * @return ESP_OK on success, error code otherwise
 */
esp_err_t battery_controller_read_temperature(float *temperature);

/**
 * @brief Get battery percentage
 * @return Battery percentage (0-100)
 */
uint8_t battery_controller_get_percentage(void);

/**
 * @brief Get battery status
 * @param status Pointer to battery status
 * @return ESP_OK on success, error code otherwise
 */
esp_err_t battery_controller_get_status(battery_status_t *status);

/**
 * @brief Get complete battery information
 * @param info Pointer to battery information structure
 * @return ESP_OK on success, error code otherwise
 */
esp_err_t battery_controller_get_info(battery_info_t *info);

/**
 * @brief Update battery information
 * @return ESP_OK on success, error code otherwise
 */
esp_err_t battery_controller_update_info(void);

/**
 * @brief Check if battery is low
 * @return True if battery is low, false otherwise
 */
bool battery_controller_is_low(void);

/**
 * @brief Check if battery is critical
 * @return True if battery is critical, false otherwise
 */
bool battery_controller_is_critical(void);

/**
 * @brief Check if battery is charging
 * @return True if battery is charging, false otherwise
 */
bool battery_controller_is_charging(void);

/**
 * @brief Check if battery is full
 * @return True if battery is full, false otherwise
 */
bool battery_controller_is_full(void);

/**
 * @brief Get battery voltage as string
 * @param voltage_str Buffer for voltage string
 * @param max_len Maximum buffer length
 * @return ESP_OK on success, error code otherwise
 */
esp_err_t battery_controller_get_voltage_string(char *voltage_str, size_t max_len);

/**
 * @brief Get battery current as string
 * @param current_str Buffer for current string
 * @param max_len Maximum buffer length
 * @return ESP_OK on success, error code otherwise
 */
esp_err_t battery_controller_get_current_string(char *current_str, size_t max_len);

/**
 * @brief Get battery power as string
 * @param power_str Buffer for power string
 * @param max_len Maximum buffer length
 * @return ESP_OK on success, error code otherwise
 */
esp_err_t battery_controller_get_power_string(char *power_str, size_t max_len);

/**
 * @brief Get battery percentage as string
 * @param percentage_str Buffer for percentage string
 * @param max_len Maximum buffer length
 * @return ESP_OK on success, error code otherwise
 */
esp_err_t battery_controller_get_percentage_string(char *percentage_str, size_t max_len);

/**
 * @brief Get battery status as string
 * @param status_str Buffer for status string
 * @param max_len Maximum buffer length
 * @return ESP_OK on success, error code otherwise
 */
esp_err_t battery_controller_get_status_string(char *status_str, size_t max_len);

/**
 * @brief Set low voltage threshold
 * @param voltage Low voltage threshold (V)
 * @return ESP_OK on success, error code otherwise
 */
esp_err_t battery_controller_set_low_voltage_threshold(float voltage);

/**
 * @brief Set critical voltage threshold
 * @param voltage Critical voltage threshold (V)
 * @return ESP_OK on success, error code otherwise
 */
esp_err_t battery_controller_set_critical_voltage_threshold(float voltage);

/**
 * @brief Enable/disable low voltage protection
 * @param enable True to enable, false to disable
 * @return ESP_OK on success, error code otherwise
 */
esp_err_t battery_controller_enable_low_voltage_protection(bool enable);

/**
 * @brief Enable/disable overcurrent protection
 * @param enable True to enable, false to disable
 * @return ESP_OK on success, error code otherwise
 */
esp_err_t battery_controller_enable_overcurrent_protection(bool enable);

/**
 * @brief Enable/disable overtemperature protection
 * @param enable True to enable, false to disable
 * @return ESP_OK on success, error code otherwise
 */
esp_err_t battery_controller_enable_overtemperature_protection(bool enable);

/**
 * @brief Get battery configuration
 * @param config Pointer to battery configuration
 * @return ESP_OK on success, error code otherwise
 */
esp_err_t battery_controller_get_config(battery_config_t *config);

/**
 * @brief Save battery configuration to NVS
 * @return ESP_OK on success, error code otherwise
 */
esp_err_t battery_controller_save_config(void);

/**
 * @brief Load battery configuration from NVS
 * @return ESP_OK on success, error code otherwise
 */
esp_err_t battery_controller_load_config(void);

/**
 * @brief Reset battery configuration to defaults
 * @return ESP_OK on success, error code otherwise
 */
esp_err_t battery_controller_reset_config(void);

/**
 * @brief Calibrate battery voltage reading
 * @param reference_voltage Reference voltage (V)
 * @return ESP_OK on success, error code otherwise
 */
esp_err_t battery_controller_calibrate_voltage(float reference_voltage);

/**
 * @brief Calibrate battery current reading
 * @param reference_current Reference current (A)
 * @return ESP_OK on success, error code otherwise
 */
esp_err_t battery_controller_calibrate_current(float reference_current);

// Default configuration values
#define DEFAULT_BATTERY_MIN_VOLTAGE          3.0f
#define DEFAULT_BATTERY_LOW_VOLTAGE          3.3f
#define DEFAULT_BATTERY_CRITICAL_VOLTAGE     3.0f
#define DEFAULT_BATTERY_MAX_VOLTAGE          4.2f
#define DEFAULT_BATTERY_MAX_CURRENT          5.0f
#define DEFAULT_BATTERY_MAX_TEMPERATURE      60.0f
#define DEFAULT_BATTERY_CAPACITY_MAH         3000
#define DEFAULT_BATTERY_ENABLE_LOW_VOLTAGE_PROTECTION true
#define DEFAULT_BATTERY_ENABLE_OVERCURRENT_PROTECTION true
#define DEFAULT_BATTERY_ENABLE_OVERTEMPERATURE_PROTECTION true

// Battery status strings
#define BATTERY_STATUS_STRING_UNKNOWN        "Unknown"
#define BATTERY_STATUS_STRING_NORMAL         "Normal"
#define BATTERY_STATUS_STRING_LOW            "Low"
#define BATTERY_STATUS_STRING_CRITICAL       "Critical"
#define BATTERY_STATUS_STRING_CHARGING       "Charging"
#define BATTERY_STATUS_STRING_FULL           "Full"
#define BATTERY_STATUS_STRING_FAULT          "Fault"

// Global variables (extern declarations)
extern battery_info_t current_battery_info;
extern battery_config_t battery_config;

#ifdef __cplusplus
}
#endif

#endif // BATTERY_CONTROLLER_H
