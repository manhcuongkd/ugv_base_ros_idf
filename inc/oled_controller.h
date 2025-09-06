#ifndef OLED_CONTROLLER_H
#define OLED_CONTROLLER_H

#include <stdint.h>
#include <stdbool.h>
#include <esp_err.h>
#include "ugv_config.h"

#ifdef __cplusplus
extern "C" {
#endif

// OLED display configuration
#define OLED_WIDTH               128
#define OLED_HEIGHT              64
#define OLED_MAX_LINES           4
#define OLED_MAX_CHARS_PER_LINE 16

// OLED text lines
typedef struct {
    char text[OLED_MAX_CHARS_PER_LINE + 1];  // Text content
    bool updated;                             // Update flag
    uint32_t timestamp;                       // Last update timestamp
} oled_line_t;

// OLED display state
typedef struct {
    oled_line_t lines[OLED_MAX_LINES];        // Display lines
    bool display_on;                          // Display power state
    uint8_t brightness;                       // Display brightness (0-255)
    uint32_t last_update;                     // Last update timestamp
    bool initialized;                          // Initialization flag
} oled_state_t;

// OLED text alignment
typedef enum {
    OLED_ALIGN_LEFT = 0,      // Left aligned
    OLED_ALIGN_CENTER,        // Center aligned
    OLED_ALIGN_RIGHT          // Right aligned
} oled_alignment_t;

// OLED font size
typedef enum {
    OLED_FONT_SMALL = 0,      // Small font (6x8)
    OLED_FONT_MEDIUM,         // Medium font (8x16)
    OLED_FONT_LARGE           // Large font (12x16)
} oled_font_size_t;

// Function prototypes

/**
 * @brief Initialize OLED controller
 * @return ESP_OK on success, error code otherwise
 */
esp_err_t oled_controller_init(void);

/**
 * @brief Deinitialize OLED controller
 * @return ESP_OK on success, error code otherwise
 */
esp_err_t oled_controller_deinit(void);

/**
 * @brief Display text on specific line
 * @param line_num Line number (0-3)
 * @param text Text to display
 * @return ESP_OK on success, error code otherwise
 */
esp_err_t oled_controller_display_text(uint8_t line_num, const char *text);

/**
 * @brief Display text with alignment
 * @param line_num Line number (0-3)
 * @param text Text to display
 * @param alignment Text alignment
 * @return ESP_OK on success, error code otherwise
 */
esp_err_t oled_controller_display_text_aligned(uint8_t line_num, const char *text, oled_alignment_t alignment);

/**
 * @brief Display text with font size
 * @param line_num Line number (0-3)
 * @param text Text to display
 * @param font_size Font size
 * @return ESP_OK on success, error code otherwise
 */
esp_err_t oled_controller_display_text_font(uint8_t line_num, const char *text, oled_font_size_t font_size);

/**
 * @brief Clear specific line
 * @param line_num Line number (0-3)
 * @return ESP_OK on success, error code otherwise
 */
esp_err_t oled_controller_clear_line(uint8_t line_num);

/**
 * @brief Clear entire display
 * @return ESP_OK on success, error code otherwise
 */
esp_err_t oled_controller_clear_display(void);

/**
 * @brief Set text on specific line
 * @param line_num Line number (0-3)
 * @param text Text to display
 * @return ESP_OK on success, error code otherwise
 */
esp_err_t oled_controller_set_text(uint8_t line_num, const char *text);

/**
 * @brief Reset display to default state
 * @return ESP_OK on success, error code otherwise
 */
esp_err_t oled_controller_reset_to_default(void);

/**
 * @brief Arduino-style OLED control function
 * @param line_num Line number (0-3)
 * @param text Text to display
 * @return ESP_OK on success, error code otherwise
 */
esp_err_t oled_controller_control(uint8_t line_num, const char *text);

/**
 * @brief Arduino-style periodic info update
 * @return ESP_OK on success, error code otherwise
 */
esp_err_t oled_controller_info_update(void);

/**
 * @brief Update display (refresh all lines)
 * @return ESP_OK on success, error code otherwise
 */
esp_err_t oled_controller_update(void);

/**
 * @brief Update specific line
 * @param line_num Line number (0-3)
 * @return ESP_OK on success, error code otherwise
 */
esp_err_t oled_controller_update_line(uint8_t line_num);

/**
 * @brief Update system information display
 * @return ESP_OK on success, error code otherwise
 */
esp_err_t oled_controller_update_system_info(void);

/**
 * @brief Display WiFi information
 * @param ssid WiFi SSID
 * @param rssi Signal strength
 * @param ip_addr IP address
 * @return ESP_OK on success, error code otherwise
 */
esp_err_t oled_controller_display_wifi_info(const char *ssid, int8_t rssi, const char *ip_addr);

/**
 * @brief Display battery information
 * @param voltage Battery voltage
 * @param percentage Battery percentage
 * @param charging Charging status
 * @return ESP_OK on success, error code otherwise
 */
esp_err_t oled_controller_display_battery_info(float voltage, uint8_t percentage, bool charging);

/**
 * @brief Display IMU information
 * @param roll Roll angle
 * @param pitch Pitch angle
 * @param yaw Yaw angle
 * @return ESP_OK on success, error code otherwise
 */
esp_err_t oled_controller_display_imu_info(float roll, float pitch, float yaw);

/**
 * @brief Display motion information
 * @param left_speed Left motor speed
 * @param right_speed Right motor speed
 * @param linear_speed Linear speed
 * @param angular_speed Angular speed
 * @return ESP_OK on success, error code otherwise
 */
esp_err_t oled_controller_display_motion_info(float left_speed, float right_speed, float linear_speed, float angular_speed);

/**
 * @brief Display startup animation
 * @return ESP_OK on success, error code otherwise
 */
esp_err_t oled_controller_startup_animation(void);

/**
 * @brief Display error message
 * @param error_code Error code
 * @param error_msg Error message
 * @return ESP_OK on success, error code otherwise
 */
esp_err_t oled_controller_display_error(uint8_t error_code, const char *error_msg);

/**
 * @brief Display status message
 * @param status Status message
 * @param timeout_ms Display timeout in milliseconds
 * @return ESP_OK on success, error code otherwise
 */
esp_err_t oled_controller_display_status(const char *status, uint32_t timeout_ms);

/**
 * @brief Set display brightness
 * @param brightness Brightness value (0-255)
 * @return ESP_OK on success, error code otherwise
 */
esp_err_t oled_controller_set_brightness(uint8_t brightness);

/**
 * @brief Get display brightness
 * @param brightness Pointer to brightness value
 * @return ESP_OK on success, error code otherwise
 */
esp_err_t oled_controller_get_brightness(uint8_t *brightness);

/**
 * @brief Turn display on/off
 * @param on True to turn on, false to turn off
 * @return ESP_OK on success, error code otherwise
 */
esp_err_t oled_controller_set_power(bool on);

/**
 * @brief Get display power state
 * @param on Pointer to power state
 * @return ESP_OK on success, error code otherwise
 */
esp_err_t oled_controller_get_power(bool *on);

/**
 * @brief Display scrolling text
 * @param line_num Line number (0-3)
 * @param text Text to scroll
 * @param scroll_speed Scroll speed in pixels per frame
 * @return ESP_OK on success, error code otherwise
 */
esp_err_t oled_controller_scroll_text(uint8_t line_num, const char *text, uint8_t scroll_speed);

/**
 * @brief Stop text scrolling
 * @param line_num Line number (0-3)
 * @return ESP_OK on success, error code otherwise
 */
esp_err_t oled_controller_stop_scroll(uint8_t line_num);

/**
 * @brief Display progress bar
 * @param line_num Line number (0-3)
 * @param progress Progress value (0-100)
 * @param label Progress label
 * @return ESP_OK on success, error code otherwise
 */
esp_err_t oled_controller_display_progress(uint8_t line_num, uint8_t progress, const char *label);

/**
 * @brief Display menu
 * @param items Menu items array
 * @param item_count Number of menu items
 * @param selected_item Selected item index
 * @return ESP_OK on success, error code otherwise
 */
esp_err_t oled_controller_display_menu(const char **items, uint8_t item_count, uint8_t selected_item);

/**
 * @brief Get OLED state
 * @param state Pointer to OLED state structure
 * @return ESP_OK on success, error code otherwise
 */
esp_err_t oled_controller_get_state(oled_state_t *state);

/**
 * @brief Check if line has been updated
 * @param line_num Line number (0-3)
 * @return True if updated, false otherwise
 */
bool oled_controller_is_line_updated(uint8_t line_num);

/**
 * @brief Reset line update flag
 * @param line_num Line number (0-3)
 * @return ESP_OK on success, error code otherwise
 */
esp_err_t oled_controller_reset_line_update_flag(uint8_t line_num);

// Default display lines
extern oled_line_t screen_line_0;
extern oled_line_t screen_line_1;
extern oled_line_t screen_line_2;
extern oled_line_t screen_line_3;

// Global OLED state
extern oled_state_t oled_state;

#ifdef __cplusplus
}
#endif

#endif // OLED_CONTROLLER_H
