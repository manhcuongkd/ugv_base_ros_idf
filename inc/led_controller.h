#ifndef LED_CONTROLLER_H
#define LED_CONTROLLER_H

#include <stdint.h>
#include <stdbool.h>
#include <esp_err.h>
#include "ugv_config.h"

#ifdef __cplusplus
extern "C" {
#endif

// LED modes
typedef enum {
    LED_MODE_OFF = 0,              // LED off
    LED_MODE_ON,                   // LED on
    LED_MODE_BLINK,                // LED blinking
    LED_MODE_PULSE,                // LED pulsing
    LED_MODE_BREATH,               // LED breathing
    LED_MODE_RAINBOW,              // LED rainbow
    LED_MODE_CUSTOM                // Custom LED pattern
} led_mode_t;

// LED colors (for RGB LEDs)
typedef enum {
    LED_COLOR_OFF = 0,             // No color
    LED_COLOR_RED,                 // Red
    LED_COLOR_GREEN,               // Green
    LED_COLOR_BLUE,                // Blue
    LED_COLOR_YELLOW,              // Yellow
    LED_COLOR_CYAN,                // Cyan
    LED_COLOR_MAGENTA,             // Magenta
    LED_COLOR_WHITE,               // White
    LED_COLOR_CUSTOM               // Custom color
} led_color_t;

// LED configuration
typedef struct {
    uint8_t pin;                   // LED GPIO pin
    bool active_high;              // Active high/low logic
    uint8_t brightness;            // LED brightness (0-255)
    led_mode_t mode;               // LED mode
    led_color_t color;             // LED color (for RGB)
    uint16_t blink_period_ms;      // Blink period in milliseconds
    uint16_t pulse_period_ms;      // Pulse period in milliseconds
    uint16_t breath_period_ms;     // Breath period in milliseconds
    uint8_t custom_pattern[32];    // Custom pattern array
    uint8_t pattern_length;        // Custom pattern length
} led_config_t;

// LED state
typedef struct {
    bool enabled;                  // LED enabled flag
    bool on;                       // LED on/off state
    uint8_t brightness;            // Current brightness
    led_mode_t current_mode;       // Current mode
    led_color_t current_color;     // Current color
    uint32_t last_update;          // Last update timestamp
    uint8_t pattern_index;         // Current pattern index
    bool initialized;               // Initialization flag
} led_state_t;

// Function prototypes

/**
 * @brief Initialize LED controller
 * @return ESP_OK on success, error code otherwise
 */
esp_err_t led_controller_init(void);

/**
 * @brief Deinitialize LED controller
 * @return ESP_OK on success, error code otherwise
 */
esp_err_t led_controller_deinit(void);

/**
 * @brief Configure LED settings
 * @param config Pointer to LED configuration
 * @return ESP_OK on success, error code otherwise
 */
esp_err_t led_controller_configure(led_config_t *config);

/**
 * @brief Turn LED on
 * @return ESP_OK on success, error code otherwise
 */
esp_err_t led_controller_turn_on(void);

/**
 * @brief Turn LED off
 * @return ESP_OK on success, error code otherwise
 */
esp_err_t led_controller_turn_off(void);

/**
 * @brief Toggle LED state
 * @return ESP_OK on success, error code otherwise
 */
esp_err_t led_controller_toggle(void);

/**
 * @brief Set LED brightness
 * @param brightness Brightness value (0-255)
 * @return ESP_OK on success, error code otherwise
 */
esp_err_t led_controller_set_brightness(uint8_t brightness);

/**
 * @brief Get LED brightness
 * @param brightness Pointer to brightness value
 * @return ESP_OK on success, error code otherwise
 */
esp_err_t led_controller_get_brightness(uint8_t *brightness);

/**
 * @brief Set LED mode
 * @param mode LED mode
 * @return ESP_OK on success, error code otherwise
 */
esp_err_t led_controller_set_mode(led_mode_t mode);

/**
 * @brief Get LED mode
 * @param mode Pointer to LED mode
 * @return ESP_OK on success, error code otherwise
 */
esp_err_t led_controller_get_mode(led_mode_t *mode);

/**
 * @brief Set LED color (for RGB LEDs)
 * @param color LED color
 * @return ESP_OK on success, error code otherwise
 */
esp_err_t led_controller_set_color(led_color_t color);

/**
 * @brief Get LED color
 * @param color Pointer to LED color
 * @return ESP_OK on success, error code otherwise
 */
esp_err_t led_controller_get_color(led_color_t *color);

/**
 * @brief Set custom LED color (RGB values)
 * @param red Red component (0-255)
 * @param green Green component (0-255)
 * @param blue Blue component (0-255)
 * @return ESP_OK on success, error code otherwise
 */
esp_err_t led_controller_set_rgb_color(uint8_t red, uint8_t green, uint8_t blue);

/**
 * @brief Set blink period
 * @param period_ms Blink period in milliseconds
 * @return ESP_OK on success, error code otherwise
 */
esp_err_t led_controller_set_blink_period(uint16_t period_ms);

/**
 * @brief Set pulse period
 * @param period_ms Pulse period in milliseconds
 * @return ESP_OK on success, error code otherwise
 */
esp_err_t led_controller_set_pulse_period(uint16_t period_ms);

/**
 * @brief Set breath period
 * @param period_ms Breath period in milliseconds
 * @return ESP_OK on success, error code otherwise
 */
esp_err_t led_controller_set_breath_period(uint16_t period_ms);

/**
 * @brief Set custom LED pattern
 * @param pattern Pattern array
 * @param length Pattern length
 * @return ESP_OK on success, error code otherwise
 */
esp_err_t led_controller_set_custom_pattern(const uint8_t *pattern, uint8_t length);

/**
 * @brief Start LED animation
 * @return ESP_OK on success, error code otherwise
 */
esp_err_t led_controller_start_animation(void);

/**
 * @brief Stop LED animation
 * @return ESP_OK on success, error code otherwise
 */
esp_err_t led_controller_stop_animation(void);

/**
 * @brief Update LED state (call in main loop)
 * @return ESP_OK on success, error code otherwise
 */
esp_err_t led_controller_update(void);

/**
 * @brief Set LED PWM control
 * @param channel PWM channel
 * @param duty_cycle PWM duty cycle (0-255)
 * @return ESP_OK on success, error code otherwise
 */
esp_err_t led_controller_set_pwm(uint8_t channel, uint8_t duty_cycle);

/**
 * @brief Enable/disable LED
 * @param enable True to enable, false to disable
 * @return ESP_OK on success, error code otherwise
 */
esp_err_t led_controller_enable(bool enable);

/**
 * @brief Check if LED is enabled
 * @return True if enabled, false otherwise
 */
bool led_controller_is_enabled(void);

/**
 * @brief Check if LED is on
 * @return True if on, false otherwise
 */
bool led_controller_is_on(void);

/**
 * @brief Get LED state
 * @param state Pointer to LED state structure
 * @return ESP_OK on success, error code otherwise
 */
esp_err_t led_controller_get_state(led_state_t *state);

// System LED functions

/**
 * @brief Initialize system LED
 * @return ESP_OK on success, error code otherwise
 */
esp_err_t led_controller_init_system_led(void);

/**
 * @brief Set system status LED
 * @param status System status
 * @return ESP_OK on success, error code otherwise
 */
esp_err_t led_controller_set_system_status(uint8_t status);

/**
 * @brief Set battery status LED
 * @param battery_level Battery level (0-100)
 * @return ESP_OK on success, error code otherwise
 */
esp_err_t led_controller_set_battery_status(uint8_t battery_level);

/**
 * @brief Set WiFi status LED
 * @param connected WiFi connection status
 * @return ESP_OK on success, error code otherwise
 */
esp_err_t led_controller_set_wifi_status(bool connected);

/**
 * @brief Set error LED
 * @param error_code Error code
 * @return ESP_OK on success, error code otherwise
 */
esp_err_t led_controller_set_error_status(uint8_t error_code);

// Heartbeat and status functions

/**
 * @brief Start LED heartbeat
 * @return ESP_OK on success, error code otherwise
 */
esp_err_t led_controller_start_heartbeat(void);

/**
 * @brief Stop LED heartbeat
 * @return ESP_OK on success, error code otherwise
 */
esp_err_t led_controller_stop_heartbeat(void);

/**
 * @brief Set heartbeat pattern
 * @param pattern Heartbeat pattern array
 * @param length Pattern length
 * @return ESP_OK on success, error code otherwise
 */
esp_err_t led_controller_set_heartbeat_pattern(const uint8_t *pattern, uint8_t length);

/**
 * @brief Update heartbeat LED
 * @return ESP_OK on success, error code otherwise
 */
esp_err_t led_controller_heartbeat(void);

// Utility functions

/**
 * @brief Convert brightness percentage to PWM value
 * @param percentage Brightness percentage (0-100)
 * @return PWM value (0-255)
 */
uint8_t led_controller_percentage_to_pwm(uint8_t percentage);

/**
 * @brief Convert PWM value to brightness percentage
 * @param pwm PWM value (0-255)
 * @return Brightness percentage (0-100)
 */
uint8_t led_controller_pwm_to_percentage(uint8_t pwm);

/**
 * @brief Get LED configuration
 * @param config Pointer to LED configuration
 * @return ESP_OK on success, error code otherwise
 */
esp_err_t led_controller_get_config(led_config_t *config);

/**
 * @brief Save LED configuration to NVS
 * @return ESP_OK on success, error code otherwise
 */
esp_err_t led_controller_save_config(void);

/**
 * @brief Load LED configuration from NVS
 * @return ESP_OK on success, error code otherwise
 */
esp_err_t led_controller_load_config(void);

/**
 * @brief Reset LED configuration to defaults
 * @return ESP_OK on success, error code otherwise
 */
esp_err_t led_controller_reset_config(void);

// Default configuration values
#define DEFAULT_LED_PIN                    LED_PIN
#define DEFAULT_LED_ACTIVE_HIGH            true
#define DEFAULT_LED_BRIGHTNESS             128
#define DEFAULT_LED_MODE                   LED_MODE_OFF
#define DEFAULT_LED_COLOR                  LED_COLOR_OFF
#define DEFAULT_LED_BLINK_PERIOD_MS        500
#define DEFAULT_LED_PULSE_PERIOD_MS        1000
#define DEFAULT_LED_BREATH_PERIOD_MS       2000
#define DEFAULT_LED_PATTERN_LENGTH         8

// LED status codes
#define LED_STATUS_SYSTEM_OK               0
#define LED_STATUS_SYSTEM_ERROR            1
#define LED_STATUS_SYSTEM_WARNING          2
#define LED_STATUS_SYSTEM_INIT             3
#define LED_STATUS_SYSTEM_READY            4

// Global variables (extern declarations)
extern led_config_t led_config;
extern led_state_t led_state;

#ifdef __cplusplus
}
#endif

#endif // LED_CONTROLLER_H
