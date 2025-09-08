#include "../inc/led_controller.h"
#include <esp_log.h>
#include <driver/gpio.h>
#include <driver/ledc.h>
#include <freertos/FreeRTOS.h>
#include <freertos/task.h>
#include <math.h>
#include <cstring>

static const char *TAG = "LEDController";

// LED pins - Arduino compatible
#define LED_IO4_PIN GPIO_NUM_4
#define LED_IO5_PIN GPIO_NUM_5

// RGB LED pins (for advanced features)
#define LED_RED_PIN GPIO_NUM_25
#define LED_GREEN_PIN GPIO_NUM_26
#define LED_BLUE_PIN GPIO_NUM_27

// PWM configuration - Arduino compatible
#define LED_PWM_FREQ_HZ 200  // Match Arduino FREQ
#define LED_PWM_RESOLUTION LEDC_TIMER_8_BIT  // Match Arduino ANALOG_WRITE_BITS
#define LED_PWM_TIMER LEDC_TIMER_1

// Arduino compatibility mode (currently always enabled)
// static bool arduino_compat_mode = true;

// Global variable definition
led_state_t led_state;

// Private variables
static uint32_t heartbeat_counter = 0;

// Private function prototypes
static void led_set_pwm_duty(uint8_t channel, uint32_t duty);
static void led_update_color(void);

esp_err_t led_controller_init(void) {
    ESP_LOGI(TAG, "Initializing LED controller...");

    // Configure GPIO pins - Arduino compatible
    gpio_config_t io_conf = {
        .pin_bit_mask = (1ULL << LED_IO4_PIN) | (1ULL << LED_IO5_PIN),
        .mode = GPIO_MODE_OUTPUT,
        .pull_up_en = GPIO_PULLUP_DISABLE,
        .pull_down_en = GPIO_PULLDOWN_DISABLE,
        .intr_type = GPIO_INTR_DISABLE,
    };

    esp_err_t ret = gpio_config(&io_conf);
    if (ret != ESP_OK) {
        ESP_LOGE(TAG, "Failed to configure GPIO: %s", esp_err_to_name(ret));
        return ret;
    }

    // Configure LEDC timer
    ledc_timer_config_t ledc_timer = {
        .speed_mode = LEDC_LOW_SPEED_MODE,
        .duty_resolution = LED_PWM_RESOLUTION,
        .timer_num = LED_PWM_TIMER,
        .freq_hz = LED_PWM_FREQ_HZ,
        .clk_cfg = LEDC_AUTO_CLK,
    };

    ret = ledc_timer_config(&ledc_timer);
    if (ret != ESP_OK) {
        ESP_LOGE(TAG, "Failed to configure LEDC timer: %s", esp_err_to_name(ret));
        return ret;
    }

    // Configure LEDC channels for Arduino-compatible LEDs
    ledc_channel_config_t ledc_channel_io4 = {
        .gpio_num = LED_IO4_PIN,
        .speed_mode = LEDC_LOW_SPEED_MODE,
        .channel = LEDC_CHANNEL_0,
        .intr_type = LEDC_INTR_DISABLE,
        .timer_sel = LED_PWM_TIMER,
        .duty = 0,
        .hpoint = 0,
        .flags = {0},
    };

    ledc_channel_config_t ledc_channel_io5 = {
        .gpio_num = LED_IO5_PIN,
        .speed_mode = LEDC_LOW_SPEED_MODE,
        .channel = LEDC_CHANNEL_1,
        .intr_type = LEDC_INTR_DISABLE,
        .timer_sel = LED_PWM_TIMER,
        .duty = 0,
        .hpoint = 0,
        .flags = {0},
    };

    ret = ledc_channel_config(&ledc_channel_io4);
    if (ret != ESP_OK) {
        ESP_LOGE(TAG, "Failed to configure IO4 LED channel: %s", esp_err_to_name(ret));
        return ret;
    }

    ret = ledc_channel_config(&ledc_channel_io5);
    if (ret != ESP_OK) {
        ESP_LOGE(TAG, "Failed to configure IO5 LED channel: %s", esp_err_to_name(ret));
        return ret;
    }

    // Initialize global led_state
    led_state.enabled = true;
    led_state.on = false;
    led_state.brightness = 128;
    led_state.current_mode = LED_MODE_OFF;
    led_state.current_color = LED_COLOR_OFF;
    led_state.last_update = xTaskGetTickCount() * portTICK_PERIOD_MS;
    led_state.pattern_index = 0;
    led_state.initialized = true;

    ESP_LOGI(TAG, "LED controller initialized successfully");
    return ESP_OK;
}

esp_err_t led_controller_heartbeat(void) {
    if (!led_state.enabled) {
        return ESP_OK;
    }

    heartbeat_counter++;

    switch (led_state.current_mode) {
        case LED_MODE_BREATH: {
            // Breathing pattern: fade in/out
            uint32_t duty = (uint32_t)((sinf(heartbeat_counter * 0.1f) + 1.0f) * 0.5f * led_state.brightness);
            led_set_pwm_duty(LEDC_CHANNEL_0, duty); // IO4
            led_set_pwm_duty(LEDC_CHANNEL_1, duty); // IO5
            break;
        }
        
        case LED_MODE_BLINK: {
            // Blink pattern: on/off every 500ms
            bool led_on = (heartbeat_counter / 50) % 2;
            uint32_t duty = led_on ? led_state.brightness : 0;
            led_set_pwm_duty(LEDC_CHANNEL_0, duty); // IO4
            led_set_pwm_duty(LEDC_CHANNEL_1, duty); // IO5
            break;
        }
        
        case LED_MODE_PULSE: {
            // Pulse pattern: smooth fade
            uint32_t duty = (uint32_t)((sinf(heartbeat_counter * 0.05f) + 1.0f) * 0.5f * led_state.brightness);
            led_set_pwm_duty(LEDC_CHANNEL_0, duty); // IO4
            led_set_pwm_duty(LEDC_CHANNEL_1, duty); // IO5
            break;
        }
        
        case LED_MODE_ON:
        case LED_MODE_OFF:
        case LED_MODE_RAINBOW:
        case LED_MODE_CUSTOM:
        default:
            // Other modes - update color
            led_update_color();
            break;
    }

    return ESP_OK;
}

esp_err_t led_controller_set_mode(led_mode_t mode) {
    if (mode > LED_MODE_CUSTOM) {
        return ESP_ERR_INVALID_ARG;
    }

    led_state.current_mode = mode;
    ESP_LOGI(TAG, "LED mode set to: %d", mode);

    // Update LEDs immediately
    led_update_color();
    return ESP_OK;
}

esp_err_t led_controller_set_color(led_color_t color) {
    if (color > LED_COLOR_CUSTOM) {
        return ESP_ERR_INVALID_ARG;
    }

    led_state.current_color = color;
    ESP_LOGI(TAG, "LED color set to: %d", color);

    // Update LEDs immediately
    led_update_color();
    return ESP_OK;
}

esp_err_t led_controller_set_brightness(uint8_t brightness) {
    // uint8_t is 0-255, so no need to check upper bound
    led_state.brightness = brightness;
    ESP_LOGI(TAG, "LED brightness set to: %d", brightness);

    // Update LEDs immediately
    led_update_color();
    return ESP_OK;
}

esp_err_t led_controller_turn_on(void) {
    led_state.enabled = true;
    led_state.on = true;
    ESP_LOGI(TAG, "LEDs turned on");
    led_update_color();
    return ESP_OK;
}

esp_err_t led_controller_turn_off(void) {
    led_state.enabled = false;
    led_state.on = false;
    ESP_LOGI(TAG, "LEDs turned off");
    
    // Turn off all LEDs
    led_set_pwm_duty(LEDC_CHANNEL_0, 0);  // IO4
    led_set_pwm_duty(LEDC_CHANNEL_1, 0);  // IO5
    
    return ESP_OK;
}

esp_err_t led_controller_get_status(led_state_t *status) {
    if (status == NULL) {
        return ESP_ERR_INVALID_ARG;
    }

    memcpy(status, &led_state, sizeof(led_state_t));
    return ESP_OK;
}

// Private functions
static void led_set_pwm_duty(uint8_t channel, uint32_t duty) {
    if (channel >= LEDC_CHANNEL_MAX) {
        return;
    }

    // Arduino compatible: 8-bit duty (0-255) maps directly to 8-bit LEDC
    uint32_t max_duty = (1 << LED_PWM_RESOLUTION) - 1;  // 255 for 8-bit
    uint32_t ledc_duty = (duty * max_duty) / 255;
    
    ledc_set_duty(LEDC_LOW_SPEED_MODE, (ledc_channel_t)channel, ledc_duty);
}

static void led_update_color(void) {
    if (!led_state.enabled || !led_state.on) {
        led_set_pwm_duty(LEDC_CHANNEL_0, 0);  // IO4
        led_set_pwm_duty(LEDC_CHANNEL_1, 0);  // IO5
        return;
    }

    uint32_t io4_duty = 0, io5_duty = 0;

    // Arduino compatible: simple on/off or brightness control
    if (led_state.current_mode == LED_MODE_ON) {
        io4_duty = led_state.brightness;
        io5_duty = led_state.brightness;
    } else if (led_state.current_mode == LED_MODE_OFF) {
        io4_duty = 0;
        io5_duty = 0;
    } else {
        // For other modes, use brightness as base
        io4_duty = led_state.brightness;
        io5_duty = led_state.brightness;
    }

    led_set_pwm_duty(LEDC_CHANNEL_0, io4_duty);  // IO4
    led_set_pwm_duty(LEDC_CHANNEL_1, io5_duty);  // IO5
}

esp_err_t led_controller_set_rgb(uint8_t red, uint8_t green, uint8_t blue) {
    ESP_LOGI(TAG, "Setting Arduino LED: IO4=%d, IO5=%d", red, green);
    
    // Arduino compatible: use red as IO4, green as IO5, ignore blue
    led_set_pwm_duty(LEDC_CHANNEL_0, red);   // IO4
    led_set_pwm_duty(LEDC_CHANNEL_1, green); // IO5
    
    return ESP_OK;
}

// Arduino compatibility functions

/**
 * @brief Initialize LED pins (Arduino compatible)
 * @return ESP_OK on success, error code otherwise
 */
esp_err_t led_pin_init(void) {
    ESP_LOGI(TAG, "Initializing Arduino-compatible LED pins...");
    
    // Use the main LED controller initialization
    esp_err_t ret = led_controller_init();
    if (ret != ESP_OK) {
        ESP_LOGE(TAG, "Failed to initialize LED controller: %s", esp_err_to_name(ret));
        return ret;
    }
    
    ESP_LOGI(TAG, "Arduino-compatible LED pins initialized successfully");
    return ESP_OK;
}

/**
 * @brief Control LED PWM (Arduino compatible)
 * @param io4_input IO4 pin PWM value (0-255)
 * @param io5_input IO5 pin PWM value (0-255)
 * @return ESP_OK on success, error code otherwise
 */
esp_err_t led_pwm_ctrl(int io4_input, int io5_input) {
    // Constrain values to 0-255 range (Arduino compatible)
    int io4_constrained = (io4_input < 0) ? 0 : (io4_input > 255) ? 255 : io4_input;
    int io5_constrained = (io5_input < 0) ? 0 : (io5_input > 255) ? 255 : io5_input;
    
    ESP_LOGI(TAG, "Arduino LED control: IO4=%d, IO5=%d", io4_constrained, io5_constrained);
    
    // Set PWM duty cycles directly
    led_set_pwm_duty(LEDC_CHANNEL_0, (uint32_t)io4_constrained);  // IO4
    led_set_pwm_duty(LEDC_CHANNEL_1, (uint32_t)io5_constrained);  // IO5
    
    return ESP_OK;
}
