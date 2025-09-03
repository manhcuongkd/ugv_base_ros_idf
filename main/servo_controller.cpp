#include "../inc/servo_controller.h"
#include <esp_log.h>
#include <driver/uart.h>
#include <driver/gpio.h>
#include <driver/ledc.h>
#include <freertos/FreeRTOS.h>
#include <freertos/task.h>
#include <string.h>
#include <math.h>

static const char *TAG = "ServoController";

// Configuration
#define SERVO_UART_NUM UART_NUM_1
#define SERVO_UART_TX_PIN GPIO_NUM_17
#define SERVO_UART_RX_PIN GPIO_NUM_16
#define SERVO_UART_BAUD_RATE 1000000

#define SERVO_PWM_FREQ 50
#define SERVO_PWM_RESOLUTION LEDC_TIMER_13_BIT
#define SERVO_PWM_TIMER LEDC_TIMER_0

#define MAX_SERVOS 5

// SCServo protocol headers
#define SCSERVO_HEADER 0xFF
#define SCSERVO_ID 0xFE

// Global variable definition
servo_feedback_t servo_feedback[5];

// Private variables
static servo_config_t servo_configs[MAX_SERVOS];
static bool scservo_mode = false;
static bool servo_initialized = false;
static bool emergency_stop_active = false;
static servo_config_t servo_status[MAX_SERVOS];

// SCServo register definitions
#define SCS_TORQUE_ENABLE 0x40

// Private function prototypes
static esp_err_t servo_init_uart(void);
static esp_err_t servo_init_pwm(void);
static esp_err_t scservo_send_command(uint8_t servo_id, uint8_t cmd, uint8_t *data, uint8_t data_len);
static esp_err_t scservo_read_response(uint8_t *buffer, size_t buffer_size);
static uint16_t servo_angle_to_pulse(uint16_t angle);
static esp_err_t servo_set_pwm_duty(uint8_t servo_id, uint16_t pulse_width);
static esp_err_t scs_servo_write_word(uint8_t servo_id, uint8_t reg, uint16_t value);

esp_err_t servo_controller_init(void) {
    ESP_LOGI(TAG, "Initializing servo controller...");

    // Initialize servo configs
    for (int i = 0; i < MAX_SERVOS; i++) {
        servo_configs[i].id = i + 11; // Servos 11-15
        servo_configs[i].min_position = 0;
        servo_configs[i].max_position = 4095;
        servo_configs[i].max_speed = 100;
        servo_configs[i].max_acceleration = 50;
        servo_configs[i].max_torque = 100;
        servo_configs[i].min_voltage = 6000; // 6V
        servo_configs[i].max_voltage = 12000; // 12V
        servo_configs[i].max_temperature = 800; // 80°C
        servo_configs[i].enable_torque = false;
        servo_configs[i].enable_led = false;
        servo_configs[i].response_time = 10;
    }

    // Initialize servo feedback array
    memset(servo_feedback, 0, sizeof(servo_feedback));
    for (int i = 0; i < MAX_SERVOS; i++) {
        servo_feedback[i].id = i + 11; // Servos 11-15
        servo_feedback[i].position = 2048; // Center position
        servo_feedback[i].target_position = 2048;
        servo_feedback[i].speed = 100;
        servo_feedback[i].load = 0;
        servo_feedback[i].voltage = 12000; // 12V
        servo_feedback[i].temperature = 250; // 25°C
        servo_feedback[i].status = SERVO_STATUS_OK;
        servo_feedback[i].moving = false;
        servo_feedback[i].error = false;
        servo_feedback[i].timestamp = xTaskGetTickCount() * portTICK_PERIOD_MS;
    }

    // Try to initialize SCServo (UART) first
    if (servo_init_uart() == ESP_OK) {
        scservo_mode = true;
        ESP_LOGI(TAG, "Servo controller initialized in SCServo (UART) mode");
        return ESP_OK;
    }

    // Fall back to PWM mode
    if (servo_init_pwm() == ESP_OK) {
        scservo_mode = false;
        ESP_LOGI(TAG, "Servo controller initialized in PWM mode");
        return ESP_OK;
    }

    ESP_LOGE(TAG, "Failed to initialize servo controller in both modes");
    return ESP_FAIL;
}

esp_err_t servo_controller_set_position(uint8_t servo_id, uint16_t position) {
    if (servo_id < 11 || servo_id > 15) {
        return ESP_ERR_INVALID_ARG;
    }

    uint8_t array_index = servo_id - 11;
    
    // Clamp position to valid range
    uint16_t clamped_position = fmaxf(servo_configs[array_index].min_position, 
                                     fminf(servo_configs[array_index].max_position, position));

    if (scservo_mode) {
        // SCServo mode - send UART command
        uint8_t data[2];
        data[0] = clamped_position & 0xFF;
        data[1] = (clamped_position >> 8) & 0xFF;
        
        esp_err_t ret = scservo_send_command(servo_id, 0x01, data, 2); // Write position command
        if (ret == ESP_OK) {
            servo_feedback[array_index].target_position = clamped_position;
            servo_feedback[array_index].position = clamped_position;
        }
        return ret;
    } else {
        // PWM mode - set PWM duty cycle
        float angle = servo_controller_position_to_angle(clamped_position);
        uint16_t pulse_width = servo_angle_to_pulse(angle);
        esp_err_t ret = servo_set_pwm_duty(array_index, pulse_width);
        if (ret == ESP_OK) {
            servo_feedback[array_index].target_position = clamped_position;
            servo_feedback[array_index].position = clamped_position;
        }
        return ret;
    }
}

esp_err_t servo_controller_get_position(uint8_t servo_id, uint16_t *position) {
    if (servo_id < 11 || servo_id > 15 || position == NULL) {
        return ESP_ERR_INVALID_ARG;
    }

    uint8_t array_index = servo_id - 11;

    if (scservo_mode) {
        // SCServo mode - read position from servo
        uint8_t data[2];
        esp_err_t ret = scservo_send_command(servo_id, 0x28, NULL, 0); // Read position command
        if (ret == ESP_OK) {
            ret = scservo_read_response(data, 2);
            if (ret == ESP_OK) {
                uint16_t pos = (data[1] << 8) | data[0];
                *position = pos;
                servo_feedback[array_index].position = pos;
            }
        }
        return ret;
    } else {
        // PWM mode - return last known position
        *position = servo_feedback[array_index].position;
        return ESP_OK;
    }
}

esp_err_t servo_controller_set_speed(uint8_t servo_id, uint16_t speed) {
    if (servo_id < 11 || servo_id > 15) {
        return ESP_ERR_INVALID_ARG;
    }

    uint8_t array_index = servo_id - 11;

    if (scservo_mode) {
        // SCServo mode - send speed command
        uint8_t data[2];
        data[0] = speed & 0xFF;
        data[1] = (speed >> 8) & 0xFF;
        
        esp_err_t ret = scservo_send_command(servo_id, 0x20, data, 2); // Write speed command
        if (ret == ESP_OK) {
            servo_feedback[array_index].speed = speed;
        }
        return ret;
    } else {
        // PWM mode - log warning
        ESP_LOGW(TAG, "Speed control not supported in PWM mode for servo %d", servo_id);
        return ESP_ERR_NOT_SUPPORTED;
    }
}

esp_err_t servo_controller_enable_torque(uint8_t servo_id, bool enable) {
    if (servo_id < 11 || servo_id > 15) {
        return ESP_ERR_INVALID_ARG;
    }

    uint8_t array_index = servo_id - 11;

    if (scservo_mode) {
        // SCServo mode - send torque command
        uint8_t data[1];
        data[0] = enable ? 0x01 : 0x00;
        
        esp_err_t ret = scservo_send_command(servo_id, 0x18, data, 1); // Write torque command
        if (ret == ESP_OK) {
            servo_configs[array_index].enable_torque = enable;
        }
        return ret;
    } else {
        // PWM mode - log warning
        ESP_LOGW(TAG, "Torque control not supported in PWM mode for servo %d", servo_id);
        return ESP_ERR_NOT_SUPPORTED;
    }
}

esp_err_t servo_controller_get_feedback(uint8_t servo_id, servo_feedback_t *feedback) {
    if (servo_id < 11 || servo_id > 15 || feedback == NULL) {
        return ESP_ERR_INVALID_ARG;
    }

    uint8_t array_index = servo_id - 11;
    memcpy(feedback, &servo_feedback[array_index], sizeof(servo_feedback_t));
    return ESP_OK;
}

esp_err_t servo_controller_set_config(uint8_t servo_id, const servo_config_t *config) {
    if (servo_id < 11 || servo_id > 15 || config == NULL) {
        return ESP_ERR_INVALID_ARG;
    }

    uint8_t array_index = servo_id - 11;
    memcpy(&servo_configs[array_index], config, sizeof(servo_config_t));
    ESP_LOGI(TAG, "Servo %d configuration updated", servo_id);
    return ESP_OK;
}

esp_err_t servo_controller_reset_emergency(void)
{
    if (!servo_initialized) {
        ESP_LOGE(TAG, "Servo controller not initialized");
        return ESP_ERR_INVALID_STATE;
    }

    emergency_stop_active = false;
    ESP_LOGI(TAG, "Emergency stop reset");
    return ESP_OK;
}

esp_err_t servo_controller_disable_all(void)
{
    if (!servo_initialized) {
        ESP_LOGE(TAG, "Servo controller not initialized");
        return ESP_ERR_INVALID_STATE;
    }

    ESP_LOGI(TAG, "Disabling all servos...");
    
    // Disable all servos by setting torque to 0
    for (int i = 0; i < MAX_SERVOS; i++) {
        if (servo_status[i].enable_torque) {
            esp_err_t ret = scs_servo_write_word(i + 1, SCS_TORQUE_ENABLE, 0);
            if (ret != ESP_OK) {
                ESP_LOGW(TAG, "Failed to disable servo %d: %s", i + 1, esp_err_to_name(ret));
            } else {
                servo_status[i].enable_torque = false;
                ESP_LOGI(TAG, "Servo %d disabled", i + 1);
            }
        }
    }
    
    ESP_LOGI(TAG, "All servos disabled");
    return ESP_OK;
}

// Private functions
static esp_err_t servo_init_uart(void) {
    uart_config_t uart_config = {
        .baud_rate = SERVO_UART_BAUD_RATE,
        .data_bits = UART_DATA_8_BITS,
        .parity = UART_PARITY_DISABLE,
        .stop_bits = UART_STOP_BITS_1,
        .flow_ctrl = UART_HW_FLOWCTRL_DISABLE,
        .rx_flow_ctrl_thresh = 0,
        .source_clk = UART_SCLK_APB,
    };

    esp_err_t ret = uart_driver_install(SERVO_UART_NUM, 1024, 1024, 0, NULL, 0);
    if (ret != ESP_OK) {
        ESP_LOGE(TAG, "Failed to install UART driver: %s", esp_err_to_name(ret));
        return ret;
    }

    ret = uart_param_config(SERVO_UART_NUM, &uart_config);
    if (ret != ESP_OK) {
        ESP_LOGE(TAG, "Failed to configure UART: %s", esp_err_to_name(ret));
        return ret;
    }

    ret = uart_set_pin(SERVO_UART_NUM, SERVO_UART_TX_PIN, SERVO_UART_RX_PIN, UART_PIN_NO_CHANGE, UART_PIN_NO_CHANGE);
    if (ret != ESP_OK) {
        ESP_LOGE(TAG, "Failed to set UART pins: %s", esp_err_to_name(ret));
        return ret;
    }

    ESP_LOGI(TAG, "UART initialized for SCServo communication");
    return ESP_OK;
}

static esp_err_t servo_init_pwm(void) {
    // Configure LEDC timer
    ledc_timer_config_t ledc_timer = {
        .speed_mode = LEDC_LOW_SPEED_MODE,
        .duty_resolution = SERVO_PWM_RESOLUTION,
        .timer_num = SERVO_PWM_TIMER,
        .freq_hz = SERVO_PWM_FREQ,
        .clk_cfg = LEDC_AUTO_CLK,
    };

    esp_err_t ret = ledc_timer_config(&ledc_timer);
    if (ret != ESP_OK) {
        ESP_LOGE(TAG, "Failed to configure LEDC timer: %s", esp_err_to_name(ret));
        return ret;
    }

    // Configure LEDC channels for each servo
    for (int i = 0; i < MAX_SERVOS; i++) {
        ledc_channel_config_t ledc_channel = {
            .gpio_num = GPIO_NUM_2 + i, // Use GPIO 2-6 for servos
            .speed_mode = LEDC_LOW_SPEED_MODE,
            .channel = (ledc_channel_t)i,
            .intr_type = LEDC_INTR_DISABLE,
            .timer_sel = SERVO_PWM_TIMER,
            .duty = 0,
            .hpoint = 0,
            .flags = {0},
        };

        ret = ledc_channel_config(&ledc_channel);
        if (ret != ESP_OK) {
            ESP_LOGE(TAG, "Failed to configure LEDC channel %d: %s", i, esp_err_to_name(ret));
            return ret;
        }
    }

    ESP_LOGI(TAG, "PWM initialized for servo control");
    return ESP_OK;
}

static esp_err_t scservo_send_command(uint8_t servo_id, uint8_t cmd, uint8_t *data, uint8_t data_len) {
    uint8_t buffer[32];
    uint8_t checksum = 0;
    int index = 0;

    // Header
    buffer[index++] = SCSERVO_HEADER;
    buffer[index++] = SCSERVO_HEADER;
    
    // ID
    buffer[index++] = servo_id;
    checksum += servo_id;
    
    // Length
    buffer[index++] = data_len + 3; // ID + CMD + DATA + CHECKSUM
    checksum += buffer[index - 1];
    
    // Command
    buffer[index++] = cmd;
    checksum += cmd;
    
    // Data
    if (data && data_len > 0) {
        memcpy(&buffer[index], data, data_len);
        for (int i = 0; i < data_len; i++) {
            checksum += data[i];
        }
        index += data_len;
    }
    
    // Checksum
    buffer[index++] = ~checksum;
    
    // Send command
    int written = uart_write_bytes(SERVO_UART_NUM, buffer, index);
    if (written != index) {
        ESP_LOGE(TAG, "Failed to send SCServo command");
        return ESP_FAIL;
    }
    
    return ESP_OK;
}

static esp_err_t scservo_read_response(uint8_t *buffer, size_t buffer_size) {
    int len = uart_read_bytes(SERVO_UART_NUM, buffer, buffer_size, pdMS_TO_TICKS(100));
    if (len <= 0) {
        ESP_LOGE(TAG, "Failed to read SCServo response");
        return ESP_FAIL;
    }
    
    return ESP_OK;
}

static uint16_t servo_angle_to_pulse(uint16_t angle) {
    // Convert angle (0-180) to pulse width (500-2500 microseconds)
    // Then convert to LEDC duty cycle
    uint16_t pulse_us = 500 + (angle * 2000) / 180;
    uint32_t max_duty = (1 << SERVO_PWM_RESOLUTION) - 1;
    uint32_t duty = (pulse_us * max_duty) / (1000000 / SERVO_PWM_FREQ);
    
    return (uint16_t)duty;
}

static esp_err_t servo_set_pwm_duty(uint8_t servo_id, uint16_t pulse_width) {
    if (servo_id >= MAX_SERVOS) {
        return ESP_ERR_INVALID_ARG;
    }
    
    // Convert pulse width to duty cycle
    uint32_t duty = (pulse_width * ((1 << SERVO_PWM_RESOLUTION) - 1)) / 8192; // 8192 = 2^13
    
    esp_err_t ret = ledc_set_duty(LEDC_LOW_SPEED_MODE, (ledc_channel_t)servo_id, duty);
    if (ret != ESP_OK) {
        ESP_LOGE(TAG, "Failed to set PWM duty for servo %d: %s", servo_id, esp_err_to_name(ret));
        return ret;
    }
    
    return ESP_OK;
}

static esp_err_t scs_servo_write_word(uint8_t servo_id, uint8_t reg, uint16_t value) {
    uint8_t data[3];
    data[0] = reg;
    data[1] = (value >> 8) & 0xFF; // MSB
    data[2] = value & 0xFF;         // LSB
    
    return scservo_send_command(servo_id, 0x03, data, 3); // Write command
}
