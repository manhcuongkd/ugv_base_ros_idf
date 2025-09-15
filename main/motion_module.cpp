#include "../inc/motion_module.h"
#include "../inc/oled_controller.h"
#include <esp_log.h>
#include <inttypes.h>
#include <driver/gpio.h>
#include <driver/ledc.h>
#include <driver/pcnt.h>
#include <soc/pcnt_struct.h>
#include <rom/gpio.h>
#include <nvs_flash.h>
#include <nvs.h>
#include <freertos/FreeRTOS.h>
#include <freertos/task.h>
#include <math.h>
#include <cstring>
#include <esp_timer.h>

static const char *TAG = "MotionModule";

// Global variables
static bool motion_initialized = false;
static float left_speed_rate = 1.0f;   // Left motor speed rate
static float right_speed_rate = 1.0f;  // Right motor speed rate
static bool pid_compute_enabled = true;

// Runtime configurable motion parameters (matching Arduino mm_settings)
static double wheel_diameter = WHEEL_D;           // Wheel diameter in meters
static int32_t one_circle_pulses = ONE_CIRCLE_PLUSES;  // Encoder pulses per wheel revolution
static double track_width = TRACK_WIDTH;          // Distance between wheels in meters
static bool motor_direction = SET_MOTOR_DIR;      // Motor direction flag

// Encoder variables
static int32_t last_left_encoder_count = 0;
static int32_t last_right_encoder_count = 0;
static double left_wheel_speed = 0.0;
static double right_wheel_speed = 0.0;
static double encoder_pulse_rate = M_PI * WHEEL_D / ONE_CIRCLE_PLUSES;
// Debug: encoder_pulse_rate = 3.14159 * 0.0800 / 660 = 0.000380398 m/pulse
static uint64_t last_left_speed_time = 0;
static uint64_t last_right_speed_time = 0;

// PID variables
static double left_pid_output = 0.0;
static double right_pid_output = 0.0;
static double left_setpoint = 0.0;
static double right_setpoint = 0.0;

// PID controller structures
static pid_controller_t left_pid_controller;
static pid_controller_t right_pid_controller;
static encoder_data_t current_encoder_data;
static motion_control_t current_motion_status;

// PCNT units for hardware encoder counting (legacy API like ESP32Encoder)
static pcnt_unit_t left_encoder_unit = PCNT_UNIT_0;
static pcnt_unit_t right_encoder_unit = PCNT_UNIT_1;
static volatile int64_t left_encoder_count = 0;
static volatile int64_t right_encoder_count = 0;

// Emergency stop flag
static bool emergency_stop_flag = false;

// Private function prototypes (matching Arduino)
static esp_err_t init_pwm_channels(void);
static esp_err_t init_encoder_pcnt(void);
static void update_encoder_counts_from_pcnt(void);
static void update_encoder_data(void);
static float compute_pid(pid_controller_t *pid, float setpoint, float input);
static void debug_encoder_status(void);

// Motor control functions
static void motor_left_control(float pwm_input);
static void motor_right_control(float pwm_input);
static void calculate_left_wheel_speed(void);
static void calculate_right_wheel_speed(void);
static void compute_left_pid_controller(void);
static void compute_right_pid_controller(void);

esp_err_t motion_module_init(void)
{
    ESP_LOGI(TAG, "Initializing motion module...");
    
    // Initialize GPIO pins for motor control
    gpio_config_t io_conf = {};
    io_conf.mode = GPIO_MODE_OUTPUT;
    io_conf.pin_bit_mask = (1ULL << AIN1) | (1ULL << AIN2) | (1ULL << BIN1) | (1ULL << BIN2);
    io_conf.pull_down_en = GPIO_PULLDOWN_DISABLE;
    io_conf.pull_up_en = GPIO_PULLUP_DISABLE;
    io_conf.intr_type = GPIO_INTR_DISABLE;
    ESP_ERROR_CHECK(gpio_config(&io_conf));
    
    // Set initial pin states
    gpio_set_level(AIN1, 0);
    gpio_set_level(AIN2, 0);
    gpio_set_level(BIN1, 0);
    gpio_set_level(BIN2, 0);
    
    // Initialize PWM channels
    ESP_ERROR_CHECK(init_pwm_channels());
    
    // Initialize PCNT encoders (hardware-based like Arduino ESP32Encoder)
    ESP_ERROR_CHECK(init_encoder_pcnt());
    
    // Initialize motion status
    memset(&current_motion_status, 0, sizeof(motion_control_t));
    memset(&current_encoder_data, 0, sizeof(encoder_data_t));
    
    // Initialize PID controllers
    memset(&left_pid_controller, 0, sizeof(pid_controller_t));
    memset(&right_pid_controller, 0, sizeof(pid_controller_t));
    
    // Initialize encoder data
    current_encoder_data.left_count = 0;
    current_encoder_data.right_count = 0;
    current_encoder_data.left_speed = 0.0f;
    current_encoder_data.right_speed = 0.0f;
    
    motion_initialized = true;
    
    // Load saved configuration from NVS (this will also update OLED display)
    esp_err_t config_ret = motion_module_load_config_from_nvs();
    if (config_ret != ESP_OK) {
        ESP_LOGW(TAG, "Failed to load configuration from NVS: %s", esp_err_to_name(config_ret));
    }
    
    ESP_LOGI(TAG, "Motion module initialized successfully");
    
    return ESP_OK;
}

esp_err_t motion_module_init_encoders(void)
{
    if (!motion_initialized) {
        ESP_LOGE(TAG, "Motion module not initialized");
        return ESP_ERR_INVALID_STATE;
    }
    
    ESP_LOGI(TAG, "Initializing encoders...");
    
    // Reset encoder counts
    current_encoder_data.left_count = 0;
    current_encoder_data.right_count = 0;
    
    ESP_LOGI(TAG, "Encoders initialized");
    return ESP_OK;
}

esp_err_t motion_module_init_pid(void)
{
    if (!motion_initialized) {
        ESP_LOGE(TAG, "Motion module not initialized");
        return ESP_ERR_INVALID_STATE;
    }
    
    ESP_LOGI(TAG, "Initializing PID controllers...");
    
    // Configure left motor PID
    left_pid_controller.kp = 200.0f;
    left_pid_controller.ki = 2500.0f;
    left_pid_controller.kd = 0.0f;
    left_pid_controller.output_limit = PID_OUTPUT_LIMIT;
    left_pid_controller.enabled = true;
    
    // Configure right motor PID
    right_pid_controller.kp = 200.0f;
    right_pid_controller.ki = 2500.0f;
    right_pid_controller.kd = 0.0f;
    right_pid_controller.output_limit = PID_OUTPUT_LIMIT;
    right_pid_controller.enabled = true;
    
    ESP_LOGI(TAG, "PID controllers initialized");
    return ESP_OK;
}

esp_err_t motion_module_update_encoders(void)
{
    if (!motion_initialized) {
        return ESP_ERR_INVALID_STATE;
    }
    
    // Update encoder data from PCNT hardware (real implementation with ISR)
    update_encoder_counts_from_pcnt();
    
    // Calculate speeds using sophisticated time-based differentiation
    // Real implementation with proper encoder pulse rate calculation
    update_encoder_data();
    
    return ESP_OK;
}

esp_err_t motion_module_get_encoder_data(encoder_data_t *data)
{
    if (!motion_initialized || !data) {
        return ESP_ERR_INVALID_ARG;
    }
    
    memcpy(data, &current_encoder_data, sizeof(encoder_data_t));
    return ESP_OK;
}

esp_err_t motion_module_compute_pid(void)
{
    if (!motion_initialized || !pid_compute_enabled) {
        return ESP_ERR_INVALID_STATE;
    }
    
    // Compute PID for left motor
    if (left_pid_controller.enabled) {
        left_pid_controller.output = compute_pid(&left_pid_controller, 
                                               left_setpoint, 
                                               left_wheel_speed);
        
        // Apply threshold
        if (abs(left_pid_controller.output) < PID_THRESHOLD_PWM) {
            left_pid_controller.output = 0;
        }
        
        // Stop if both setpoint and speed are zero
        if (left_setpoint == 0 && left_wheel_speed == 0) {
            left_pid_controller.output = 0;
        }
    }
    
    // Compute PID for right motor
    if (right_pid_controller.enabled) {
        right_pid_controller.output = compute_pid(&right_pid_controller, 
                                                right_setpoint, 
                                                right_wheel_speed);
        
        // Apply threshold
        if (abs(right_pid_controller.output) < PID_THRESHOLD_PWM) {
            right_pid_controller.output = 0;
        }
        
        // Stop if both setpoint and speed are zero
        if (right_setpoint == 0 && right_wheel_speed == 0) {
            right_pid_controller.output = 0;
        }
    }
    
    return ESP_OK;
}

esp_err_t motion_module_apply_motor_control(void)
{
    if (!motion_initialized || emergency_stop_flag) {
        return ESP_ERR_INVALID_STATE;
    }
    
    // Apply PID output to motors
    float left_pwm = left_pid_controller.output;
    float right_pwm = right_pid_controller.output;
    
    // Apply speed rates to PWM values
    left_pwm = left_pwm * left_speed_rate;
    right_pwm = right_pwm * right_speed_rate;
    
    // PWM smoothing for ultra-smooth motion (exponential moving average)
    static float left_pwm_filtered = 0.0f;
    static float right_pwm_filtered = 0.0f;
    
    left_pwm_filtered = left_pwm_filtered + PWM_SMOOTHING_FACTOR * (left_pwm - left_pwm_filtered);
    right_pwm_filtered = right_pwm_filtered + PWM_SMOOTHING_FACTOR * (right_pwm - right_pwm_filtered);
    
    // Limit filtered PWM values
    left_pwm_filtered = (left_pwm_filtered > PWM_MAX_DUTY) ? PWM_MAX_DUTY : (left_pwm_filtered < -PWM_MAX_DUTY) ? -PWM_MAX_DUTY : left_pwm_filtered;
    right_pwm_filtered = (right_pwm_filtered > PWM_MAX_DUTY) ? PWM_MAX_DUTY : (right_pwm_filtered < -PWM_MAX_DUTY) ? -PWM_MAX_DUTY : right_pwm_filtered;
    
    // Apply smoothed PWM to motors
    motor_left_control(left_pwm_filtered);
    motor_right_control(right_pwm_filtered);
    
    return ESP_OK;
}

esp_err_t motion_module_set_speeds(float left_speed, float right_speed)
{
    if (!motion_initialized) {
        return ESP_ERR_INVALID_STATE;
    }
    
    // Limit speeds to valid range (-3.0 to 3.0)
    left_speed = (left_speed > 3.0f) ? 3.0f : (left_speed < -3.0f) ? -3.0f : left_speed;
    right_speed = (right_speed > 3.0f) ? 3.0f : (right_speed < -3.0f) ? -3.0f : right_speed;
    
    // Enable PID computation
    pid_compute_enabled = true;
    
    // Set setpoints with speed rate scaling
    left_setpoint = left_speed * left_speed_rate;
    right_setpoint = right_speed * right_speed_rate;
    
    // Update PID controller setpoints
    left_pid_controller.setpoint = left_setpoint;
    right_pid_controller.setpoint = right_setpoint;
    
    // Update motion status
    current_motion_status.left_speed = left_speed;
    current_motion_status.right_speed = right_speed;
    
    ESP_LOGI(TAG, "Set speeds: L=%.2f, R=%.2f (setpoints: L=%.2f, R=%.2f) PID: %s", 
             left_speed, right_speed, left_setpoint, right_setpoint,
             pid_compute_enabled ? "ENABLED" : "DISABLED");
    
    return ESP_OK;
}

esp_err_t motion_module_set_ros_motion(float linear_speed, float angular_speed)
{
    if (!motion_initialized) {
        return ESP_ERR_INVALID_STATE;
    }
    
    // Convert ROS-style motion to left/right wheel speeds (matching Arduino rosCtrl)
    // For differential drive: v_left = v_linear - v_angular * track_width/2
    //                        v_right = v_linear + v_angular * track_width/2
    float left_speed = linear_speed - (angular_speed * track_width / 2.0f);
    float right_speed = linear_speed + (angular_speed * track_width / 2.0f);
    
    // Set the speeds using the standard speed control
    ESP_ERROR_CHECK(motion_module_set_speeds(left_speed, right_speed));
    
    // Update motion status
    current_motion_status.linear_speed = linear_speed;
    current_motion_status.angular_speed = angular_speed;
    
    ESP_LOGI(TAG, "Set ROS motion: linear=%.2f, angular=%.2f -> L=%.2f, R=%.2f", 
             linear_speed, angular_speed, left_speed, right_speed);
    
    return ESP_OK;
}

esp_err_t motion_module_set_ros_control(float linear_speed, float angular_speed)
{
    // Alias for set_ros_motion
    return motion_module_set_ros_motion(linear_speed, angular_speed);
}

esp_err_t motion_module_emergency_stop(void)
{
    if (!motion_initialized) {
        return ESP_ERR_INVALID_STATE;
    }
    
    ESP_LOGW(TAG, "Emergency stop activated!");
    
    emergency_stop_flag = true;
    
    // Stop all motors
    motion_module_set_motor(0, 0, 0);
    motion_module_set_motor(1, 0, 0);
    
    // Reset PID controllers
    left_pid_controller.integral = 0.0f;
    right_pid_controller.integral = 0.0f;
    
    return ESP_OK;
}

esp_err_t motion_module_reset_emergency(void)
{
    if (!motion_initialized) {
        return ESP_ERR_INVALID_STATE;
    }
    
    ESP_LOGI(TAG, "Emergency stop reset");
    
    emergency_stop_flag = false;
    
    return ESP_OK;
}

esp_err_t motion_module_set_pid_params(pid_controller_t *left_pid, pid_controller_t *right_pid)
{
    if (!motion_initialized || !left_pid || !right_pid) {
        return ESP_ERR_INVALID_ARG;
    }
    
    // Copy PID parameters
    memcpy(&left_pid_controller, left_pid, sizeof(pid_controller_t));
    memcpy(&right_pid_controller, right_pid, sizeof(pid_controller_t));
    
    ESP_LOGI(TAG, "PID parameters updated");
    
    return ESP_OK;
}

esp_err_t motion_module_set_pid_params_direct(float kp, float ki, float kd, float limit)
{
    if (!motion_initialized) {
        return ESP_ERR_INVALID_STATE;
    }
    
    // Set PID parameters for both motors
    left_pid_controller.kp = kp;
    left_pid_controller.ki = ki;
    left_pid_controller.kd = kd;
    left_pid_controller.output_limit = limit;
    left_pid_controller.enabled = true;
    
    right_pid_controller.kp = kp;
    right_pid_controller.ki = ki;
    right_pid_controller.kd = kd;
    right_pid_controller.output_limit = limit;
    right_pid_controller.enabled = true;
    
    ESP_LOGI(TAG, "PID parameters set: Kp=%.2f, Ki=%.2f, Kd=%.2f, Limit=%.2f", kp, ki, kd, limit);
    
    return ESP_OK;
}

esp_err_t motion_module_get_status(motion_control_t *status)
{
    if (!motion_initialized || !status) {
        return ESP_ERR_INVALID_ARG;
    }
    
    memcpy(status, &current_motion_status, sizeof(motion_control_t));
    return ESP_OK;
}

esp_err_t motion_module_process_command(motion_command_t *cmd)
{
    if (!motion_initialized || !cmd) {
        return ESP_ERR_INVALID_ARG;
    }
    
    switch (cmd->type) {
        case MOTION_CMD_STOP:
            ESP_ERROR_CHECK(motion_module_set_speeds(0.0f, 0.0f));
            break;
            
        case MOTION_CMD_SPEED_CTRL:
            ESP_ERROR_CHECK(motion_module_set_speeds(cmd->left_speed, cmd->right_speed));
            break;
            
        case MOTION_CMD_ROS_CTRL:
            ESP_ERROR_CHECK(motion_module_set_ros_motion(cmd->linear_speed, cmd->angular_speed));
            break;
            
        case MOTION_CMD_EMERGENCY:
            ESP_ERROR_CHECK(motion_module_emergency_stop());
            break;
            
        default:
            ESP_LOGW(TAG, "Unknown motion command type: %d", cmd->type);
            return ESP_ERR_INVALID_ARG;
    }
    
    return ESP_OK;
}

esp_err_t motion_module_set_speed_rates(float left_rate, float right_rate)
{
    if (!motion_initialized) {
        return ESP_ERR_INVALID_STATE;
    }
    
    // Limit rates to 0.0 - 1.0
    left_speed_rate = (left_rate > 1.0f) ? 1.0f : (left_rate < 0.0f) ? 0.0f : left_rate;
    right_speed_rate = (right_rate > 1.0f) ? 1.0f : (right_rate < 0.0f) ? 0.0f : right_rate;
    
    ESP_LOGI(TAG, "Speed rates set: L=%.2f, R=%.2f", left_speed_rate, right_speed_rate);
    
    return ESP_OK;
}

esp_err_t motion_module_init_pwm(void)
{
    if (!motion_initialized) {
        return ESP_ERR_INVALID_STATE;
    }
    
    ESP_LOGI(TAG, "PWM channels already initialized");
    return ESP_OK;
}

esp_err_t motion_module_set_motor(uint8_t motor_id, int8_t direction, uint16_t pwm_value)
{
    if (!motion_initialized) {
        return ESP_ERR_INVALID_STATE;
    }
    
    if (motor_id > 1) {
        ESP_LOGE(TAG, "Invalid motor ID: %d", motor_id);
        return ESP_ERR_INVALID_ARG;
    }
    
    // Limit PWM value
    if (pwm_value > PWM_MAX_DUTY) {
        pwm_value = PWM_MAX_DUTY;
    }
    
    if (motor_id == 0) { // Left motor
        if (direction > 0) {
            gpio_set_level(AIN1, 0);
            gpio_set_level(AIN2, 1);
        } else if (direction < 0) {
            gpio_set_level(AIN1, 1);
            gpio_set_level(AIN2, 0);
        } else {
            gpio_set_level(AIN1, 0);
            gpio_set_level(AIN2, 0);
        }
        ledc_set_duty(LEDC_LOW_SPEED_MODE, LEDC_CHANNEL_0, pwm_value);
    } else { // Right motor
        if (direction > 0) {
            gpio_set_level(BIN1, 0);
            gpio_set_level(BIN2, 1);
        } else if (direction < 0) {
            gpio_set_level(BIN1, 1);
            gpio_set_level(BIN2, 0);
        } else {
            gpio_set_level(BIN1, 0);
            gpio_set_level(BIN2, 0);
        }
        ledc_set_duty(LEDC_LOW_SPEED_MODE, LEDC_CHANNEL_1, pwm_value);
    }
    
    return ESP_OK;
}

// Private function implementations

static esp_err_t init_pwm_channels(void)
{
    ESP_LOGI(TAG, "Initializing PWM channels...");
    
    // Configure LEDC timer
    ledc_timer_config_t ledc_timer = {
        .speed_mode = LEDC_LOW_SPEED_MODE,
        .duty_resolution = PWM_RESOLUTION,
        .timer_num = LEDC_TIMER_0,
        .freq_hz = PWM_FREQ,
        .clk_cfg = LEDC_AUTO_CLK,
    };
    ESP_ERROR_CHECK(ledc_timer_config(&ledc_timer));
    
    // Configure left motor PWM channel
    ledc_channel_config_t left_channel = {
        .gpio_num = PWMA,
        .speed_mode = LEDC_LOW_SPEED_MODE,
        .channel = LEDC_CHANNEL_0,
        .intr_type = LEDC_INTR_DISABLE,
        .timer_sel = LEDC_TIMER_0,
        .duty = 0,
        .hpoint = 0,
        .flags = {0},
    };
    ESP_ERROR_CHECK(ledc_channel_config(&left_channel));
    
    // Configure right motor PWM channel
    ledc_channel_config_t right_channel = {
        .gpio_num = PWMB,
        .speed_mode = LEDC_LOW_SPEED_MODE,
        .channel = LEDC_CHANNEL_1,
        .intr_type = LEDC_INTR_DISABLE,
        .timer_sel = LEDC_TIMER_0,
        .duty = 0,
        .hpoint = 0,
        .flags = {0},
    };
    ESP_ERROR_CHECK(ledc_channel_config(&right_channel));
    
    ESP_LOGI(TAG, "PWM channels initialized");
    return ESP_OK;
}

// PCNT interrupt handler (like ESP32Encoder, adapted for ESP-IDF v5.0)
static void IRAM_ATTR pcnt_intr_handler(void *arg)
{
    uint32_t intr_status = PCNT.int_st.val;
    
    // Handle left encoder (UNIT_0) events
    if (intr_status & BIT(PCNT_UNIT_0)) {
        if (PCNT.status_unit[PCNT_UNIT_0].h_lim_lat) {
            left_encoder_count = left_encoder_count + 32767;
            pcnt_counter_clear(PCNT_UNIT_0);
        }
        if (PCNT.status_unit[PCNT_UNIT_0].l_lim_lat) {
            left_encoder_count = left_encoder_count - 32767;
            pcnt_counter_clear(PCNT_UNIT_0);
        }
        PCNT.int_clr.val = BIT(PCNT_UNIT_0);
    }
    
    // Handle right encoder (UNIT_1) events  
    if (intr_status & BIT(PCNT_UNIT_1)) {
        if (PCNT.status_unit[PCNT_UNIT_1].h_lim_lat) {
            right_encoder_count = right_encoder_count + 32767;
            pcnt_counter_clear(PCNT_UNIT_1);
        }
        if (PCNT.status_unit[PCNT_UNIT_1].l_lim_lat) {
            right_encoder_count = right_encoder_count - 32767;
            pcnt_counter_clear(PCNT_UNIT_1);
        }
        PCNT.int_clr.val = BIT(PCNT_UNIT_1);
    }
}

static esp_err_t init_encoder_pcnt(void)
{
    ESP_LOGI(TAG, "Initializing PCNT encoders (legacy API like Arduino ESP32Encoder)");
    
    // Configure GPIO pins first (like ESP32Encoder does, using ESP-IDF v5.0 compatible function)
    esp_rom_gpio_pad_select_gpio(AENCA);
    esp_rom_gpio_pad_select_gpio(AENCB);
    esp_rom_gpio_pad_select_gpio(BENCA);
    esp_rom_gpio_pad_select_gpio(BENCB);
    
    gpio_set_direction(AENCA, GPIO_MODE_INPUT);
    gpio_set_direction(AENCB, GPIO_MODE_INPUT);
    gpio_set_direction(BENCA, GPIO_MODE_INPUT);
    gpio_set_direction(BENCB, GPIO_MODE_INPUT);
    
    // Enable pull-down resistors (like ESP32Encoder default)
    gpio_pulldown_en(AENCA);
    gpio_pulldown_en(AENCB);
    gpio_pulldown_en(BENCA);
    gpio_pulldown_en(BENCB);
    
    // Left encoder PCNT configuration (half-quad like Arduino)
    pcnt_config_t left_pcnt_config = {
        .pulse_gpio_num = AENCA,
        .ctrl_gpio_num = AENCB,
        .lctrl_mode = PCNT_MODE_KEEP,
        .hctrl_mode = PCNT_MODE_REVERSE,
        .pos_mode = PCNT_COUNT_DEC,
        .neg_mode = PCNT_COUNT_INC,
        .counter_h_lim = 32767,
        .counter_l_lim = -32767,
        .unit = left_encoder_unit,
        .channel = PCNT_CHANNEL_0,
    };
    ESP_ERROR_CHECK(pcnt_unit_config(&left_pcnt_config));
    
    // Right encoder PCNT configuration (half-quad like Arduino)
    pcnt_config_t right_pcnt_config = {
        .pulse_gpio_num = BENCA,
        .ctrl_gpio_num = BENCB,
        .lctrl_mode = PCNT_MODE_KEEP,
        .hctrl_mode = PCNT_MODE_REVERSE,
        .pos_mode = PCNT_COUNT_DEC,
        .neg_mode = PCNT_COUNT_INC,
        .counter_h_lim = 32767,
        .counter_l_lim = -32767,
        .unit = right_encoder_unit,
        .channel = PCNT_CHANNEL_0,
    };
    ESP_ERROR_CHECK(pcnt_unit_config(&right_pcnt_config));
    
    // Set up filters (like ESP32Encoder setFilter(250))
    ESP_ERROR_CHECK(pcnt_set_filter_value(left_encoder_unit, 250));
    ESP_ERROR_CHECK(pcnt_filter_enable(left_encoder_unit));
    ESP_ERROR_CHECK(pcnt_set_filter_value(right_encoder_unit, 250));
    ESP_ERROR_CHECK(pcnt_filter_enable(right_encoder_unit));
    
    // Enable interrupts for overflow/underflow
    ESP_ERROR_CHECK(pcnt_event_enable(left_encoder_unit, PCNT_EVT_H_LIM));
    ESP_ERROR_CHECK(pcnt_event_enable(left_encoder_unit, PCNT_EVT_L_LIM));
    ESP_ERROR_CHECK(pcnt_event_enable(right_encoder_unit, PCNT_EVT_H_LIM));
    ESP_ERROR_CHECK(pcnt_event_enable(right_encoder_unit, PCNT_EVT_L_LIM));
    
    // Install ISR service and handlers
    ESP_ERROR_CHECK(pcnt_isr_service_install(0));
    ESP_ERROR_CHECK(pcnt_isr_handler_add(left_encoder_unit, pcnt_intr_handler, NULL));
    ESP_ERROR_CHECK(pcnt_isr_handler_add(right_encoder_unit, pcnt_intr_handler, NULL));
    
    // Enable interrupts
    ESP_ERROR_CHECK(pcnt_intr_enable(left_encoder_unit));
    ESP_ERROR_CHECK(pcnt_intr_enable(right_encoder_unit));
    
    // Clear and start counters
    ESP_ERROR_CHECK(pcnt_counter_clear(left_encoder_unit));
    ESP_ERROR_CHECK(pcnt_counter_clear(right_encoder_unit));
    ESP_ERROR_CHECK(pcnt_counter_resume(left_encoder_unit));
    ESP_ERROR_CHECK(pcnt_counter_resume(right_encoder_unit));
    
    ESP_LOGI(TAG, "PCNT encoders initialized successfully (legacy API)");
    return ESP_OK;
}


// PCNT-based encoder reading functions (replaces ISR, like ESP32Encoder.getCount())
static void update_encoder_counts_from_pcnt(void)
{
    // Read counts from PCNT hardware using legacy API
    int16_t left_raw_count = 0;
    int16_t right_raw_count = 0;
    
    pcnt_get_counter_value(left_encoder_unit, &left_raw_count);
    pcnt_get_counter_value(right_encoder_unit, &right_raw_count);
    
    // Combine overflow counts with raw counts (like ESP32Encoder does)
    current_encoder_data.left_count = left_encoder_count + left_raw_count;
    current_encoder_data.right_count = right_encoder_count + right_raw_count;
}

static void update_encoder_data(void)
{
    // Calculate speed based on encoder counts and time
    static uint32_t last_update_time = 0;
    static int32_t last_left_count = 0;
    static int32_t last_right_count = 0;
    
    uint32_t current_time = esp_timer_get_time() / 1000; // Convert to ms
    
    if (last_update_time > 0) {
        float dt = (current_time - last_update_time) / 1000.0f; // Convert to seconds
        
        if (dt > 0.001f) { // Minimum 1ms time difference
            // Calculate speed in counts per second
            int32_t left_delta = current_encoder_data.left_count - last_left_count;
            int32_t right_delta = current_encoder_data.right_count - last_right_count;
            
            // Apply motor direction if needed
            if (motor_direction) {
                left_delta = -left_delta;
                right_delta = -right_delta;
            }
            
            // Convert to wheel speed (m/s)
            current_encoder_data.left_speed = (left_delta * encoder_pulse_rate) / dt;
            current_encoder_data.right_speed = (right_delta * encoder_pulse_rate) / dt;
            
            // Update last values
            last_left_count = current_encoder_data.left_count;
            last_right_count = current_encoder_data.right_count;
        }
    } else {
        // First call - initialize
        last_left_count = current_encoder_data.left_count;
        last_right_count = current_encoder_data.right_count;
    }
    
    last_update_time = current_time;
}

static float compute_pid(pid_controller_t *pid, float setpoint, float input)
{
    if (!pid || !pid->enabled) {
        return 0.0f;
    }
    
    float error = setpoint - input;
    
    // Proportional term
    float p_term = pid->kp * error;
    
    // Integral term
    pid->integral += error * (PID_SAMPLE_TIME_MS / 1000.0f);
    float i_term = pid->ki * pid->integral;
    
    // Derivative term
    float d_term = pid->kd * (error - pid->prev_error) / (PID_SAMPLE_TIME_MS / 1000.0f);
    pid->prev_error = error;
    
    // Compute output
    float output = p_term + i_term + d_term;
    
    // Apply output limit
    if (output > pid->output_limit) {
        output = pid->output_limit;
    } else if (output < -pid->output_limit) {
        output = -pid->output_limit;
    }
    
    return output;
}

esp_err_t motion_module_get_speed_rates(float *left_rate, float *right_rate) {
    if (!motion_initialized) {
        return ESP_ERR_INVALID_STATE;
    }
    
    if (!left_rate || !right_rate) {
        return ESP_ERR_INVALID_ARG;
    }
    
    *left_rate = left_speed_rate;
    *right_rate = right_speed_rate;
    
    ESP_LOGI(TAG, "Speed rates retrieved: L=%.2f, R=%.2f", *left_rate, *right_rate);
    return ESP_OK;
}

esp_err_t motion_module_save_speed_rates(void) {
    if (!motion_initialized) {
        return ESP_ERR_INVALID_STATE;
    }
    
    // Save speed rates to NVS
    nvs_handle_t nvs_handle;
    esp_err_t ret = nvs_open("motion_config", NVS_READWRITE, &nvs_handle);
    if (ret != ESP_OK) {
        ESP_LOGE(TAG, "Failed to open NVS: %s", esp_err_to_name(ret));
        return ret;
    }
    
    // Save speed rates as individual values
    ret = nvs_set_u32(nvs_handle, "left_speed_rate", *(uint32_t*)&left_speed_rate);
    if (ret == ESP_OK) {
        ret = nvs_set_u32(nvs_handle, "right_speed_rate", *(uint32_t*)&right_speed_rate);
    }
    if (ret != ESP_OK) {
        ESP_LOGE(TAG, "Failed to save speed rates: %s", esp_err_to_name(ret));
        nvs_close(nvs_handle);
        return ret;
    }
    
    nvs_close(nvs_handle);
    ESP_LOGI(TAG, "Speed rates saved to NVS");
    return ESP_OK;
}

esp_err_t motion_module_set_type(uint8_t main_type, uint8_t module_type) {
    if (!motion_initialized) {
        return ESP_ERR_INVALID_STATE;
    }
    
    ESP_LOGI(TAG, "Setting module type: main=%d, module=%d", main_type, module_type);
    
    // Configure motion parameters based on main_type (matching Arduino mm_settings)
    const char* robot_name = "Unknown";
    switch (main_type) {
        case 1: // RaspRover
            ESP_LOGI(TAG, "Configuring for RaspRover");
            robot_name = "RaspRover";
            wheel_diameter = 0.0800;
            one_circle_pulses = 2100;
            track_width = 0.125;
            motor_direction = false;
            ESP_LOGI(TAG, "RaspRover config: WHEEL_D=%.4f, ONE_CIRCLE_PLUSES=%" PRId32 ", TRACK_WIDTH=%.3f, SET_MOTOR_DIR=%s",
                     wheel_diameter, one_circle_pulses, track_width, motor_direction ? "true" : "false");
            break;
            
        case 2: // UGV Rover
            ESP_LOGI(TAG, "Configuring for UGV Rover");
            robot_name = "UGV Rover";
            wheel_diameter = 0.0800;
            one_circle_pulses = 660;
            track_width = 0.172;
            motor_direction = false;
            ESP_LOGI(TAG, "UGV Rover config: WHEEL_D=%.4f, ONE_CIRCLE_PLUSES=%" PRId32 ", TRACK_WIDTH=%.3f, SET_MOTOR_DIR=%s",
                     wheel_diameter, one_circle_pulses, track_width, motor_direction ? "true" : "false");
            break;
            
        case 3: // UGV Beast
            ESP_LOGI(TAG, "Configuring for UGV Beast");
            robot_name = "UGV Beast";
            wheel_diameter = 0.0523;
            one_circle_pulses = 1092;
            track_width = 0.141;
            motor_direction = true;
            ESP_LOGI(TAG, "UGV Beast config: WHEEL_D=%.4f, ONE_CIRCLE_PLUSES=%" PRId32 ", TRACK_WIDTH=%.3f, SET_MOTOR_DIR=%s",
                     wheel_diameter, one_circle_pulses, track_width, motor_direction ? "true" : "false");
            break;
            
        default:
            ESP_LOGW(TAG, "Unknown main_type: %d, using default configuration", main_type);
            break;
    }
    
    // Update OLED display (matching Arduino mm_settings)
    char display_text[21];
    snprintf(display_text, sizeof(display_text), "%s", robot_name);
    
    // Add module type suffix (matching Arduino logic)
    if (module_type == 0) {
        strncat(display_text, " Null", sizeof(display_text) - strlen(display_text) - 1);
    } else if (module_type == 1) {
        strncat(display_text, " Arm", sizeof(display_text) - strlen(display_text) - 1);
    } else if (module_type == 2) {
        strncat(display_text, " PT", sizeof(display_text) - strlen(display_text) - 1);
    }
    
    // Update OLED screen line 2 (matching Arduino screenLine_2)
    oled_controller_display_text(2, display_text);
    ESP_LOGI(TAG, "Updated OLED display: %s", display_text);
    
    // Update encoder pulse rate based on current configuration
    encoder_pulse_rate = M_PI * wheel_diameter / one_circle_pulses;
    ESP_LOGI(TAG, "Encoder pulse rate: %.9f m/pulse (diameter=%.4f, pulses=%" PRId32 ")", 
             encoder_pulse_rate, wheel_diameter, one_circle_pulses);
    ESP_LOGI(TAG, "Updated encoder_pulse_rate: %.6f", encoder_pulse_rate);
    
    // Store module types in NVS
    nvs_handle_t nvs_handle;
    esp_err_t ret = nvs_open("motion_config", NVS_READWRITE, &nvs_handle);
    if (ret != ESP_OK) {
        ESP_LOGE(TAG, "Failed to open NVS: %s", esp_err_to_name(ret));
        return ret;
    }
    
    ret = nvs_set_u8(nvs_handle, "main_type", main_type);
    if (ret == ESP_OK) {
        ret = nvs_set_u8(nvs_handle, "module_type", module_type);
    }
    
    if (ret != ESP_OK) {
        ESP_LOGE(TAG, "Failed to save module types: %s", esp_err_to_name(ret));
        nvs_close(nvs_handle);
        return ret;
    }
    
    ret = nvs_commit(nvs_handle);
    nvs_close(nvs_handle);
    
    if (ret != ESP_OK) {
        ESP_LOGE(TAG, "Failed to commit module types: %s", esp_err_to_name(ret));
        return ret;
    }
    
    ESP_LOGI(TAG, "Module type set successfully");
    return ESP_OK;
}

esp_err_t motion_module_get_config(double *wheel_d, int32_t *pulses, double *track_w, bool *motor_dir) {
    if (!motion_initialized) {
        return ESP_ERR_INVALID_STATE;
    }
    
    if (!wheel_d || !pulses || !track_w || !motor_dir) {
        return ESP_ERR_INVALID_ARG;
    }
    
    *wheel_d = wheel_diameter;
    *pulses = one_circle_pulses;
    *track_w = track_width;
    *motor_dir = motor_direction;
    
    ESP_LOGI(TAG, "Current config: WHEEL_D=%.4f, ONE_CIRCLE_PLUSES=%" PRId32 ", TRACK_WIDTH=%.3f, SET_MOTOR_DIR=%s",
             *wheel_d, *pulses, *track_w, *motor_dir ? "true" : "false");
    
    return ESP_OK;
}

esp_err_t motion_module_load_config_from_nvs(void) {
    if (!motion_initialized) {
        return ESP_ERR_INVALID_STATE;
    }
    
    nvs_handle_t nvs_handle;
    esp_err_t ret = nvs_open("motion_config", NVS_READONLY, &nvs_handle);
    if (ret != ESP_OK) {
        ESP_LOGW(TAG, "Failed to open NVS for config load: %s", esp_err_to_name(ret));
        return ret;
    }
    
    uint8_t main_type = 0;
    uint8_t module_type = 0;
    
    ret = nvs_get_u8(nvs_handle, "main_type", &main_type);
    if (ret == ESP_OK) {
        ret = nvs_get_u8(nvs_handle, "module_type", &module_type);
    }
    
    nvs_close(nvs_handle);
    
    if (ret == ESP_OK) {
        ESP_LOGI(TAG, "Loaded config from NVS: main_type=%d, module_type=%d", main_type, module_type);
        // Apply the loaded configuration (this will also update OLED display)
        return motion_module_set_type(main_type, module_type);
    } else {
        ESP_LOGW(TAG, "No saved configuration found, using defaults");
        // Set default display for unknown configuration
        oled_controller_display_text(2, "Unknown Config");
        return ESP_OK;
    }
}

// Motor control functions implementation


static void motor_left_control(float pwm_input) {
    int pwm_int = round(pwm_input);
    
    static int last_pwm_int = 999999; // Force first log
    if (pwm_int != last_pwm_int) {
        ESP_LOGI(TAG, "Left motor control: pwm_input=%.1f, pwm_int=%d", pwm_input, pwm_int);
        last_pwm_int = pwm_int;
    }
    
    // Handle zero PWM case
    if (abs(pwm_int) < 1) {
        ESP_LOGI(TAG, "Left motor STOP (PWM=0)");
        gpio_set_level(AIN1, 0);
        gpio_set_level(AIN2, 0);
        ledc_set_duty(LEDC_LOW_SPEED_MODE, LEDC_CHANNEL_0, 0);
        ledc_update_duty(LEDC_LOW_SPEED_MODE, LEDC_CHANNEL_0);
        return;
    }
    
    // Limit PWM to valid range
    if (pwm_int > PWM_MAX_DUTY) pwm_int = PWM_MAX_DUTY;
    if (pwm_int < -PWM_MAX_DUTY) pwm_int = -PWM_MAX_DUTY;
    
    // Direction control (same as right motor - no inversion needed)
    if (pwm_int > 0) {
        // Forward direction for left motor
        if (SET_MOTOR_DIR) {
            gpio_set_level(AIN1, 1);
            gpio_set_level(AIN2, 0);
        } else {
            // Same direction as right motor
            gpio_set_level(AIN1, 1);
            gpio_set_level(AIN2, 0);
        }
        ESP_LOGI(TAG, "Left motor: Setting PWM duty to %d (max: %d)", pwm_int, PWM_MAX_DUTY);
        ledc_set_duty(LEDC_LOW_SPEED_MODE, LEDC_CHANNEL_0, pwm_int);
    } else {
        // Reverse direction for left motor
        if (SET_MOTOR_DIR) {
            gpio_set_level(AIN1, 0);
            gpio_set_level(AIN2, 1);
        } else {
            // Same direction as right motor
            gpio_set_level(AIN1, 0);
            gpio_set_level(AIN2, 1);
        }
        ESP_LOGI(TAG, "Left motor: Setting PWM duty to %d (max: %d)", -pwm_int, PWM_MAX_DUTY);
        ledc_set_duty(LEDC_LOW_SPEED_MODE, LEDC_CHANNEL_0, -pwm_int);
    }
    
    ledc_update_duty(LEDC_LOW_SPEED_MODE, LEDC_CHANNEL_0);
}

static void motor_right_control(float pwm_input) {
    int pwm_int = round(pwm_input);
    
    static int last_pwm_int = 999999; // Force first log
    if (pwm_int != last_pwm_int) {
        ESP_LOGI(TAG, "Right motor control: pwm_input=%.1f, pwm_int=%d", pwm_input, pwm_int);
        last_pwm_int = pwm_int;
    }
    
    // Handle zero PWM case
    if (abs(pwm_int) < 1) {
        ESP_LOGI(TAG, "Right motor STOP (PWM=0)");
        gpio_set_level(BIN1, 0);
        gpio_set_level(BIN2, 0);
        ledc_set_duty(LEDC_LOW_SPEED_MODE, LEDC_CHANNEL_1, 0);
        ledc_update_duty(LEDC_LOW_SPEED_MODE, LEDC_CHANNEL_1);
        return;
    }
    
    // Arduino-style motor control logic
    if (motor_direction) { // SET_MOTOR_DIR = true
        if (pwm_int < 0) {
            gpio_set_level(BIN1, 1);  // HIGH
            gpio_set_level(BIN2, 0);  // LOW
        } else {
            gpio_set_level(BIN1, 0);  // LOW
            gpio_set_level(BIN2, 1);  // HIGH
        }
    } else { // SET_MOTOR_DIR = false
        if (pwm_int < 0) {
            gpio_set_level(BIN1, 0);  // LOW
            gpio_set_level(BIN2, 1);  // HIGH
        } else {
            gpio_set_level(BIN1, 1);  // HIGH
            gpio_set_level(BIN2, 0);  // LOW
        }
    }
    
    // Use PWM value directly (Arduino style: 8-bit, 0-255 range)
    uint32_t pwm_duty = abs(pwm_int);
    ESP_LOGI(TAG, "Right motor: Setting PWM duty to %ld (max: %d)", pwm_duty, PWM_MAX_DUTY);
    ledc_set_duty(LEDC_LOW_SPEED_MODE, LEDC_CHANNEL_1, pwm_duty);
    ledc_update_duty(LEDC_LOW_SPEED_MODE, LEDC_CHANNEL_1);
}

static void calculate_left_wheel_speed(void) {
    // Update encoder counts from PCNT hardware first
    update_encoder_counts_from_pcnt();
    
    uint64_t current_time = esp_timer_get_time();
    int32_t encoder_pulses = current_encoder_data.left_count;
    
    // Debug logging for encoder counts
    static int32_t last_logged_count = -999999;
    if (encoder_pulses != last_logged_count) {
        ESP_LOGI(TAG, "Left encoder count: %ld (changed by %ld)", encoder_pulses, encoder_pulses - last_logged_count);
        last_logged_count = encoder_pulses;
    }
    
    if (last_left_speed_time > 0) {
        double dt = (double)(current_time - last_left_speed_time) / 1000000.0; // Convert to seconds
        
        if (dt > 0.001) { // Minimum 1ms time difference
            double old_speed = left_wheel_speed;
            // Encoder direction (normal, since we inverted the motor instead)
            if (!motor_direction) {
                left_wheel_speed = (encoder_pulse_rate * (encoder_pulses - last_left_encoder_count)) / dt;
            } else {
                left_wheel_speed = (encoder_pulse_rate * (last_left_encoder_count - encoder_pulses)) / dt;
            }
            
            // Log speed calculation details
            if (abs(left_wheel_speed - old_speed) > 0.01) {
                ESP_LOGI(TAG, "Left speed calc: pulses=%ld->%ld, dt=%.3f, speed=%.3f m/s", 
                         last_left_encoder_count, encoder_pulses, dt, left_wheel_speed);
            }
        }
    }
    
    last_left_encoder_count = encoder_pulses;
    last_left_speed_time = current_time;
    
    // Debug encoder status periodically
    debug_encoder_status();
}

static void calculate_right_wheel_speed(void) {
    // No need to update PCNT again since left wheel function already did it
    uint64_t current_time = esp_timer_get_time();
    int32_t encoder_pulses = current_encoder_data.right_count;
    
    // Debug logging for encoder counts
    static int32_t last_logged_count = -999999;
    if (encoder_pulses != last_logged_count) {
        ESP_LOGI(TAG, "Right encoder count: %ld (changed by %ld)", encoder_pulses, encoder_pulses - last_logged_count);
        last_logged_count = encoder_pulses;
    }
    
    if (last_right_speed_time > 0) {
        double dt = (double)(current_time - last_right_speed_time) / 1000000.0; // Convert to seconds
        
        if (dt > 0.001) { // Minimum 1ms time difference
            double old_speed = right_wheel_speed;
            // Encoder direction (normal) with potential 2x scaling fix
            long pulse_diff = (!motor_direction) ? 
                (encoder_pulses - last_right_encoder_count) : 
                (last_right_encoder_count - encoder_pulses);
            
            // TEST: Divide by 2 to match Arduino's half-quad counting behavior
            pulse_diff = pulse_diff / 2;
            right_wheel_speed = (encoder_pulse_rate * pulse_diff) / dt;
            
            // Log speed calculation details with scaling debug
            if (abs(right_wheel_speed - old_speed) > 0.01) {
                long raw_pulse_diff = encoder_pulses - last_right_encoder_count;
                ESP_LOGI(TAG, "RIGHT SPEED DEBUG:");
                ESP_LOGI(TAG, "  Pulses: %ld->%ld (raw_diff=%ld, scaled_diff=%ld)", last_right_encoder_count, encoder_pulses, raw_pulse_diff, pulse_diff);
                ESP_LOGI(TAG, "  Time: dt=%.6f sec", dt);
                ESP_LOGI(TAG, "  Pulse rate: %.9f m/pulse", encoder_pulse_rate);
                ESP_LOGI(TAG, "  Scaled calc: (%.9f * %ld) / %.6f = %.3f m/s", encoder_pulse_rate, pulse_diff, dt, right_wheel_speed);
                ESP_LOGI(TAG, "  Final speed: %.3f m/s", right_wheel_speed);
            }
        }
    }
    
    last_right_encoder_count = encoder_pulses;
    last_right_speed_time = current_time;
}

static void debug_encoder_status(void) {
    static uint64_t last_debug_time = 0;
    uint64_t current_time = esp_timer_get_time() / 1000; // ms
    
    if (current_time - last_debug_time > 1000) { // Every 1 second
        // Force PCNT read using legacy API
        int16_t left_raw_count = 0;
        int16_t right_raw_count = 0;
        
        esp_err_t left_err = pcnt_get_counter_value(left_encoder_unit, &left_raw_count);
        esp_err_t right_err = pcnt_get_counter_value(right_encoder_unit, &right_raw_count);
        
        ESP_LOGI(TAG, "=== PCNT DEBUG (Legacy API) ===");
        ESP_LOGI(TAG, "Left PCNT: raw=%d, overflow=%lld, total=%lld, err=%s", 
                 left_raw_count, left_encoder_count, left_encoder_count + left_raw_count, esp_err_to_name(left_err));
        ESP_LOGI(TAG, "Right PCNT: raw=%d, overflow=%lld, total=%lld, err=%s", 
                 right_raw_count, right_encoder_count, right_encoder_count + right_raw_count, esp_err_to_name(right_err));
        ESP_LOGI(TAG, "Stored counts - Left: %ld, Right: %ld", 
                 current_encoder_data.left_count, current_encoder_data.right_count);
        ESP_LOGI(TAG, "Wheel speeds - Left: %.3f, Right: %.3f", 
                 left_wheel_speed, right_wheel_speed);
        last_debug_time = current_time;
    }
}

static void compute_left_pid_controller(void) {
    if (!pid_compute_enabled) {
        ESP_LOGW(TAG, "Left PID: compute disabled");
        return;
    }

    left_pid_output = compute_pid(&left_pid_controller, left_setpoint, left_wheel_speed);
    
    // Debug logging (only log when values change)
    static float last_setpoint = -999.0f;
    static float last_speed = -999.0f;
    static float last_output = -999.0f;
    
    if (left_setpoint != last_setpoint || left_wheel_speed != last_speed || left_pid_output != last_output) {
        ESP_LOGI(TAG, "Left PID: setpoint=%.3f, speed=%.3f, output=%.1f", 
                 left_setpoint, left_wheel_speed, left_pid_output);
        last_setpoint = left_setpoint;
        last_speed = left_wheel_speed;
        last_output = left_pid_output;
    }
    
    if (abs(left_pid_output) < PID_THRESHOLD_PWM) {
        left_pid_output = 0;
    }
    if (left_setpoint == 0 && left_wheel_speed == 0) {
        left_pid_output = 0;
    }
    motor_left_control(left_pid_output);
}

static void compute_right_pid_controller(void) {
    if (!pid_compute_enabled) {
        ESP_LOGW(TAG, "Right PID: compute disabled");
        return;
    }

    right_pid_output = compute_pid(&right_pid_controller, right_setpoint, right_wheel_speed);
    
    // Debug logging (only log when values change)
    static float last_setpoint = -999.0f;
    static float last_speed = -999.0f;
    static float last_output = -999.0f;
    
    if (right_setpoint != last_setpoint || right_wheel_speed != last_speed || right_pid_output != last_output) {
        ESP_LOGI(TAG, "Right PID: setpoint=%.3f, speed=%.3f, output=%.1f", 
                 right_setpoint, right_wheel_speed, right_pid_output);
        last_setpoint = right_setpoint;
        last_speed = right_wheel_speed;
        last_output = right_pid_output;
    }
    
    if (abs(right_pid_output) < PID_THRESHOLD_PWM) {
        right_pid_output = 0;
    }
    if (right_setpoint == 0 && right_wheel_speed == 0) {
        right_pid_output = 0;
    }
    motor_right_control(right_pid_output);
}

// Public functions for main loop integration
void motion_module_get_left_speed(void) {
    calculate_left_wheel_speed();
}

void motion_module_get_right_speed(void) {
    calculate_right_wheel_speed();
}

void motion_module_left_pid_compute(void) {
    compute_left_pid_controller();
}

void motion_module_right_pid_compute(void) {
    compute_right_pid_controller();
}

// Global test state variables (persistent across calls)
static bool g_test_started = false;
static uint64_t g_test_start_time = 0;
static int g_test_phase = 0; // 0=forward, 1=backward, 2=completed
static uint64_t g_last_log_time = 0;

esp_err_t motion_module_test_wheels(void)
{
    if (!motion_initialized) {
        ESP_LOGE(TAG, "Motion module not initialized");
        return ESP_ERR_INVALID_STATE;
    }

    uint64_t current_time = esp_timer_get_time() / 1000; // Convert to milliseconds
    
    // Initialize test on first call
    if (!g_test_started) {
        g_test_start_time = current_time;
        g_test_started = true;
        g_test_phase = 0;
        g_last_log_time = 0;
        ESP_LOGI(TAG, "=== STARTING SIMPLE MOTOR TEST ===");
        ESP_LOGI(TAG, "Phase 1: Moving FORWARD for 15 seconds");
        
        // Set motors to forward immediately
        motor_left_control(3000);
        motor_right_control(3000);
        return ESP_OK;
    }
    
    uint64_t elapsed_time = current_time - g_test_start_time;
    
    if (g_test_phase == 0) {
        // Forward phase: 0-15 seconds
        if (elapsed_time < 15000) {
            // Log progress every 3 seconds
            if (current_time - g_last_log_time > 3000) {
                ESP_LOGI(TAG, "Moving FORWARD - Time: %.1fs / 15s", elapsed_time / 1000.0);
                g_last_log_time = current_time;
            }
            // Keep moving forward (don't call motor functions repeatedly)
        } else {
            // Switch to backward phase
            g_test_phase = 1;
            g_test_start_time = current_time; // Reset timer for backward phase
            g_last_log_time = current_time;
            ESP_LOGI(TAG, "Phase 2: Moving BACKWARD for 15 seconds");
            motor_left_control(-3000);  // Backward
            motor_right_control(-3000); // Backward
        }
        
    } else if (g_test_phase == 1) {
        // Backward phase
        uint64_t backward_time = current_time - g_test_start_time;
        if (backward_time < 15000) {
            // Log progress every 3 seconds
            if (current_time - g_last_log_time > 3000) {
                ESP_LOGI(TAG, "Moving BACKWARD - Time: %.1fs / 15s", backward_time / 1000.0);
                g_last_log_time = current_time;
            }
            // Keep moving backward (don't call motor functions repeatedly)
        } else {
            // Test complete
            g_test_phase = 2;
            ESP_LOGI(TAG, "=== MOTOR TEST COMPLETED - STOPPING ===");
            motor_left_control(0);   // Stop
            motor_right_control(0);  // Stop
        }
        
    } else {
        // Test completed - keep motors stopped
        // Do nothing, motors already stopped
    }
    
    return ESP_OK;
}
