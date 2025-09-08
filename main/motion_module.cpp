#include "../inc/motion_module.h"
#include "../inc/oled_controller.h"
#include <esp_log.h>
#include <inttypes.h>
#include <driver/gpio.h>
#include <driver/ledc.h>
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

// Emergency stop flag
static bool emergency_stop_flag = false;

// Private function prototypes (matching Arduino)
static esp_err_t init_pwm_channels(void);
static esp_err_t init_encoder_pins(void);
static void encoder_isr_handler(void *arg);
static void update_encoder_data(void);
static float compute_pid(pid_controller_t *pid, float setpoint, float input);

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
    
    // Initialize encoders
    ESP_ERROR_CHECK(init_encoder_pins());
    
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
    
    // Update encoder data (this would be called from ISR in real implementation)
    update_encoder_data();
    
    // Calculate speeds from encoder counts
    // This is a simplified calculation - in reality you'd use time-based differentiation
    current_encoder_data.left_speed = (float)current_encoder_data.left_count * 0.001f; // m/s
    current_encoder_data.right_speed = (float)current_encoder_data.right_count * 0.001f; // m/s
    
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
    float left_pwm = left_pid_output;
    float right_pwm = right_pid_output;
    
    // Apply speed rates to PWM values
    left_pwm = left_pwm * left_speed_rate;
    right_pwm = right_pwm * right_speed_rate;
    
    // Limit PWM values
    left_pwm = (left_pwm > PWM_MAX_DUTY) ? PWM_MAX_DUTY : (left_pwm < -PWM_MAX_DUTY) ? -PWM_MAX_DUTY : left_pwm;
    right_pwm = (right_pwm > PWM_MAX_DUTY) ? PWM_MAX_DUTY : (right_pwm < -PWM_MAX_DUTY) ? -PWM_MAX_DUTY : right_pwm;
    
    // Apply to motors
    motor_left_control(left_pwm);
    motor_right_control(right_pwm);
    
    return ESP_OK;
}

esp_err_t motion_module_set_speeds(float left_speed, float right_speed)
{
    if (!motion_initialized) {
        return ESP_ERR_INVALID_STATE;
    }
    
    // Limit speeds to valid range (-2.0 to 2.0)
    left_speed = (left_speed > 2.0f) ? 2.0f : (left_speed < -2.0f) ? -2.0f : left_speed;
    right_speed = (right_speed > 2.0f) ? 2.0f : (right_speed < -2.0f) ? -2.0f : right_speed;
    
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
    
    ESP_LOGI(TAG, "Set speeds: L=%.2f, R=%.2f (setpoints: L=%.2f, R=%.2f)", 
             left_speed, right_speed, left_setpoint, right_setpoint);
    
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

static esp_err_t init_encoder_pins(void)
{
    ESP_LOGI(TAG, "Initializing encoder pins...");
    
    // Configure encoder pins as inputs with pull-up
    gpio_config_t io_conf = {};
    io_conf.mode = GPIO_MODE_INPUT;
    io_conf.pin_bit_mask = (1ULL << AENCA) | (1ULL << AENCB) |
                           (1ULL << BENCA) | (1ULL << BENCB);
    io_conf.pull_down_en = GPIO_PULLDOWN_DISABLE;
    io_conf.pull_up_en = GPIO_PULLUP_ENABLE;
    io_conf.intr_type = GPIO_INTR_ANYEDGE;
    ESP_ERROR_CHECK(gpio_config(&io_conf));
    
    // Install GPIO ISR service
    ESP_ERROR_CHECK(gpio_install_isr_service(0));
    
    // Add ISR handlers for encoder pins
    ESP_ERROR_CHECK(gpio_isr_handler_add(AENCA, encoder_isr_handler, (void*)0));
    ESP_ERROR_CHECK(gpio_isr_handler_add(AENCB, encoder_isr_handler, (void*)1));
    ESP_ERROR_CHECK(gpio_isr_handler_add(BENCA, encoder_isr_handler, (void*)2));
    ESP_ERROR_CHECK(gpio_isr_handler_add(BENCB, encoder_isr_handler, (void*)3));
    
    ESP_LOGI(TAG, "Encoder pins initialized");
    return ESP_OK;
}

static void encoder_isr_handler(void *arg)
{
    // Proper quadrature encoder ISR handler
    uint32_t pin_num = (uint32_t)arg;
    
    if (pin_num == 0 || pin_num == 1) { // Left encoder (AENCA/AENCB)
        static int last_a_state = 0;
        static int last_b_state = 0;
        
        int a_state = gpio_get_level(AENCA);
        int b_state = gpio_get_level(AENCB);
        
        // Quadrature decoding: A leads B for forward, B leads A for reverse
        if (a_state != last_a_state) {
            if (a_state == b_state) {
                current_encoder_data.left_count++;
            } else {
                current_encoder_data.left_count--;
            }
            last_a_state = a_state;
        }
        if (b_state != last_b_state) {
            if (b_state != a_state) {
                current_encoder_data.left_count++;
            } else {
                current_encoder_data.left_count--;
            }
            last_b_state = b_state;
        }
    } else if (pin_num == 2 || pin_num == 3) { // Right encoder (BENCA/BENCB)
        static int last_a_state_r = 0;
        static int last_b_state_r = 0;
        
        int a_state = gpio_get_level(BENCA);
        int b_state = gpio_get_level(BENCB);
        
        // Quadrature decoding: A leads B for forward, B leads A for reverse
        if (a_state != last_a_state_r) {
            if (a_state == b_state) {
                current_encoder_data.right_count++;
            } else {
                current_encoder_data.right_count--;
            }
            last_a_state_r = a_state;
        }
        if (b_state != last_b_state_r) {
            if (b_state != a_state) {
                current_encoder_data.right_count++;
            } else {
                current_encoder_data.right_count--;
            }
            last_b_state_r = b_state;
        }
    }
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
    if (motor_direction) {
        if (pwm_int < 0) {
            gpio_set_level(AIN1, 1);
            gpio_set_level(AIN2, 0);
            ledc_set_duty(LEDC_LOW_SPEED_MODE, LEDC_CHANNEL_0, abs(pwm_int));
        } else {
            gpio_set_level(AIN1, 0);
            gpio_set_level(AIN2, 1);
            ledc_set_duty(LEDC_LOW_SPEED_MODE, LEDC_CHANNEL_0, abs(pwm_int));
        }
    } else {
        if (pwm_int < 0) {
            gpio_set_level(AIN1, 0);
            gpio_set_level(AIN2, 1);
            ledc_set_duty(LEDC_LOW_SPEED_MODE, LEDC_CHANNEL_0, abs(pwm_int));
        } else {
            gpio_set_level(AIN1, 1);
            gpio_set_level(AIN2, 0);
            ledc_set_duty(LEDC_LOW_SPEED_MODE, LEDC_CHANNEL_0, abs(pwm_int));
        }
    }
}

static void motor_right_control(float pwm_input) {
    int pwm_int = round(pwm_input);
    if (motor_direction) {
        if (pwm_int < 0) {
            gpio_set_level(BIN1, 1);
            gpio_set_level(BIN2, 0);
            ledc_set_duty(LEDC_LOW_SPEED_MODE, LEDC_CHANNEL_1, abs(pwm_int));
        } else {
            gpio_set_level(BIN1, 0);
            gpio_set_level(BIN2, 1);
            ledc_set_duty(LEDC_LOW_SPEED_MODE, LEDC_CHANNEL_1, abs(pwm_int));
        }
    } else {
        if (pwm_int < 0) {
            gpio_set_level(BIN1, 0);
            gpio_set_level(BIN2, 1);
            ledc_set_duty(LEDC_LOW_SPEED_MODE, LEDC_CHANNEL_1, abs(pwm_int));
        } else {
            gpio_set_level(BIN1, 1);
            gpio_set_level(BIN2, 0);
            ledc_set_duty(LEDC_LOW_SPEED_MODE, LEDC_CHANNEL_1, abs(pwm_int));
        }
    }
}

static void calculate_left_wheel_speed(void) {
    uint64_t current_time = esp_timer_get_time();
    int32_t encoder_pulses = current_encoder_data.left_count;
    
    if (last_left_speed_time > 0) {
        double dt = (double)(current_time - last_left_speed_time) / 1000000.0; // Convert to seconds
        
        if (dt > 0.001) { // Minimum 1ms time difference
            if (!motor_direction) {
                left_wheel_speed = (encoder_pulse_rate * (encoder_pulses - last_left_encoder_count)) / dt;
            } else {
                left_wheel_speed = (encoder_pulse_rate * (last_left_encoder_count - encoder_pulses)) / dt;
            }
        }
    }
    
    last_left_encoder_count = encoder_pulses;
    last_left_speed_time = current_time;
}

static void calculate_right_wheel_speed(void) {
    uint64_t current_time = esp_timer_get_time();
    int32_t encoder_pulses = current_encoder_data.right_count;
    
    if (last_right_speed_time > 0) {
        double dt = (double)(current_time - last_right_speed_time) / 1000000.0; // Convert to seconds
        
        if (dt > 0.001) { // Minimum 1ms time difference
            if (!motor_direction) {
                right_wheel_speed = (encoder_pulse_rate * (encoder_pulses - last_right_encoder_count)) / dt;
            } else {
                right_wheel_speed = (encoder_pulse_rate * (last_right_encoder_count - encoder_pulses)) / dt;
            }
        }
    }
    
    last_right_encoder_count = encoder_pulses;
    last_right_speed_time = current_time;
}

static void compute_left_pid_controller(void) {
    if (!pid_compute_enabled) {
        return;
    }

    left_pid_output = compute_pid(&left_pid_controller, left_setpoint, left_wheel_speed);
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
        return;
    }

    right_pid_output = compute_pid(&right_pid_controller, right_setpoint, right_wheel_speed);
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
