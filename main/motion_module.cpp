#include "../inc/motion_module.h"
#include <esp_log.h>
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

// Global variables (matching Arduino)
static bool motion_initialized = false;
static float spd_rate_A = 1.0f;  // Left motor speed rate
static float spd_rate_B = 1.0f;  // Right motor speed rate
static bool usePIDCompute = true;
static bool heartbeatStopFlag = false;

// Encoder variables (matching Arduino)
static int lastEncoderA = 0;
static int lastEncoderB = 0;
static double speedGetA = 0.0;
static double speedGetB = 0.0;
static double plusesRate = M_PI * WHEEL_D / ONE_CIRCLE_PLUSES;
static uint64_t lastLeftSpdTime = 0;
static uint64_t lastRightSpdTime = 0;

// PID variables (matching Arduino)
static double outputA = 0.0;
static double outputB = 0.0;
static double setpointA = 0.0;
static double setpointB = 0.0;
static float setpointA_buffer = 0.0f;
static float setpointB_buffer = 0.0f;

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

// Arduino-style motor control functions
static void switchPortCtrlA(float pwmInputA);
static void switchPortCtrlB(float pwmInputB);
static void leftCtrl(float pwmInputA);
static void rightCtrl(float pwmInputB);
static void getLeftSpeed(void);
static void getRightSpeed(void);
static void LeftPidControllerCompute(void);
static void RightPidControllerCompute(void);

esp_err_t motion_module_init(void)
{
    ESP_LOGI(TAG, "Initializing motion module (Arduino-style)...");
    
    // Initialize GPIO pins (matching Arduino movtionPinInit)
    gpio_config_t io_conf = {};
    io_conf.mode = GPIO_MODE_OUTPUT;
    io_conf.pin_bit_mask = (1ULL << AIN1) | (1ULL << AIN2) | (1ULL << BIN1) | (1ULL << BIN2);
    io_conf.pull_down_en = GPIO_PULLDOWN_DISABLE;
    io_conf.pull_up_en = GPIO_PULLUP_DISABLE;
    io_conf.intr_type = GPIO_INTR_DISABLE;
    ESP_ERROR_CHECK(gpio_config(&io_conf));
    
    // Set initial pin states (matching Arduino)
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
    
    // Initialize PID controllers (matching Arduino pidControllerInit)
    memset(&left_pid_controller, 0, sizeof(pid_controller_t));
    memset(&right_pid_controller, 0, sizeof(pid_controller_t));
    
    // Initialize encoder data
    current_encoder_data.left_count = 0;
    current_encoder_data.right_count = 0;
    current_encoder_data.left_speed = 0.0f;
    current_encoder_data.right_speed = 0.0f;
    
    motion_initialized = true;
    ESP_LOGI(TAG, "Motion module initialized successfully (Arduino-style)");
    
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
    if (!motion_initialized) {
        return ESP_ERR_INVALID_STATE;
    }
    
    // Compute PID for left motor
    if (left_pid_controller.enabled) {
        left_pid_controller.output = compute_pid(&left_pid_controller, 
                                               left_pid_controller.setpoint, 
                                               current_encoder_data.left_speed);
    }
    
    // Compute PID for right motor
    if (right_pid_controller.enabled) {
        right_pid_controller.output = compute_pid(&right_pid_controller, 
                                                right_pid_controller.setpoint, 
                                                current_encoder_data.right_speed);
    }
    
    return ESP_OK;
}

esp_err_t motion_module_apply_motor_control(void)
{
    if (!motion_initialized || emergency_stop_flag) {
        return ESP_ERR_INVALID_STATE;
    }
    
    // Apply PID output to motors
    int left_pwm = (int)left_pid_controller.output;
    int right_pwm = (int)right_pid_controller.output;
    
    // Apply speed rates to PWM values
    left_pwm = (int)(left_pwm * spd_rate_A);
    right_pwm = (int)(right_pwm * spd_rate_B);
    
    // Limit PWM values
    left_pwm = (left_pwm > PWM_MAX_DUTY) ? PWM_MAX_DUTY : (left_pwm < -PWM_MAX_DUTY) ? -PWM_MAX_DUTY : left_pwm;
    right_pwm = (right_pwm > PWM_MAX_DUTY) ? PWM_MAX_DUTY : (right_pwm < -PWM_MAX_DUTY) ? -PWM_MAX_DUTY : right_pwm;
    
    // Apply to motors
    motion_module_set_motor(0, left_pwm > 0 ? 1 : -1, abs(left_pwm));
    motion_module_set_motor(1, right_pwm > 0 ? 1 : -1, abs(right_pwm));
    
    return ESP_OK;
}

esp_err_t motion_module_set_speeds(float left_speed, float right_speed)
{
    if (!motion_initialized) {
        return ESP_ERR_INVALID_STATE;
    }
    
    // Limit speeds
    left_speed = (left_speed > 1.0f) ? 1.0f : (left_speed < -1.0f) ? -1.0f : left_speed;
    right_speed = (right_speed > 1.0f) ? 1.0f : (right_speed < -1.0f) ? -1.0f : right_speed;
    
    // Set PID setpoints
    left_pid_controller.setpoint = left_speed * MOTOR_MAX_SPEED;
    right_pid_controller.setpoint = right_speed * MOTOR_MAX_SPEED;
    
    // Update motion status
    current_motion_status.left_speed = left_speed;
    current_motion_status.right_speed = right_speed;
    
    ESP_LOGI(TAG, "Set speeds: L=%.2f, R=%.2f", left_speed, right_speed);
    
    return ESP_OK;
}

esp_err_t motion_module_set_ros_motion(float linear_speed, float angular_speed)
{
    if (!motion_initialized) {
        return ESP_ERR_INVALID_STATE;
    }
    
    // Convert ROS-style motion to left/right wheel speeds
    // For differential drive: v_left = v_linear - v_angular * wheelbase/2
    //                        v_right = v_linear + v_angular * wheelbase/2
    const float wheelbase = 0.2f; // 20cm wheelbase (adjust for your robot)
    
    float left_speed = linear_speed - angular_speed * wheelbase / 2.0f;
    float right_speed = linear_speed + angular_speed * wheelbase / 2.0f;
    
    // Normalize speeds
    float max_speed = fmaxf(fabsf(left_speed), fabsf(right_speed));
    if (max_speed > MOTOR_MAX_SPEED) {
        left_speed = left_speed * MOTOR_MAX_SPEED / max_speed;
        right_speed = right_speed * MOTOR_MAX_SPEED / max_speed;
    }
    
    // Set the speeds
    ESP_ERROR_CHECK(motion_module_set_speeds(left_speed, right_speed));
    
    // Update motion status
    current_motion_status.linear_speed = linear_speed;
    current_motion_status.angular_speed = angular_speed;
    
    ESP_LOGI(TAG, "Set ROS motion: linear=%.2f, angular=%.2f", linear_speed, angular_speed);
    
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
    spd_rate_A = (left_rate > 1.0f) ? 1.0f : (left_rate < 0.0f) ? 0.0f : left_rate;
    spd_rate_B = (right_rate > 1.0f) ? 1.0f : (right_rate < 0.0f) ? 0.0f : right_rate;
    
    ESP_LOGI(TAG, "Speed rates set: L=%.2f, R=%.2f", spd_rate_A, spd_rate_B);
    
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
    // Arduino-style quadrature encoder ISR handler
    // Based on ESP32Encoder library logic from Arduino project
    uint32_t pin_num = (uint32_t)arg;
    
    // Read current state of both encoder pins
    int a_state = gpio_get_level(AENCA);
    int b_state = gpio_get_level(AENCB);
    
    if (pin_num == 0 || pin_num == 1) { // Left encoder (AENCA/AENCB)
        // Quadrature encoding: determine direction based on A and B states
        static int last_a_state = 0;
        static int last_b_state = 0;
        
        if (a_state != last_a_state) {
            // A pin changed - check B state to determine direction
            if (a_state == b_state) {
                current_encoder_data.left_count++;
            } else {
                current_encoder_data.left_count--;
            }
            last_a_state = a_state;
        }
        if (b_state != last_b_state) {
            // B pin changed - check A state to determine direction
            if (b_state != a_state) {
                current_encoder_data.left_count++;
            } else {
                current_encoder_data.left_count--;
            }
            last_b_state = b_state;
        }
    } else if (pin_num == 2 || pin_num == 3) { // Right encoder (BENCA/BENCB)
        // Quadrature encoding: determine direction based on A and B states
        static int last_a_state_r = 0;
        static int last_b_state_r = 0;
        
        if (a_state != last_a_state_r) {
            // A pin changed - check B state to determine direction
            if (a_state == b_state) {
                current_encoder_data.right_count++;
            } else {
                current_encoder_data.right_count--;
            }
            last_a_state_r = a_state;
        }
        if (b_state != last_b_state_r) {
            // B pin changed - check A state to determine direction
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
    // Update encoder data from ISR counts
    // Calculate speed based on encoder counts and time
    static uint32_t last_update_time = 0;
    uint32_t current_time = esp_timer_get_time() / 1000; // Convert to ms
    
    if (last_update_time > 0) {
        float dt = (current_time - last_update_time) / 1000.0f; // Convert to seconds
        
        // Calculate speed in counts per second
        current_encoder_data.left_speed = current_encoder_data.left_count / dt;
        current_encoder_data.right_speed = current_encoder_data.right_count / dt;
        
        // Convert to wheel speed (assuming 20 counts per revolution)
        const float COUNTS_PER_REVOLUTION = 20.0f;
        // Store wheel speeds in the available fields
        current_encoder_data.left_speed = (current_encoder_data.left_speed / COUNTS_PER_REVOLUTION) * 2 * M_PI;
        current_encoder_data.right_speed = (current_encoder_data.right_speed / COUNTS_PER_REVOLUTION) * 2 * M_PI;
        
        // Reset counts for next calculation
        current_encoder_data.left_count = 0;
        current_encoder_data.right_count = 0;
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
    
    *left_rate = spd_rate_A;
    *right_rate = spd_rate_B;
    
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
    ret = nvs_set_u32(nvs_handle, "left_speed_rate", *(uint32_t*)&spd_rate_A);
    if (ret == ESP_OK) {
        ret = nvs_set_u32(nvs_handle, "right_speed_rate", *(uint32_t*)&spd_rate_B);
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

// Arduino-style motor control functions implementation

static void switchPortCtrlA(float pwmInputA) {
    int pwmIntA = round(pwmInputA * spd_rate_A);
    if (abs(pwmIntA) < 1e-6) {
        gpio_set_level(AIN1, 0);
        gpio_set_level(AIN2, 0);
        return;
    }

    if (pwmIntA > 0) {
        gpio_set_level(AIN1, 0);
        gpio_set_level(AIN2, 1);
        ledc_set_duty(LEDC_LOW_SPEED_MODE, LEDC_CHANNEL_0, pwmIntA);
    } else {
        gpio_set_level(AIN1, 1);
        gpio_set_level(AIN2, 0);
        ledc_set_duty(LEDC_LOW_SPEED_MODE, LEDC_CHANNEL_0, -pwmIntA);
    }
}

static void switchPortCtrlB(float pwmInputB) {
    int pwmIntB = round(pwmInputB * spd_rate_B);
    if (abs(pwmIntB) < 1e-6) {
        gpio_set_level(BIN1, 0);
        gpio_set_level(BIN2, 0);
        return;
    }

    if (pwmIntB > 0) {
        gpio_set_level(BIN1, 0);
        gpio_set_level(BIN2, 1);
        ledc_set_duty(LEDC_LOW_SPEED_MODE, LEDC_CHANNEL_1, pwmIntB);
    } else {
        gpio_set_level(BIN1, 1);
        gpio_set_level(BIN2, 0);
        ledc_set_duty(LEDC_LOW_SPEED_MODE, LEDC_CHANNEL_1, -pwmIntB);
    }
}

static void leftCtrl(float pwmInputA) {
    int pwmIntA = round(pwmInputA);
    if (SET_MOTOR_DIR) {
        if (pwmIntA < 0) {
            gpio_set_level(AIN1, 1);
            gpio_set_level(AIN2, 0);
            ledc_set_duty(LEDC_LOW_SPEED_MODE, LEDC_CHANNEL_0, abs(pwmIntA));
        } else {
            gpio_set_level(AIN1, 0);
            gpio_set_level(AIN2, 1);
            ledc_set_duty(LEDC_LOW_SPEED_MODE, LEDC_CHANNEL_0, abs(pwmIntA));
        }
    } else {
        if (pwmIntA < 0) {
            gpio_set_level(AIN1, 0);
            gpio_set_level(AIN2, 1);
            ledc_set_duty(LEDC_LOW_SPEED_MODE, LEDC_CHANNEL_0, abs(pwmIntA));
        } else {
            gpio_set_level(AIN1, 1);
            gpio_set_level(AIN2, 0);
            ledc_set_duty(LEDC_LOW_SPEED_MODE, LEDC_CHANNEL_0, abs(pwmIntA));
        }
    }
}

static void rightCtrl(float pwmInputB) {
    int pwmIntB = round(pwmInputB);
    if (SET_MOTOR_DIR) {
        if (pwmIntB < 0) {
            gpio_set_level(BIN1, 1);
            gpio_set_level(BIN2, 0);
            ledc_set_duty(LEDC_LOW_SPEED_MODE, LEDC_CHANNEL_1, abs(pwmIntB));
        } else {
            gpio_set_level(BIN1, 0);
            gpio_set_level(BIN2, 1);
            ledc_set_duty(LEDC_LOW_SPEED_MODE, LEDC_CHANNEL_1, abs(pwmIntB));
        }
    } else {
        if (pwmIntB < 0) {
            gpio_set_level(BIN1, 0);
            gpio_set_level(BIN2, 1);
            ledc_set_duty(LEDC_LOW_SPEED_MODE, LEDC_CHANNEL_1, abs(pwmIntB));
        } else {
            gpio_set_level(BIN1, 1);
            gpio_set_level(BIN2, 0);
            ledc_set_duty(LEDC_LOW_SPEED_MODE, LEDC_CHANNEL_1, abs(pwmIntB));
        }
    }
}

static void getLeftSpeed(void) {
    uint64_t currentTime = esp_timer_get_time();
    long encoderPulsesA = current_encoder_data.left_count;
    
    if (!SET_MOTOR_DIR) {
        speedGetA = (plusesRate * (encoderPulsesA - lastEncoderA)) / ((double)(currentTime - lastLeftSpdTime) / 1000000);
    } else {
        speedGetA = (plusesRate * (lastEncoderA - encoderPulsesA)) / ((double)(currentTime - lastLeftSpdTime) / 1000000);
    }
    lastEncoderA = encoderPulsesA;
    lastLeftSpdTime = currentTime;
}

static void getRightSpeed(void) {
    uint64_t currentTime = esp_timer_get_time();
    long encoderPulsesB = current_encoder_data.right_count;
    
    if (!SET_MOTOR_DIR) {
        speedGetB = (plusesRate * (encoderPulsesB - lastEncoderB)) / ((double)(currentTime - lastRightSpdTime) / 1000000);
    } else {
        speedGetB = (plusesRate * (lastEncoderB - encoderPulsesB)) / ((double)(currentTime - lastRightSpdTime) / 1000000);
    }
    lastEncoderB = encoderPulsesB;
    lastRightSpdTime = currentTime;
}

static void LeftPidControllerCompute(void) {
    if (!usePIDCompute) {
        return;
    }

    outputA = compute_pid(&left_pid_controller, setpointA, speedGetA);
    if (abs(outputA) < PID_THRESHOLD_PWM) {
        outputA = 0;
    }
    if (setpointA == 0 && speedGetA == 0) {
        outputA = 0;
    }
    leftCtrl(outputA);
}

static void RightPidControllerCompute(void) {
    if (!usePIDCompute) {
        return;
    }

    outputB = compute_pid(&right_pid_controller, setpointB, speedGetB);
    if (abs(outputB) < PID_THRESHOLD_PWM) {
        outputB = 0;
    }
    if (setpointB == 0 && speedGetB == 0) {
        outputB = 0;
    }
    rightCtrl(outputB);
}
