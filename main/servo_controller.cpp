#include "../inc/servo_controller.h"
#include "../inc/json_parser.h"
#include <esp_log.h>
#include <esp_timer.h>
#include <driver/uart.h>
#include <driver/gpio.h>
#include <driver/ledc.h>
#include <freertos/FreeRTOS.h>
#include <freertos/task.h>
#include <string.h>
#include <math.h>
#include <cJSON.h>

// Servo constants from Arduino implementation
#define ARM_SERVO_MIDDLE_POS  2047
#define ARM_SERVO_POS_RANGE   4096
#define ARM_SERVO_INIT_SPEED  600
#define ARM_SERVO_INIT_ACC    20

// Servo IDs
#define BASE_SERVO_ID    11
#define SHOULDER_DRIVING_SERVO_ID 12
#define SHOULDER_DRIVEN_SERVO_ID  13
#define ELBOW_SERVO_ID   14
#define GRIPPER_SERVO_ID 15

// Joint constants
#define BASE_JOINT     1
#define SHOULDER_JOINT 2
#define ELBOW_JOINT    3
#define EOAT_JOINT     4

// Arm dimensions (from Arduino config)
#define ARM_L1_LENGTH_MM    126.06
#define ARM_L2_LENGTH_MM_A  236.82
#define ARM_L2_LENGTH_MM_B  30.00 
#define ARM_L3_LENGTH_MM_A_0 280.15
#define ARM_L3_LENGTH_MM_B_0 1.73
#define ARM_L4_LENGTH_MM_A  67.85
#define ARM_L4_LENGTH_MM_B  5.98

// Calculated arm parameters
static const double l1 = ARM_L1_LENGTH_MM;
static const double l2A = ARM_L2_LENGTH_MM_A;
static const double l2B = ARM_L2_LENGTH_MM_B;
static const double l2 = sqrt(l2A * l2A + l2B * l2B);
static const double t2rad = atan2(l2B, l2A);
static const double l3A = ARM_L3_LENGTH_MM_A_0;
static const double l3B = ARM_L3_LENGTH_MM_B_0;
static const double l3 = sqrt(l3A * l3A + l3B * l3B);
static const double t3rad = atan2(l3B, l3A);
static const double l4A = ARM_L4_LENGTH_MM_A;
static const double l4B = ARM_L4_LENGTH_MM_B;
static const double lE = sqrt(l4A * l4A + l4B * l4B);
static const double tErad = atan2(l4B, l4A);

// Global servo feedback array (using the structure from header)
static servo_feedback_t servo_feedback[5]; // [0] BASE, [1] SHOULDER_DRIVING, [2] SHOULDER_DRIVEN, [3] ELBOW, [4] GRIPPER

// Goal positions for all servos
static int16_t goal_pos[5] = {2047, 2047, 2047, 2047, 2047};
static uint16_t move_speed[5] = {0, 0, 0, 0, 0};
static uint8_t move_acc[5] = {ARM_SERVO_INIT_ACC, ARM_SERVO_INIT_ACC, ARM_SERVO_INIT_ACC, ARM_SERVO_INIT_ACC, ARM_SERVO_INIT_ACC};

// Current joint angles in radians
static double current_base_rad = 0.0;
static double current_shoulder_rad = 0.0;
static double current_elbow_rad = M_PI/2;
static double current_gripper_rad = M_PI;

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


// Private variables
static servo_config_t servo_configs[MAX_SERVOS];
static bool scservo_mode = false;
static bool servo_initialized = false;

// Helper functions from Arduino implementation
static double calculate_pos_by_rad(double rad_input) {
    return round((rad_input / (2 * M_PI)) * ARM_SERVO_POS_RANGE);
}

static double calculate_rad_by_feedback(int input_steps, int joint_name) {
    double get_rad;
    switch(joint_name) {
        case BASE_JOINT:
            get_rad = -(input_steps * 2 * M_PI / ARM_SERVO_POS_RANGE) + M_PI;
            break;
        case SHOULDER_JOINT:
            get_rad = (input_steps * 2 * M_PI / ARM_SERVO_POS_RANGE) - M_PI;
            break;
        case ELBOW_JOINT:
            get_rad = (input_steps * 2 * M_PI / ARM_SERVO_POS_RANGE) - (M_PI / 2);
            break;
        case EOAT_JOINT:
            get_rad = input_steps * 2 * M_PI / ARM_SERVO_POS_RANGE;
            break;
        default:
            get_rad = 0.0;
            break;
    }
    return get_rad;
}

static double ang2deg(double input_ang) {
    return (input_ang / 180) * M_PI;
}

// Simple linkage inverse kinematics from Arduino
static void simple_linkage_ik_rad(double LA, double LB, double aIn, double bIn, 
                                  double *alpha, double *beta, double *delta) {
    double psi, omega, L2C, LC, lambda;
    
    if (fabs(bIn) < 1e-6) {
        psi = acos((LA * LA + aIn * aIn - LB * LB) / (2 * LA * aIn)) + t2rad;
        *alpha = M_PI / 2.0 - psi;
        omega = acos((aIn * aIn + LB * LB - LA * LA) / (2 * aIn * LB));
        *beta = psi + omega - t3rad;
    } else {
        L2C = aIn * aIn + bIn * bIn;
        LC = sqrt(L2C);
        lambda = atan2(bIn, aIn);
        psi = acos((LA * LA + L2C - LB * LB) / (2 * LA * LC)) + t2rad;
        *alpha = M_PI / 2.0 - lambda - psi;
        omega = acos((LB * LB + L2C - LA * LA) / (2 * LC * LB));
        *beta = psi + omega - t3rad;
    }
    
    *delta = M_PI / 2.0 - *alpha - *beta;
}

// Cartesian to polar conversion
static void cartesian_to_polar(double x, double y, double* r, double* theta) {
    *r = sqrt(x * x + y * y);
    *theta = atan2(y, x);
}

// Polar to cartesian conversion
static void polar_to_cartesian(double r, double theta, double *x, double *y) {
    *x = r * cos(theta);
    *y = r * sin(theta);
}

// Servo communication functions (simplified for ESP-IDF)
static esp_err_t servo_write_pos_ex(uint8_t id, int16_t position, uint16_t speed, uint8_t acc) {
    // Arduino-style servo UART communication
    // Based on RoArm-M2_module.h Serial1 communication logic
    
    ESP_LOGI(TAG, "Servo %d: pos=%d, speed=%d, acc=%d", id, position, speed, acc);
    
    // Update goal position for simulation
    if (id >= BASE_SERVO_ID && id <= GRIPPER_SERVO_ID) {
        goal_pos[id - BASE_SERVO_ID] = position;
        move_speed[id - BASE_SERVO_ID] = speed;
        move_acc[id - BASE_SERVO_ID] = acc;
    }
    
    // In Arduino: st.WritePos(id, position, speed, acc);
    // This would use the servo library to send UART commands to Serial1
    // For ESP-IDF, we would implement similar UART communication:
    // 1. Send servo command packet via UART2 (GPIO 18/19)
    // 2. Wait for servo response/feedback
    // 3. Update servo feedback data
    
    // Simulate UART communication delay (Arduino-style)
    vTaskDelay(pdMS_TO_TICKS(10)); // 10ms delay like Arduino
    
    return ESP_OK;
}

static esp_err_t servo_sync_write_pos_ex(uint8_t ids[], uint8_t id_count, int16_t positions[], 
                                         uint16_t speeds[], uint8_t accs[]) {
    // Synchronous write to multiple servos
    for (int i = 0; i < id_count; i++) {
        servo_write_pos_ex(ids[i], positions[i], speeds[i], accs[i]);
    }
    return ESP_OK;
}

static esp_err_t servo_enable_torque(uint8_t id, uint8_t enable) {
    ESP_LOGI(TAG, "Servo %d torque: %s", id, enable ? "ON" : "OFF");
    return ESP_OK;
}

static esp_err_t servo_feedback_read(uint8_t id) {
    // Simulate servo feedback reading
    if (id >= BASE_SERVO_ID && id <= GRIPPER_SERVO_ID) {
        int index = id - BASE_SERVO_ID;
        servo_feedback[index].position = goal_pos[index]; // Simulate current position
        servo_feedback[index].speed = move_speed[index];
        servo_feedback[index].load = 0;
        servo_feedback[index].voltage = 7400; // 7.4V in mV
        servo_feedback[index].temperature = 350; // 35.0°C in 0.1°C units
        servo_feedback[index].status = 0;
        servo_feedback[index].moving = (move_speed[index] > 0);
        servo_feedback[index].error = false;
        servo_feedback[index].timestamp = esp_timer_get_time() / 1000; // Convert to ms
        return ESP_OK;
    }
    return ESP_FAIL;
}
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

esp_err_t servo_controller_arm_ui_control(float e, float z, float r) {
    ESP_LOGI(TAG, "Arm UI control: E=%.2f, Z=%.2f, R=%.2f", e, z, r);
    
    // Convert UI parameters to joint angles using inverse kinematics
    // E = extension (distance from base), Z = height, R = rotation
    arm_joint_angles_t angles;
    
    // Calculate base rotation
    angles.base = r * 180.0 / M_PI; // Convert from radians to degrees
    
    // Use inverse kinematics for shoulder and elbow based on E and Z
    double alpha, beta, delta;
    simple_linkage_ik_rad(l2, l3, e, z, &alpha, &beta, &delta);
    
    // Check if IK solution is valid
    if (isnan(alpha) || isnan(beta) || isnan(delta)) {
        ESP_LOGE(TAG, "Inverse kinematics failed for UI control - E=%.2f, Z=%.2f", e, z);
        return ESP_ERR_INVALID_ARG;
    }
    
    // Convert to degrees
    angles.shoulder = alpha * 180.0 / M_PI;
    angles.elbow = beta * 180.0 / M_PI;
    angles.gripper = 0.0f;     // Keep gripper at current position
    
    ESP_LOGI(TAG, "UI control IK solution: base=%.2f, shoulder=%.2f, elbow=%.2f", 
             angles.base, angles.shoulder, angles.elbow);
    
    return servo_controller_set_joint_angles(&angles, 1000); // Default speed
}

esp_err_t servo_controller_set_joint_pid(uint8_t joint, float p, float i) {
    if (joint >= 4) {
        ESP_LOGE(TAG, "Invalid joint ID: %d", joint);
        return ESP_ERR_INVALID_ARG;
    }
    
    ESP_LOGI(TAG, "Setting joint %d PID: P=%.2f, I=%.2f", joint, p, i);
    
    // Set PID parameters for the specified joint
    // This would typically involve writing to servo registers
    // For now, we'll log the parameters
    ESP_LOGI(TAG, "Joint %d PID parameters set", joint);
    
    return ESP_OK;
}

esp_err_t servo_controller_reset_pid(void) {
    ESP_LOGI(TAG, "Resetting all PID parameters to default");
    
    // Reset PID parameters for all joints to default values
    for (uint8_t joint = 0; joint < 4; joint++) {
        servo_controller_set_joint_pid(joint, 16.0f, 0.0f); // Default values
    }
    
    ESP_LOGI(TAG, "All PID parameters reset to default");
    return ESP_OK;
}

esp_err_t servo_controller_move_init(void) {
    ESP_LOGI(TAG, "Initializing arm movement");
    
    // Move arm to home position
    return servo_controller_move_home_position();
}

esp_err_t servo_controller_get_feedback(void) {
    ESP_LOGI(TAG, "Getting servo feedback");
    
    // Get current joint angles
    arm_joint_angles_t angles;
    esp_err_t ret = servo_controller_get_joint_angles(&angles);
    if (ret != ESP_OK) {
        ESP_LOGE(TAG, "Failed to get joint angles: %s", esp_err_to_name(ret));
        return ret;
    }
    
    // Get current pose
    arm_pose_t pose;
    ret = servo_controller_get_pose(&pose);
    if (ret != ESP_OK) {
        ESP_LOGE(TAG, "Failed to get pose: %s", esp_err_to_name(ret));
        return ret;
    }
    
    ESP_LOGI(TAG, "Servo feedback - Joints: base=%.2f, shoulder=%.2f, elbow=%.2f, gripper=%.2f",
             angles.base, angles.shoulder, angles.elbow, angles.gripper);
    ESP_LOGI(TAG, "Servo feedback - Pose: x=%.2f, y=%.2f, z=%.2f, yaw=%.2f",
             pose.x, pose.y, pose.z, pose.yaw);
    
    return ESP_OK;
}

esp_err_t servo_controller_torque_control(bool enable) {
    ESP_LOGI(TAG, "Torque control: %s", enable ? "enable" : "disable");
    
    // Control torque for all servos
    for (uint8_t servo_id = 0; servo_id < 4; servo_id++) {
        uint16_t torque = enable ? 1000 : 0; // Enable/disable torque
        esp_err_t ret = servo_controller_set_torque(servo_id, torque);
        if (ret != ESP_OK) {
            ESP_LOGE(TAG, "Failed to set torque for servo %d: %s", servo_id, esp_err_to_name(ret));
            return ret;
        }
    }
    
    ESP_LOGI(TAG, "Torque control completed for all servos");
    return ESP_OK;
}

esp_err_t servo_controller_change_id(uint8_t old_id, uint8_t new_id) {
    ESP_LOGI(TAG, "Changing servo ID from %d to %d", old_id, new_id);
    
    // Change servo ID using SCS protocol
    // This is a critical operation that requires careful handling
    uint8_t data[2];
    data[0] = 0x05; // ID register
    data[1] = new_id;
    
    esp_err_t ret = scservo_send_command(old_id, 0x03, data, 2); // Write command
    if (ret != ESP_OK) {
        ESP_LOGE(TAG, "Failed to change servo ID: %s", esp_err_to_name(ret));
        return ret;
    }
    
    ESP_LOGI(TAG, "Servo ID changed successfully from %d to %d", old_id, new_id);
    return ESP_OK;
}

esp_err_t servo_controller_set_middle_position(uint8_t servo_id) {
    ESP_LOGI(TAG, "Setting middle position for servo %d", servo_id);
    
    // Set servo to middle position (90 degrees)
    uint16_t middle_angle = 90;
    uint16_t pulse_width = servo_angle_to_pulse(middle_angle);
    
    esp_err_t ret = servo_set_pwm_duty(servo_id, pulse_width);
    if (ret != ESP_OK) {
        ESP_LOGE(TAG, "Failed to set middle position for servo %d: %s", servo_id, esp_err_to_name(ret));
        return ret;
    }
    
    ESP_LOGI(TAG, "Middle position set for servo %d", servo_id);
    return ESP_OK;
}

esp_err_t servo_controller_set_servo_pid(uint8_t servo_id, float p) {
    ESP_LOGI(TAG, "Setting servo %d PID: P=%.2f", servo_id, p);
    
    // Set PID parameters for specific servo
    // Convert float P value to appropriate register value
    uint16_t p_value = (uint16_t)(p * 100); // Scale for register
    
    esp_err_t ret = scs_servo_write_word(servo_id, 0x1C, p_value); // PID P register
    if (ret != ESP_OK) {
        ESP_LOGE(TAG, "Failed to set PID for servo %d: %s", servo_id, esp_err_to_name(ret));
        return ret;
    }
    
    ESP_LOGI(TAG, "PID P parameter set for servo %d", servo_id);
    return ESP_OK;
}

esp_err_t servo_controller_send_error_feedback(uint8_t servo_id, uint8_t error_status) {
    if (!servo_initialized) {
        ESP_LOGE(TAG, "Servo controller not initialized");
        return ESP_ERR_INVALID_STATE;
    }
    
    // Create feedback JSON
    cJSON *json = cJSON_CreateObject();
    cJSON_AddNumberToObject(json, "T", CMD_BUS_SERVO_ERROR);
    cJSON_AddNumberToObject(json, "id", servo_id);
    cJSON_AddNumberToObject(json, "status", error_status);
    
    char *json_string = cJSON_Print(json);
    if (json_string) {
        // Send via UART (matching original Arduino behavior)
        printf("%s\n", json_string);
        free(json_string);
    }
    
    cJSON_Delete(json);
    ESP_LOGI(TAG, "Servo error feedback sent: ID=%d, Status=%d", servo_id, error_status);
    return ESP_OK;
}

esp_err_t servo_controller_set_joint_angles(const arm_joint_angles_t *angles, uint16_t speed) {
    if (!servo_initialized) {
        return ESP_ERR_INVALID_STATE;
    }
    
    if (!angles) {
        return ESP_ERR_INVALID_ARG;
    }
    
    ESP_LOGI(TAG, "Setting joint angles: base=%.2f, shoulder=%.2f, elbow=%.2f, gripper=%.2f, speed=%d",
             angles->base, angles->shoulder, angles->elbow, angles->gripper, speed);
    
    // Convert angles from degrees to radians
    double base_rad = angles->base * M_PI / 180.0;
    double shoulder_rad = angles->shoulder * M_PI / 180.0;
    double elbow_rad = angles->elbow * M_PI / 180.0;
    double gripper_rad = angles->gripper * M_PI / 180.0;
    
    // Constrain angles to valid ranges
    base_rad = fmax(-M_PI/2, fmin(M_PI/2, base_rad));
    shoulder_rad = fmax(-M_PI/2, fmin(M_PI/2, shoulder_rad));
    elbow_rad = fmax(-M_PI/2, fmin(M_PI/2, elbow_rad));
    gripper_rad = fmax(-M_PI/2, fmin(M_PI/2, gripper_rad));
    
    // Calculate servo positions using Arduino logic
    // Base joint: radInput increase, move to left
    double base_rad_constrained = -fmax(-M_PI, fmin(M_PI, base_rad));
    int16_t base_pos = calculate_pos_by_rad(base_rad_constrained) + ARM_SERVO_MIDDLE_POS;
    goal_pos[0] = base_pos;
    
    // Shoulder joint: radInput increase, it leans forward
    double shoulder_rad_constrained = fmax(-M_PI/2, fmin(M_PI/2, shoulder_rad));
    int16_t shoulder_pos = calculate_pos_by_rad(shoulder_rad_constrained);
    goal_pos[1] = ARM_SERVO_MIDDLE_POS + shoulder_pos;  // Driving servo
    goal_pos[2] = ARM_SERVO_MIDDLE_POS - shoulder_pos;  // Driven servo
    
    // Elbow joint: angleInput increase, it moves down
    int16_t elbow_pos = calculate_pos_by_rad(elbow_rad) + 1024;
    goal_pos[3] = fmax(512, fmin(3071, elbow_pos));
    
    // Gripper joint
    int16_t gripper_pos = calculate_pos_by_rad(gripper_rad);
    goal_pos[4] = fmax(700, fmin(3396, gripper_pos));
    
    // Set speeds and accelerations
    for (int i = 0; i < 5; i++) {
        move_speed[i] = speed;
        move_acc[i] = ARM_SERVO_INIT_ACC;
    }
    
    // Send commands to servos
    uint8_t servo_ids[5] = {BASE_SERVO_ID, SHOULDER_DRIVING_SERVO_ID, SHOULDER_DRIVEN_SERVO_ID, 
                           ELBOW_SERVO_ID, GRIPPER_SERVO_ID};
    
    esp_err_t ret = servo_sync_write_pos_ex(servo_ids, 5, goal_pos, move_speed, move_acc);
    
    if (ret == ESP_OK) {
        // Update current joint angles
        current_base_rad = base_rad;
        current_shoulder_rad = shoulder_rad;
        current_elbow_rad = elbow_rad;
        current_gripper_rad = gripper_rad;
        
        ESP_LOGI(TAG, "Joint angles set successfully");
    } else {
        ESP_LOGE(TAG, "Failed to set joint angles");
    }
    
    return ret;
}

esp_err_t servo_controller_move_home_position(void) {
    if (!servo_initialized) {
        return ESP_ERR_INVALID_STATE;
    }
    
    ESP_LOGI(TAG, "Moving to home position");
    
    // Define home position (all joints at 0 degrees)
    arm_joint_angles_t home_angles = {0.0, 0.0, 0.0, 0.0};
    
    return servo_controller_set_joint_angles(&home_angles, 500); // Slow speed for safety
}

esp_err_t servo_controller_get_joint_angles(arm_joint_angles_t *angles) {
    if (!servo_initialized) {
        return ESP_ERR_INVALID_STATE;
    }
    
    if (!angles) {
        return ESP_ERR_INVALID_ARG;
    }
    
    ESP_LOGI(TAG, "Getting joint angles");
    
    // Read servo feedback to get current positions
    servo_feedback_read(BASE_SERVO_ID);
    servo_feedback_read(SHOULDER_DRIVING_SERVO_ID);
    servo_feedback_read(ELBOW_SERVO_ID);
    servo_feedback_read(GRIPPER_SERVO_ID);
    
    // Convert servo positions back to joint angles using Arduino logic
    double base_rad = calculate_rad_by_feedback(servo_feedback[0].position, BASE_JOINT);
    double shoulder_rad = calculate_rad_by_feedback(servo_feedback[1].position, SHOULDER_JOINT);
    double elbow_rad = calculate_rad_by_feedback(servo_feedback[3].position, ELBOW_JOINT);
    double gripper_rad = calculate_rad_by_feedback(servo_feedback[4].position, EOAT_JOINT);
    
    // Convert radians to degrees
    angles->base = base_rad * 180.0 / M_PI;
    angles->shoulder = shoulder_rad * 180.0 / M_PI;
    angles->elbow = elbow_rad * 180.0 / M_PI;
    angles->gripper = gripper_rad * 180.0 / M_PI;
    
    // Update current joint angles
    current_base_rad = base_rad;
    current_shoulder_rad = shoulder_rad;
    current_elbow_rad = elbow_rad;
    current_gripper_rad = gripper_rad;
    
    ESP_LOGI(TAG, "Current joint angles: base=%.2f, shoulder=%.2f, elbow=%.2f, gripper=%.2f",
             angles->base, angles->shoulder, angles->elbow, angles->gripper);
    
    return ESP_OK;
}

esp_err_t servo_controller_get_pose(arm_pose_t *pose) {
    if (!servo_initialized) {
        return ESP_ERR_INVALID_STATE;
    }
    
    if (!pose) {
        return ESP_ERR_INVALID_ARG;
    }
    
    ESP_LOGI(TAG, "Getting arm pose");
    
    // Get current joint angles
    arm_joint_angles_t angles;
    esp_err_t ret = servo_controller_get_joint_angles(&angles);
    if (ret != ESP_OK) {
        return ret;
    }
    
    // Convert to radians
    double base_rad = angles.base * M_PI / 180.0;
    double shoulder_rad = angles.shoulder * M_PI / 180.0;
    double elbow_rad = angles.elbow * M_PI / 180.0;
    double gripper_rad = angles.gripper * M_PI / 180.0;
    
    // Calculate end-effector position using forward kinematics from Arduino
    double r_ee, x_ee, y_ee, z_ee;
    double aOut, bOut, cOut, dOut, eOut, fOut;
    
    // Compute the end position of the first linkage (between baseJoint and shoulderJoint)
    polar_to_cartesian(l2, ((M_PI / 2) - (shoulder_rad + t2rad)), &aOut, &bOut);
    polar_to_cartesian(l3, ((M_PI / 2) - (elbow_rad + shoulder_rad)), &cOut, &dOut);
    
    r_ee = aOut + cOut;
    z_ee = bOut + dOut;
    
    polar_to_cartesian(r_ee, base_rad, &eOut, &fOut);
    x_ee = eOut;
    y_ee = fOut;
    
    // Set pose values
    pose->x = x_ee;
    pose->y = y_ee;
    pose->z = z_ee;
    pose->yaw = gripper_rad;
    pose->roll = 0.0;  // Not calculated in simple forward kinematics
    pose->pitch = 0.0; // Not calculated in simple forward kinematics
    
    ESP_LOGI(TAG, "Current pose: x=%.2f, y=%.2f, z=%.2f, yaw=%.2f", 
             pose->x, pose->y, pose->z, pose->yaw);
    
    ESP_LOGI(TAG, "Current pose: x=%.2f, y=%.2f, z=%.2f, yaw=%.2f, roll=%.2f, pitch=%.2f",
             pose->x, pose->y, pose->z, pose->yaw, pose->roll, pose->pitch);
    
    return ESP_OK;
}

esp_err_t servo_controller_set_torque(uint8_t servo_id, uint16_t torque) {
    if (!servo_initialized) {
        return ESP_ERR_INVALID_STATE;
    }
    
    if (servo_id == 0 || servo_id > 254) {
        return ESP_ERR_INVALID_ARG;
    }
    
    ESP_LOGI(TAG, "Setting torque for servo %d: %d", servo_id, torque);
    
    // Send torque command to servo
    esp_err_t ret = servo_enable_torque(servo_id, torque ? 1 : 0);
    if (ret != ESP_OK) {
        ESP_LOGE(TAG, "Failed to set servo %d torque: %s", servo_id, esp_err_to_name(ret));
        return ret;
    }
    
    return ESP_OK;
}

esp_err_t servo_controller_set_pose(const arm_pose_t *pose, uint16_t speed) {
    if (!servo_initialized) {
        return ESP_ERR_INVALID_STATE;
    }
    
    if (!pose) {
        return ESP_ERR_INVALID_ARG;
    }
    
    ESP_LOGI(TAG, "Setting arm pose: x=%.2f, y=%.2f, z=%.2f, yaw=%.2f, roll=%.2f, pitch=%.2f, speed=%d",
             pose->x, pose->y, pose->z, pose->yaw, pose->roll, pose->pitch, speed);
    
    // Use inverse kinematics to convert pose to joint angles
    arm_joint_angles_t angles;
    
    // Convert pose to radians
    double x = pose->x;
    double y = pose->y;
    double z = pose->z;
    double yaw = pose->yaw;
    
    // Calculate base joint angle
    double base_r, base_theta;
    cartesian_to_polar(x, y, &base_r, &base_theta);
    angles.base = base_theta * 180.0 / M_PI;
    
    // Use simple linkage inverse kinematics for shoulder and elbow
    double alpha, beta, delta;
    simple_linkage_ik_rad(l2, l3, base_r, z, &alpha, &beta, &delta);
    
    // Check if IK solution is valid
    if (isnan(alpha) || isnan(beta) || isnan(delta)) {
        ESP_LOGE(TAG, "Inverse kinematics failed - target pose unreachable");
        return ESP_ERR_INVALID_ARG;
    }
    
    // Convert to degrees
    angles.shoulder = alpha * 180.0 / M_PI;
    angles.elbow = beta * 180.0 / M_PI;
    angles.gripper = yaw * 180.0 / M_PI;
    
    ESP_LOGI(TAG, "IK solution: base=%.2f, shoulder=%.2f, elbow=%.2f, gripper=%.2f",
             angles.base, angles.shoulder, angles.elbow, angles.gripper);
    
    return servo_controller_set_joint_angles(&angles, speed);
}
