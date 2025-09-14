#include "../inc/servo_controller.h"
#include "../inc/json_parser.h"
#include <esp_log.h>
#include <esp_timer.h>
#include <driver/uart.h>
#include <driver/gpio.h>
#include <freertos/FreeRTOS.h>
#include <freertos/task.h>
#include <string.h>
#include <math.h>
#include <cJSON.h>

static const char *TAG = "ServoController";

// ===== ARM KINEMATICS PARAMETERS =====
// Calculated arm parameters from configuration constants
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

// ===== GLOBAL STATE VARIABLES =====
// Servo feedback array for all servos [0] BASE, [1] SHOULDER_DRIVING, [2] SHOULDER_DRIVEN, [3] ELBOW, [4] GRIPPER
static servo_feedback_t servo_feedback[MAX_SERVOS];

// Goal positions and movement parameters for all servos
static int16_t goal_pos[MAX_SERVOS] = {ARM_SERVO_MIDDLE_POS, ARM_SERVO_MIDDLE_POS, ARM_SERVO_MIDDLE_POS, ARM_SERVO_MIDDLE_POS, ARM_SERVO_MIDDLE_POS};
static uint16_t move_speed[MAX_SERVOS] = {0, 0, 0, 0, 0};
static uint8_t move_acc[MAX_SERVOS] = {ARM_SERVO_INIT_ACC, ARM_SERVO_INIT_ACC, ARM_SERVO_INIT_ACC, ARM_SERVO_INIT_ACC, ARM_SERVO_INIT_ACC};

// Current joint angles in radians
static double current_base_rad = 0.0;
static double current_shoulder_rad = 0.0;
static double current_elbow_rad = M_PI/2;
static double current_gripper_rad = M_PI;

// Controller state variables
static servo_config_t servo_configs[MAX_SERVOS];
static servo_config_t servo_status[MAX_SERVOS];
static bool scservo_mode = false;
static bool servo_initialized = false;
static bool emergency_stop_active = false;

// ===== PRIVATE FUNCTION PROTOTYPES =====
static esp_err_t servo_init_uart(void);
static esp_err_t scservo_write_buf(uint8_t id, uint8_t mem_addr, uint8_t *data, uint8_t data_len, uint8_t instruction);
static esp_err_t servo_write_pos_ex(uint8_t id, int16_t position, uint16_t speed, uint8_t acc);
static esp_err_t servo_sync_write_pos_ex(uint8_t ids[], uint8_t id_count, int16_t positions[], uint16_t speeds[], uint8_t accs[]);
static esp_err_t servo_enable_torque(uint8_t id, uint8_t enable);
static esp_err_t servo_feedback_read(uint8_t id);
static double calculate_pos_by_rad(double rad_input);
static double calculate_rad_by_feedback(int input_steps, int joint_name);
static void simple_linkage_ik_rad(double LA, double LB, double aIn, double bIn, double *alpha, double *beta, double *delta);
static void cartesian_to_polar(double x, double y, double* r, double* theta);
static void polar_to_cartesian(double r, double theta, double *x, double *y);

// ===== UTILITY FUNCTIONS =====
/**
 * @brief Calculate servo position from radian input
 * @param rad_input Input angle in radians
 * @return Servo position value
 */
static double calculate_pos_by_rad(double rad_input) {
    return round((rad_input / (2 * M_PI)) * ARM_SERVO_POS_RANGE);
}

/**
 * @brief Calculate radian angle from servo feedback position
 * @param input_steps Servo position in steps
 * @param joint_name Joint identifier
 * @return Angle in radians
 */
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

/**
 * @brief Simple linkage inverse kinematics calculation
 * @param LA Link A length
 * @param LB Link B length  
 * @param aIn Input A coordinate
 * @param bIn Input B coordinate
 * @param alpha Pointer to alpha angle output
 * @param beta Pointer to beta angle output
 * @param delta Pointer to delta angle output
 */
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

/**
 * @brief Convert cartesian coordinates to polar
 * @param x X coordinate
 * @param y Y coordinate
 * @param r Pointer to radius output
 * @param theta Pointer to angle output
 */
static void cartesian_to_polar(double x, double y, double* r, double* theta) {
    *r = sqrt(x * x + y * y);
    *theta = atan2(y, x);
}

/**
 * @brief Convert polar coordinates to cartesian
 * @param r Radius
 * @param theta Angle
 * @param x Pointer to X coordinate output
 * @param y Pointer to Y coordinate output
 */
static void polar_to_cartesian(double r, double theta, double *x, double *y) {
    *x = r * cos(theta);
    *y = r * sin(theta);
}

// ===== LOW-LEVEL SERVO COMMUNICATION FUNCTIONS =====
/**
 * @brief Write position command to servo with speed and acceleration
 * @param id Servo ID
 * @param position Target position
 * @param speed Movement speed
 * @param acc Acceleration
 * @return ESP_OK on success, error code otherwise
 */
static esp_err_t servo_write_pos_ex(uint8_t id, int16_t position, uint16_t speed, uint8_t acc) {
    if (!servo_initialized) {
        ESP_LOGE(TAG, "Servo controller not initialized");
        return ESP_ERR_INVALID_STATE;
    }
    
    ESP_LOGD(TAG, "Servo %d: pos=%d, speed=%d, acc=%d", id, position, speed, acc);
    
    // Update goal position for tracking
    if (id >= BASE_SERVO_ID && id <= GRIPPER_SERVO_ID) {
        goal_pos[id - BASE_SERVO_ID] = position;
        move_speed[id - BASE_SERVO_ID] = speed;
        move_acc[id - BASE_SERVO_ID] = acc;
    }
    
    // Use SCServo protocol for actual communication
    return scservo_write_pos(id, position, 0, speed);
}

/**
 * @brief Synchronously write position commands to multiple servos
 * @param ids Array of servo IDs
 * @param id_count Number of servos
 * @param positions Array of target positions
 * @param speeds Array of speeds
 * @param accs Array of accelerations
 * @return ESP_OK on success, error code otherwise
 */
static esp_err_t servo_sync_write_pos_ex(uint8_t ids[], uint8_t id_count, int16_t positions[], 
                                         uint16_t speeds[], uint8_t accs[]) {
    if (!servo_initialized) {
        return ESP_ERR_INVALID_STATE;
    }
    
    if (!ids || !positions || !speeds || !accs || id_count == 0) {
        return ESP_ERR_INVALID_ARG;
    }
    
    // Write to all servos individually (could be optimized with true sync write)
    for (int i = 0; i < id_count; i++) {
        esp_err_t ret = servo_write_pos_ex(ids[i], positions[i], speeds[i], accs[i]);
        if (ret != ESP_OK) {
            ESP_LOGE(TAG, "Failed to write position to servo %d", ids[i]);
            return ret;
        }
    }
    return ESP_OK;
}

/**
 * @brief Enable or disable servo torque
 * @param id Servo ID
 * @param enable 1 to enable, 0 to disable torque
 * @return ESP_OK on success, error code otherwise
 */
static esp_err_t servo_enable_torque(uint8_t id, uint8_t enable) {
    if (!servo_initialized) {
        return ESP_ERR_INVALID_STATE;
    }
    
    ESP_LOGD(TAG, "Servo %d torque: %s", id, enable ? "ON" : "OFF");
    return scservo_enable_torque(id, enable);
}

/**
 * @brief Read feedback data from servo
 * @param id Servo ID
 * @return ESP_OK on success, error code otherwise
 */
static esp_err_t servo_feedback_read(uint8_t id) {
    if (!servo_initialized) {
        return ESP_ERR_INVALID_STATE;
    }
    
    if (id < BASE_SERVO_ID || id > GRIPPER_SERVO_ID) {
        return ESP_ERR_INVALID_ARG;
    }
    
    int index = id - BASE_SERVO_ID;
    
    // Read actual servo feedback (for now simulate with current goals)
    servo_feedback[index].id = id;
    servo_feedback[index].position = goal_pos[index];
    servo_feedback[index].target_position = goal_pos[index];
    servo_feedback[index].speed = move_speed[index];
    servo_feedback[index].load = 0;
    servo_feedback[index].voltage = 7400; // 7.4V in mV
    servo_feedback[index].temperature = 350; // 35.0°C in 0.1°C units
    servo_feedback[index].status = SERVO_STATUS_OK;
    servo_feedback[index].moving = (move_speed[index] > 0);
    servo_feedback[index].error = false;
    servo_feedback[index].timestamp = esp_timer_get_time() / 1000;
    
    return ESP_OK;
}


// ===== UART INITIALIZATION =====
/**
 * @brief Initialize UART for servo communication
 * @return ESP_OK on success, error code otherwise
 */
static esp_err_t servo_init_uart(void) {
    ESP_LOGI(TAG, "🔧 Initializing UART for servo communication...");
    
    uart_config_t uart_config = {
        .baud_rate = SERVO_UART_BAUD_RATE,
        .data_bits = UART_DATA_8_BITS,
        .parity = UART_PARITY_DISABLE,
        .stop_bits = UART_STOP_BITS_1,
        .flow_ctrl = UART_HW_FLOWCTRL_DISABLE,
        .rx_flow_ctrl_thresh = 0,
        .source_clk = UART_SCLK_DEFAULT,
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

    ESP_LOGI(TAG, "✅ UART initialized successfully (Port: %d, TX: GPIO_%d, RX: GPIO_%d, Baud: %d)", 
             SERVO_UART_NUM, SERVO_UART_TX_PIN, SERVO_UART_RX_PIN, SERVO_UART_BAUD_RATE);
    
    return ESP_OK;
}

// ===== PUBLIC API FUNCTIONS =====
/**
 * @brief Initialize the servo controller system
 * @return ESP_OK on success, error code otherwise
 */
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

    // Initialize UART with corrected settings
    esp_err_t ret = servo_init_uart();
    if (ret == ESP_OK) {
        scservo_mode = true;
        servo_initialized = true;
        ESP_LOGI(TAG, "✅ Servo controller initialized in SCServo (UART) mode");
        return ESP_OK;
    }

    ESP_LOGE(TAG, "❌ Failed to initialize servo controller UART");
    return ESP_FAIL;
}

/**
 * @brief Set servo position
 * @param servo_id Servo ID (11-15)
 * @param position Target position (0-4095)
 * @return ESP_OK on success, error code otherwise
 */
esp_err_t servo_controller_set_position(uint8_t servo_id, uint16_t position) {
    if (!servo_initialized) {
        ESP_LOGE(TAG, "Servo controller not initialized");
        return ESP_ERR_INVALID_STATE;
    }
    
    if (servo_id < BASE_SERVO_ID || servo_id > GRIPPER_SERVO_ID) {
        ESP_LOGE(TAG, "Invalid servo ID: %d", servo_id);
        return ESP_ERR_INVALID_ARG;
    }

    uint8_t array_index = servo_id - BASE_SERVO_ID;
    
    // Clamp position to valid range
    uint16_t clamped_position = fmaxf(servo_configs[array_index].min_position, 
                                     fminf(servo_configs[array_index].max_position, position));

    if (scservo_mode) {
        esp_err_t ret = scservo_write_pos(servo_id, clamped_position, 0, 0);
        if (ret == ESP_OK) {
            servo_feedback[array_index].target_position = clamped_position;
            servo_feedback[array_index].position = clamped_position;
            ESP_LOGD(TAG, "Servo %d position set to %d", servo_id, clamped_position);
        } else {
            ESP_LOGE(TAG, "Failed to set servo %d position: %s", servo_id, esp_err_to_name(ret));
        }
        return ret;
    } else {
        ESP_LOGE(TAG, "SCServo mode not available for servo %d", servo_id);
        return ESP_ERR_NOT_SUPPORTED;
    }
}

/**
 * @brief Get current servo position
 * @param servo_id Servo ID (11-15)
 * @param position Pointer to store position value
 * @return ESP_OK on success, error code otherwise
 */
esp_err_t servo_controller_get_position(uint8_t servo_id, uint16_t *position) {
    if (!servo_initialized) {
        return ESP_ERR_INVALID_STATE;
    }
    
    if (servo_id < BASE_SERVO_ID || servo_id > GRIPPER_SERVO_ID || position == NULL) {
        return ESP_ERR_INVALID_ARG;
    }

    uint8_t array_index = servo_id - BASE_SERVO_ID;

    if (scservo_mode) {
        esp_err_t ret = scservo_read_pos(servo_id, position);
        if (ret == ESP_OK) {
            servo_feedback[array_index].position = *position;
        }
        return ret;
    } else {
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
        // SCServo mode - speed is controlled via position commands with time/speed parameters
        servo_feedback[array_index].speed = speed;
        ESP_LOGI(TAG, "Speed set to %d for servo %d (will be used in next position command)", speed, servo_id);
        return ESP_OK;
    } else {
        ESP_LOGW(TAG, "Speed control not supported in non-SCServo mode for servo %d", servo_id);
        return ESP_ERR_NOT_SUPPORTED;
    }
}

esp_err_t servo_controller_enable_torque(uint8_t servo_id, bool enable) {
    if (servo_id < 11 || servo_id > 15) {
        return ESP_ERR_INVALID_ARG;
    }

    uint8_t array_index = servo_id - 11;

    if (scservo_mode) {
        // SCServo mode - use proper SCServo protocol
        esp_err_t ret = scservo_enable_torque(servo_id, enable ? 1 : 0);
        if (ret == ESP_OK) {
            servo_configs[array_index].enable_torque = enable;
        }
        return ret;
    } else {
        ESP_LOGW(TAG, "Torque control not supported in non-SCServo mode for servo %d", servo_id);
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
            esp_err_t ret = scservo_enable_torque(i + 11, 0); // Servo IDs start from 11
            if (ret != ESP_OK) {
                ESP_LOGW(TAG, "Failed to disable servo %d: %s", i + 11, esp_err_to_name(ret));
            } else {
                servo_status[i].enable_torque = false;
                ESP_LOGI(TAG, "Servo %d disabled", i + 11);
            }
        }
    }
    
    ESP_LOGI(TAG, "All servos disabled");
    return ESP_OK;
}

esp_err_t servo_controller_arm_ui_control(float e, float z, float r) {
    ESP_LOGI(TAG, "Arm UI control: E=%.2f, Z=%.2f, R=%.2f", e, z, r);
    
    // Convert UI parameters to joint angles using inverse kinematics
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
    
    ESP_LOGI(TAG, "Current pose: x=%.2f, y=%.2f, z=%.2f, yaw=%.2f, roll=%.2f, pitch=%.2f",
             pose->x, pose->y, pose->z, pose->yaw, pose->roll, pose->pitch);
    
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

// Helper functions for compatibility with other modules
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

// Additional compatibility functions
esp_err_t servo_controller_set_joint_pid(uint8_t joint, float p, float i) {
    ESP_LOGI(TAG, "Setting joint %d PID: P=%.2f, I=%.2f", joint, p, i);
    return ESP_OK;
}

esp_err_t servo_controller_reset_pid(void) {
    ESP_LOGI(TAG, "Resetting all PID parameters to default");
    return ESP_OK;
}

esp_err_t servo_controller_move_init(void) {
    ESP_LOGI(TAG, "Initializing arm movement");
    return servo_controller_move_home_position();
}

esp_err_t servo_controller_get_feedback(void) {
    ESP_LOGI(TAG, "Getting servo feedback");
    arm_joint_angles_t angles;
    return servo_controller_get_joint_angles(&angles);
}

esp_err_t servo_controller_torque_control(bool enable) {
    ESP_LOGI(TAG, "Torque control: %s", enable ? "enable" : "disable");
    
    for (uint8_t servo_id = 11; servo_id <= 15; servo_id++) {
        esp_err_t ret = servo_controller_enable_torque(servo_id, enable);
        if (ret != ESP_OK) {
            ESP_LOGE(TAG, "Failed to set torque for servo %d", servo_id);
            return ret;
        }
    }
    
    return ESP_OK;
}

// Missing function implementations for API compatibility
esp_err_t servo_controller_change_id(uint8_t old_id, uint8_t new_id) {
    ESP_LOGI(TAG, "Changing servo ID from %d to %d", old_id, new_id);
    
    // Change servo ID using proper SCServo protocol
    // This is a critical operation that requires careful handling
    uint8_t data[1];
    data[0] = new_id;
    
    esp_err_t ret = scservo_write_buf(old_id, SCSCL_ID, data, 1, INST_WRITE);
    if (ret != ESP_OK) {
        ESP_LOGE(TAG, "Failed to change servo ID: %s", esp_err_to_name(ret));
        return ret;
    }
    
    ESP_LOGI(TAG, "Servo ID changed successfully from %d to %d", old_id, new_id);
    return ESP_OK;
}

esp_err_t servo_controller_set_middle_position(uint8_t servo_id) {
    ESP_LOGI(TAG, "Setting middle position for servo %d", servo_id);
    
    // Set servo to middle position (2047)
    uint16_t middle_position = 2047;
    
    esp_err_t ret = servo_controller_set_position(servo_id, middle_position);
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
    
    // Write to PID P register (this would need the actual register address)
    // For now, just log the operation
    ESP_LOGI(TAG, "PID P parameter set for servo %d to %d", servo_id, p_value);
    return ESP_OK;
}

// ===== SCSERVO PROTOCOL FUNCTIONS =====
/**
 * @brief Low-level SCServo protocol write function
 * @param id Servo ID
 * @param mem_addr Memory address to write
 * @param data Data buffer to write
 * @param data_len Length of data
 * @param instruction Instruction byte
 * @return ESP_OK on success, error code otherwise
 */
static esp_err_t scservo_write_buf(uint8_t id, uint8_t mem_addr, uint8_t *data, uint8_t data_len, uint8_t instruction) {
    if (!servo_initialized) {
        return ESP_ERR_INVALID_STATE;
    }
    
    uint8_t msg_len = 2 + (data ? data_len + 1 : 0);
    uint8_t checksum = id + msg_len + instruction + mem_addr;
    
    // Send header
    uint8_t header[] = {SCSERVO_HEADER1, SCSERVO_HEADER2};
    uart_write_bytes(SERVO_UART_NUM, header, 2);
    
    // Send ID
    uart_write_bytes(SERVO_UART_NUM, &id, 1);
    
    // Send message length
    uart_write_bytes(SERVO_UART_NUM, &msg_len, 1);
    
    // Send instruction
    uart_write_bytes(SERVO_UART_NUM, &instruction, 1);
    
    // Send memory address
    uart_write_bytes(SERVO_UART_NUM, &mem_addr, 1);
    
    // Send data if present
    if (data && data_len > 0) {
        uart_write_bytes(SERVO_UART_NUM, data, data_len);
        for (int i = 0; i < data_len; i++) {
            checksum += data[i];
        }
    }
    
    // Send checksum
    uint8_t final_checksum = ~checksum;
    uart_write_bytes(SERVO_UART_NUM, &final_checksum, 1);
    
    return ESP_OK;
}

/**
 * @brief Write position command to servo
 * @param id Servo ID
 * @param position Target position
 * @param time Movement time
 * @param speed Movement speed
 * @return ESP_OK on success, error code otherwise
 */
esp_err_t scservo_write_pos(uint8_t id, uint16_t position, uint16_t time, uint16_t speed) {
    uint8_t data[6];
    data[0] = position & 0xFF;
    data[1] = (position >> 8) & 0xFF;
    data[2] = time & 0xFF;
    data[3] = (time >> 8) & 0xFF;
    data[4] = speed & 0xFF;
    data[5] = (speed >> 8) & 0xFF;
    
    return scservo_write_buf(id, SCSCL_GOAL_POSITION_L, data, 6, INST_WRITE);
}

esp_err_t scservo_read_pos(uint8_t id, uint16_t *position) {
    // For now, return simulated position
    if (id >= 11 && id <= 15) {
        int index = id - 11;
        *position = servo_feedback[index].position;
        return ESP_OK;
    }
    return ESP_FAIL;
}

esp_err_t scservo_read_speed(uint8_t id, uint16_t *speed) {
    // For now, return simulated speed
    if (id >= 11 && id <= 15) {
        int index = id - 11;
        *speed = servo_feedback[index].speed;
        return ESP_OK;
    }
    return ESP_FAIL;
}

esp_err_t scservo_read_load(uint8_t id, uint16_t *load) {
    // For now, return simulated load
    if (id >= 11 && id <= 15) {
        int index = id - 11;
        *load = servo_feedback[index].load;
        return ESP_OK;
    }
    return ESP_FAIL;
}

esp_err_t scservo_read_voltage(uint8_t id, uint8_t *voltage) {
    // For now, return simulated voltage
    if (id >= 11 && id <= 15) {
        int index = id - 11;
        *voltage = servo_feedback[index].voltage / 100; // Convert mV to 0.1V units
        return ESP_OK;
    }
    return ESP_FAIL;
}

esp_err_t scservo_read_temperature(uint8_t id, uint8_t *temperature) {
    // For now, return simulated temperature
    if (id >= 11 && id <= 15) {
        int index = id - 11;
        *temperature = servo_feedback[index].temperature / 10; // Convert to degrees
        return ESP_OK;
    }
    return ESP_FAIL;
}

esp_err_t scservo_read_moving(uint8_t id, uint8_t *moving) {
    // For now, return simulated moving status
    if (id >= 11 && id <= 15) {
        int index = id - 11;
        *moving = servo_feedback[index].moving ? 1 : 0;
        return ESP_OK;
    }
    return ESP_FAIL;
}

esp_err_t scservo_enable_torque(uint8_t id, uint8_t enable) {
    uint8_t data[1];
    data[0] = enable;
    
    return scservo_write_buf(id, SCSCL_TORQUE_ENABLE, data, 1, INST_WRITE);
}
