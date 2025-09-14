#include "../inc/uart_controller.h"
#include "../inc/json_parser.h"
#include "../inc/motion_module.h"
#include "../inc/servo_controller.h"
#include "../inc/gimbal_controller.h"
#include "../inc/files_controller.h"
#include "../inc/esp_now_controller.h"
#include "../inc/wifi_controller.h"
#include "../inc/imu_controller.h"
#include "../inc/battery_controller.h"
#include "../inc/oled_controller.h"
#include "../inc/led_controller.h"
#include "../inc/ugv_config.h"
#include "../inc/mission_system.h"
#include "../inc/system_info.h"
#include <driver/uart.h>
#include <driver/gpio.h>
#include <freertos/FreeRTOS.h>
#include <freertos/task.h>
#include <freertos/queue.h>
#include <esp_log.h>
#include <esp_timer.h>
#include <string.h>
#include <cJSON.h>
#include <nvs_flash.h>
#include <esp_spiffs.h>

static const char *TAG = "UART_Controller";

// Global variables
static bool uart_initialized = false;
static bool echo_mode = false;
static uint32_t heartbeat_delay_ms = 1000;
static uint32_t last_cmd_time = 0;
static uint32_t feedback_interval_ms = 100;
static bool base_feedback_flow = true;

// Private function prototypes
static esp_err_t uart_parse_command(const char *data, size_t len);

esp_err_t uart_controller_init(void)
{
    if (uart_initialized) {
        ESP_LOGW(TAG, "UART controller already initialized");
        return ESP_OK;
    }

    ESP_LOGI(TAG, "Initializing UART controller...");

    // Configure UART parameters
    uart_config_t uart_config = {
        .baud_rate = UART_BAUD_RATE,
        .data_bits = UART_DATA_8_BITS,
        .parity = UART_PARITY_DISABLE,
        .stop_bits = UART_STOP_BITS_1,
        .flow_ctrl = UART_HW_FLOWCTRL_DISABLE,
        .rx_flow_ctrl_thresh = 0,
        .source_clk = UART_SCLK_APB,
    };

    // Install UART driver
    esp_err_t ret = uart_driver_install(UART_NUM, UART_BUFFER_SIZE * 2, UART_BUFFER_SIZE * 2, 20, NULL, 0);
    if (ret != ESP_OK) {
        ESP_LOGE(TAG, "Failed to install UART driver: %s", esp_err_to_name(ret));
        return ret;
    }

    // Set UART parameters
    ret = uart_param_config(UART_NUM, &uart_config);
    if (ret != ESP_OK) {
        ESP_LOGE(TAG, "Failed to configure UART parameters: %s", esp_err_to_name(ret));
        return ret;
    }

    // Set UART pins
    ret = uart_set_pin(UART_NUM, GPIO_NUM_1, GPIO_NUM_3, UART_PIN_NO_CHANGE, UART_PIN_NO_CHANGE);
    if (ret != ESP_OK) {
        ESP_LOGE(TAG, "Failed to set UART pins: %s", esp_err_to_name(ret));
        return ret;
    }

    uart_initialized = true;
    ESP_LOGI(TAG, "UART controller initialized successfully");
    return ESP_OK;
}

esp_err_t uart_controller_deinit(void)
{
    if (!uart_initialized) {
        return ESP_OK;
    }

    ESP_LOGI(TAG, "Deinitializing UART controller...");

    // Delete UART driver
    uart_driver_delete(UART_NUM);

    uart_initialized = false;
    ESP_LOGI(TAG, "UART controller deinitialized");
    return ESP_OK;
}

esp_err_t uart_controller_parse_command(const char *data, size_t len) {
    if (!uart_initialized) {
        return ESP_ERR_INVALID_STATE;
    }
    
    if (!data || len == 0) {
        return ESP_ERR_INVALID_ARG;
    }
    
    return uart_parse_command(data, len);
}

void uart_controller_task(void *pvParameters)
{
    ESP_LOGI(TAG, "UART controller task started");

    // Arduino-compatible JSON command buffering
    static char json_buffer[UART_BUFFER_SIZE];
    static size_t buffer_pos = 0;
    static uint32_t last_char_time = 0;
    static bool json_command_ready = false;
    const uint32_t JSON_TIMEOUT_MS = 100; // 100ms timeout for incomplete JSON
    
    char* temp_data = (char*) malloc(UART_BUFFER_SIZE);
    if (temp_data == NULL) {
        ESP_LOGE(TAG, "Failed to allocate UART buffer");
        vTaskDelete(NULL);
        return;
    }

    ESP_LOGI(TAG, "UART task using Arduino-compatible character-by-character buffering");

    while (1) {
        // Read available data (can be multiple characters)
        int len = uart_read_bytes(UART_NUM, temp_data, UART_BUFFER_SIZE - 1, pdMS_TO_TICKS(10));
        if (len > 0) {
            temp_data[len] = '\0';
            last_char_time = esp_timer_get_time() / 1000; // Convert to milliseconds
            
            // Process each character (Arduino-compatible logic)
            for (int i = 0; i < len && buffer_pos < sizeof(json_buffer) - 1; i++) {
                char received_char = temp_data[i];
                json_buffer[buffer_pos++] = received_char;
                
                // Detect end of JSON command (newline character)
                if (received_char == '\n') {
                    json_command_ready = true;
                    break; // Process this command first
                }
            }
        }
        
        // Process complete JSON command (Arduino-compatible)
        if (json_command_ready) {
            json_buffer[buffer_pos] = '\0';
            ESP_LOGI(TAG, "Complete JSON command received (%d bytes): %s", buffer_pos, json_buffer);
            
            // Parse and process command
            esp_err_t result = uart_parse_command(json_buffer, buffer_pos);
            if (result != ESP_OK) {
                ESP_LOGE(TAG, "Failed to parse JSON command: %s", esp_err_to_name(result));
            }
            
            // Reset for next command (Arduino-compatible)
            buffer_pos = 0;
            json_command_ready = false;
        }
        
        // Handle timeout for incomplete JSON (Arduino-compatible)
        uint32_t current_time = esp_timer_get_time() / 1000;
        if (buffer_pos > 0 && (current_time - last_char_time > JSON_TIMEOUT_MS)) {
            ESP_LOGW(TAG, "JSON timeout - clearing buffer (%d bytes): %.*s", buffer_pos, buffer_pos, json_buffer);
            buffer_pos = 0;
            json_command_ready = false;
        }

        vTaskDelay(pdMS_TO_TICKS(10)); // 100Hz UART processing rate
    }

    free(temp_data);
    vTaskDelete(NULL);
}

static esp_err_t uart_parse_command(const char *data, size_t len)
{
    if (data == NULL || len == 0) {
        return ESP_ERR_INVALID_ARG;
    }

    // Parse JSON command
    json_command_t cmd;
    esp_err_t result = json_parser_parse_command(data, &cmd);
    if (result != ESP_OK) {
        ESP_LOGE(TAG, "Failed to parse JSON command: %s", esp_err_to_name(result));
        return result;
    }

    ESP_LOGI(TAG, "Processing command type: %d", cmd.type);

    // Process command based on type
    switch (cmd.type) {
        case CMD_EMERGENCY_STOP: // {"T":0}
            ESP_LOGI(TAG, "Emergency stop command received");
            motion_module_emergency_stop();
            break;

        case CMD_SPEED_CTRL: // {"T":1,"L":0.5,"R":0.5}
            ESP_LOGI(TAG, "Speed control: L=%.2f, R=%.2f", cmd.data.speed_ctrl.L, cmd.data.speed_ctrl.R);
            motion_module_set_speeds(cmd.data.speed_ctrl.L, cmd.data.speed_ctrl.R);
            break;

        case CMD_PWM_INPUT: // {"T":11,"L":164,"R":164}
            ESP_LOGI(TAG, "PWM input: L=%d, R=%d", cmd.data.pwm_input.L, cmd.data.pwm_input.R);
            // Direct PWM control - bypass PID
            motion_module_set_motor(0, cmd.data.pwm_input.L > 0 ? 1 : -1, abs(cmd.data.pwm_input.L));
            motion_module_set_motor(1, cmd.data.pwm_input.R > 0 ? 1 : -1, abs(cmd.data.pwm_input.R));
            break;

        case CMD_ROS_CTRL: // {"T":13,"X":0.1,"Z":0.3}
            ESP_LOGI(TAG, "ROS control: X=%.2f, Z=%.2f", cmd.data.ros_ctrl.X, cmd.data.ros_ctrl.Z);
            motion_module_set_ros_motion(cmd.data.ros_ctrl.X, cmd.data.ros_ctrl.Z);
            break;

        case CMD_SET_MOTOR_PID: // {"T":2,"P":200,"I":2500,"D":0,"L":255}
            ESP_LOGI(TAG, "Set motor PID: P=%.2f, I=%.2f, D=%.2f, L=%.2f", 
                     cmd.data.pid_params.P, cmd.data.pid_params.I, cmd.data.pid_params.D, cmd.data.pid_params.L);
            motion_module_set_pid_params_direct(cmd.data.pid_params.P, cmd.data.pid_params.I, 
                                              cmd.data.pid_params.D, cmd.data.pid_params.L);
            break;

        case CMD_OLED_CTRL: // {"T":3,"lineNum":0,"Text":"Hello"}
            ESP_LOGI(TAG, "OLED control: line=%d, text=%s", cmd.data.oled_ctrl.lineNum, cmd.data.oled_ctrl.Text);
            oled_controller_control(cmd.data.oled_ctrl.lineNum, cmd.data.oled_ctrl.Text);
            break;

        case (uint16_t)CMD_OLED_DEFAULT: // {"T":-3}
            ESP_LOGI(TAG, "OLED default");
            oled_controller_reset_to_default();
            break;

        case CMD_MODULE_TYPE: // {"T":4,"cmd":0}
            ESP_LOGI(TAG, "Module type: %d", cmd.data.module_type.cmd);
            motion_module_set_type(cmd.data.module_type.cmd, 0);
            break;

        case CMD_SET_SPD_RATE: // {"T":138,"L":1,"R":1}
            ESP_LOGI(TAG, "Set speed rate: L=%.2f, R=%.2f", cmd.data.set_spd_rate.L, cmd.data.set_spd_rate.R);
            motion_module_set_speed_rates(cmd.data.set_spd_rate.L, cmd.data.set_spd_rate.R);
            break;

        case CMD_GET_SPD_RATE: // {"T":139}
            ESP_LOGI(TAG, "Get speed rate");
            {
                float left_rate, right_rate;
                if (motion_module_get_speed_rates(&left_rate, &right_rate) == ESP_OK) {
                    ESP_LOGI(TAG, "Speed rates: L=%.2f, R=%.2f", left_rate, right_rate);
                } else {
                    ESP_LOGE(TAG, "Failed to get speed rates");
                }
            }
            break;

        case CMD_SAVE_SPD_RATE: // {"T":140}
            ESP_LOGI(TAG, "Save speed rate");
            motion_module_save_speed_rates();
            break;

        case CMD_MM_TYPE_SET: // {"T":900,"main":1,"module":0}
            ESP_LOGI(TAG, "MM type set: main=%d, module=%d", cmd.data.mm_type_set.main, cmd.data.mm_type_set.module);
            motion_module_set_type(cmd.data.mm_type_set.main, cmd.data.mm_type_set.module);
            break;

        case CMD_GET_IMU_DATA: // {"T":126}
            ESP_LOGI(TAG, "Get IMU data");
            imu_controller_get_data_log();
            break;

        case CMD_CALI_IMU_STEP: // {"T":127}
            ESP_LOGI(TAG, "IMU calibration step");
            imu_controller_calibrate();
            break;

        case CMD_GET_IMU_OFFSET: // {"T":128}
            ESP_LOGI(TAG, "Get IMU offset");
            {
                float gx, gy, gz, ax, ay, az, cx, cy, cz;
                if (imu_controller_get_offset(&gx, &gy, &gz, &ax, &ay, &az, &cx, &cy, &cz) == ESP_OK) {
                    ESP_LOGI(TAG, "IMU offsets retrieved successfully");
                    
                    // Send feedback with offset values
                    cJSON *json = cJSON_CreateObject();
                    cJSON_AddNumberToObject(json, "T", FEEDBACK_IMU_OFFSET);
                    cJSON_AddNumberToObject(json, "gx", gx);
                    cJSON_AddNumberToObject(json, "gy", gy);
                    cJSON_AddNumberToObject(json, "gz", gz);
                    cJSON_AddNumberToObject(json, "ax", ax);
                    cJSON_AddNumberToObject(json, "ay", ay);
                    cJSON_AddNumberToObject(json, "az", az);
                    cJSON_AddNumberToObject(json, "cx", cx);
                    cJSON_AddNumberToObject(json, "cy", cy);
                    cJSON_AddNumberToObject(json, "cz", cz);
                    
                    char *json_string = cJSON_Print(json);
                    if (json_string) {
                        printf("%s\n", json_string);
                        free(json_string);
                    }
                    cJSON_Delete(json);
                } else {
                    ESP_LOGE(TAG, "Failed to get IMU offsets");
                }
            }
            break;

        case CMD_SET_IMU_OFFSET: // {"T":129,"gx":0,"gy":0,"gz":0,"ax":0,"ay":0,"az":0,"cx":0,"cy":0,"cz":0}
            ESP_LOGI(TAG, "Set IMU offset: gx=%.3f, gy=%.3f, gz=%.3f, ax=%.3f, ay=%.3f, az=%.3f, cx=%.3f, cy=%.3f, cz=%.3f",
                     cmd.data.set_imu_offset.gx, cmd.data.set_imu_offset.gy, cmd.data.set_imu_offset.gz,
                     cmd.data.set_imu_offset.ax, cmd.data.set_imu_offset.ay, cmd.data.set_imu_offset.az,
                     cmd.data.set_imu_offset.cx, cmd.data.set_imu_offset.cy, cmd.data.set_imu_offset.cz);
            imu_controller_set_offset(cmd.data.set_imu_offset.gx, cmd.data.set_imu_offset.gy, cmd.data.set_imu_offset.gz,
                                    cmd.data.set_imu_offset.ax, cmd.data.set_imu_offset.ay, cmd.data.set_imu_offset.az,
                                    cmd.data.set_imu_offset.cx, cmd.data.set_imu_offset.cy, cmd.data.set_imu_offset.cz);
            break;

        case CMD_BASE_FEEDBACK: // {"T":130}
            ESP_LOGI(TAG, "Base feedback");
            json_parser_send_base_feedback();
            break;

        case CMD_BASE_FEEDBACK_FLOW: // {"T":131,"cmd":1}
            ESP_LOGI(TAG, "Base feedback flow: %d", cmd.data.base_feedback_flow.cmd);
            base_feedback_flow = (cmd.data.base_feedback_flow.cmd == 1);
            break;

        case CMD_FEEDBACK_FLOW_INTERVAL: // {"T":142,"cmd":0}
            ESP_LOGI(TAG, "Feedback flow interval: %d", cmd.data.feedback_flow_interval.cmd);
            feedback_interval_ms = cmd.data.feedback_flow_interval.cmd;
            break;

        case CMD_UART_ECHO_MODE: // {"T":143,"cmd":0}
            ESP_LOGI(TAG, "UART echo mode: %d", cmd.data.uart_echo_mode.cmd);
            echo_mode = (cmd.data.uart_echo_mode.cmd == 1);
            break;

        case CMD_LED_CTRL: // {"T":132,"IO4":255,"IO5":255}
            ESP_LOGI(TAG, "LED control: IO4=%d, IO5=%d", cmd.data.led_ctrl.IO4, cmd.data.led_ctrl.IO5);
            led_controller_set_rgb(cmd.data.led_ctrl.IO4, cmd.data.led_ctrl.IO5, 0);
            break;

        case CMD_GIMBAL_CTRL_SIMPLE: // {"T":133,"X":45,"Y":45,"SPD":0,"ACC":0}
            ESP_LOGI(TAG, "Gimbal control simple: X=%.2f, Y=%.2f, SPD=%.2f, ACC=%.2f", 
                     cmd.data.gimbal_ctrl_simple.X, cmd.data.gimbal_ctrl_simple.Y, 
                     cmd.data.gimbal_ctrl_simple.SPD, cmd.data.gimbal_ctrl_simple.ACC);
            gimbal_controller_set_pan_tilt(cmd.data.gimbal_ctrl_simple.X, cmd.data.gimbal_ctrl_simple.Y);
            break;

        case CMD_GIMBAL_CTRL_MOVE: // {"T":134,"X":45,"Y":45,"SX":300,"SY":300}
            ESP_LOGI(TAG, "Gimbal control move: X=%.2f, Y=%.2f, SX=%.2f, SY=%.2f", 
                     cmd.data.gimbal_ctrl_move.X, cmd.data.gimbal_ctrl_move.Y, 
                     cmd.data.gimbal_ctrl_move.SX, cmd.data.gimbal_ctrl_move.SY);
            gimbal_controller_smooth_move(
                gimbal_controller_degrees_to_pulse(cmd.data.gimbal_ctrl_move.X, 0), // pan
                gimbal_controller_degrees_to_pulse(cmd.data.gimbal_ctrl_move.Y, 1), // tilt
                cmd.data.gimbal_ctrl_move.SX, // speed
                cmd.data.gimbal_ctrl_move.SY  // acceleration
            );
            break;

        case CMD_GIMBAL_CTRL_STOP: // {"T":135}
            ESP_LOGI(TAG, "Gimbal control stop");
            gimbal_controller_stop();
            break;

        case CMD_HEART_BEAT_SET: // {"T":136,"cmd":3000}
            ESP_LOGI(TAG, "Heart beat set: %d", cmd.data.heart_beat_set.cmd);
            heartbeat_delay_ms = cmd.data.heart_beat_set.cmd;
            break;

        case CMD_GIMBAL_STEADY: // {"T":137,"s":1,"y":0}
            ESP_LOGI(TAG, "Gimbal steady: s=%d, y=%d", cmd.data.gimbal_steady.s, cmd.data.gimbal_steady.y);
            if (cmd.data.gimbal_steady.s == 1) {
                // Enable steady mode
                gimbal_controller_enable_steady_mode(true);
            } else {
                // Disable steady mode
                gimbal_controller_enable_steady_mode(false);
            }
            break;

        case CMD_GIMBAL_USER_CTRL: // {"T":141,"X":0,"Y":0,"SPD":300}
            ESP_LOGI(TAG, "Gimbal user control: X=%.2f, Y=%.2f, SPD=%.2f", 
                     cmd.data.gimbal_user_ctrl.X, cmd.data.gimbal_user_ctrl.Y, cmd.data.gimbal_user_ctrl.SPD);
            gimbal_controller_set_velocity(cmd.data.gimbal_user_ctrl.X, cmd.data.gimbal_user_ctrl.Y);
            break;

        case CMD_ARM_CTRL_UI: // {"T":144,"E":100,"Z":0,"R":0}
            ESP_LOGI(TAG, "Arm control UI: E=%.2f, Z=%.2f, R=%.2f", 
                     cmd.data.arm_ctrl_ui.E, cmd.data.arm_ctrl_ui.Z, cmd.data.arm_ctrl_ui.R);
            servo_controller_arm_ui_control(cmd.data.arm_ctrl_ui.E, cmd.data.arm_ctrl_ui.Z, cmd.data.arm_ctrl_ui.R);
            break;

        case CMD_SINGLE_JOINT_CTRL: // {"T":101,"joint":0,"rad":0,"spd":0,"acc":10}
            ESP_LOGI(TAG, "Single joint control: joint=%d, rad=%.2f, spd=%.2f, acc=%.2f", 
                     cmd.data.single_joint_ctrl.joint, cmd.data.single_joint_ctrl.rad, 
                     cmd.data.single_joint_ctrl.spd, cmd.data.single_joint_ctrl.acc);
            {
                // Get current joint angles
                arm_joint_angles_t angles;
                if (servo_controller_get_joint_angles(&angles) == ESP_OK) {
                    // Update the specified joint
                    switch (cmd.data.single_joint_ctrl.joint) {
                        case 0: angles.base = cmd.data.single_joint_ctrl.rad; break;
                        case 1: angles.shoulder = cmd.data.single_joint_ctrl.rad; break;
                        case 2: angles.elbow = cmd.data.single_joint_ctrl.rad; break;
                        case 3: angles.gripper = cmd.data.single_joint_ctrl.rad; break;
                        default: ESP_LOGW(TAG, "Invalid joint ID: %d", cmd.data.single_joint_ctrl.joint); break;
                    }
                    // Set the updated joint angles
                    servo_controller_set_joint_angles(&angles, (uint16_t)cmd.data.single_joint_ctrl.spd);
                }
            }
            break;

        case CMD_JOINTS_RAD_CTRL: // {"T":102,"base":0,"shoulder":0,"elbow":1.57,"hand":1.57,"spd":0,"acc":10}
            ESP_LOGI(TAG, "Joints rad control: base=%.2f, shoulder=%.2f, elbow=%.2f, hand=%.2f, spd=%.2f, acc=%.2f", 
                     cmd.data.joints_rad_ctrl.base, cmd.data.joints_rad_ctrl.shoulder, cmd.data.joints_rad_ctrl.elbow, 
                     cmd.data.joints_rad_ctrl.hand, cmd.data.joints_rad_ctrl.spd, cmd.data.joints_rad_ctrl.acc);
            {
                arm_joint_angles_t angles;
                angles.base = cmd.data.joints_rad_ctrl.base;
                angles.shoulder = cmd.data.joints_rad_ctrl.shoulder;
                angles.elbow = cmd.data.joints_rad_ctrl.elbow;
                angles.gripper = cmd.data.joints_rad_ctrl.hand;
                servo_controller_set_joint_angles(&angles, (uint16_t)cmd.data.joints_rad_ctrl.spd);
            }
            break;

        case CMD_XYZT_GOAL_CTRL: // {"T":104,"x":235,"y":0,"z":234,"t":3.14,"spd":0.25}
            ESP_LOGI(TAG, "XYZT goal control: x=%.2f, y=%.2f, z=%.2f, t=%.2f, spd=%.2f", 
                     cmd.data.xyzt_goal_ctrl.x, cmd.data.xyzt_goal_ctrl.y, cmd.data.xyzt_goal_ctrl.z, 
                     cmd.data.xyzt_goal_ctrl.t, cmd.data.xyzt_goal_ctrl.spd);
            {
                arm_pose_t pose;
                pose.x = cmd.data.xyzt_goal_ctrl.x;
                pose.y = cmd.data.xyzt_goal_ctrl.y;
                pose.z = cmd.data.xyzt_goal_ctrl.z;
                pose.yaw = cmd.data.xyzt_goal_ctrl.t;
                pose.roll = 0.0f;  // Default values
                pose.pitch = 0.0f;
                servo_controller_set_pose(&pose, (uint16_t)(cmd.data.xyzt_goal_ctrl.spd * 1000)); // Convert to appropriate speed units
            }
            break;

        case CMD_XYZT_DIRECT_CTRL: // {"T":1041,"x":235,"y":0,"z":234,"t":3.14}
            ESP_LOGI(TAG, "XYZT direct control: x=%.2f, y=%.2f, z=%.2f, t=%.2f", 
                     cmd.data.xyzt_direct_ctrl.x, cmd.data.xyzt_direct_ctrl.y, cmd.data.xyzt_direct_ctrl.z, 
                     cmd.data.xyzt_direct_ctrl.t);
            {
                arm_pose_t pose;
                pose.x = cmd.data.xyzt_direct_ctrl.x;
                pose.y = cmd.data.xyzt_direct_ctrl.y;
                pose.z = cmd.data.xyzt_direct_ctrl.z;
                pose.yaw = cmd.data.xyzt_direct_ctrl.t;
                pose.roll = 0.0f;  // Default values
                pose.pitch = 0.0f;
                // Direct control without interpolation - use maximum speed
                servo_controller_set_pose(&pose, 1000);
            }
            break;

        case CMD_EOAT_HAND_CTRL: // {"T":106,"cmd":1.57,"spd":0,"acc":0}
            ESP_LOGI(TAG, "EOAT hand control: cmd=%.2f, spd=%.2f, acc=%.2f", 
                     cmd.data.eoat_hand_ctrl.cmd, cmd.data.eoat_hand_ctrl.spd, cmd.data.eoat_hand_ctrl.acc);
            {
                // Get current joint angles and update gripper
                arm_joint_angles_t angles;
                if (servo_controller_get_joint_angles(&angles) == ESP_OK) {
                    angles.gripper = cmd.data.eoat_hand_ctrl.cmd;
                    servo_controller_set_joint_angles(&angles, (uint16_t)cmd.data.eoat_hand_ctrl.spd);
                }
            }
            break;

        case CMD_EOAT_GRAB_TORQUE: // {"T":107,"tor":200}
            ESP_LOGI(TAG, "EOAT grab torque: tor=%.2f", cmd.data.eoat_grab_torque.tor);
            // Use servo ID 4 for gripper (assuming gripper is servo ID 4)
            servo_controller_set_torque(4, (uint16_t)cmd.data.eoat_grab_torque.tor);
            break;

        case CMD_SET_JOINT_PID: // {"T":108,"joint":3,"p":16,"i":0}
            ESP_LOGI(TAG, "Set joint PID: joint=%d, p=%.2f, i=%.2f", 
                     cmd.data.set_joint_pid.joint, cmd.data.set_joint_pid.p, cmd.data.set_joint_pid.i);
            servo_controller_set_joint_pid(cmd.data.set_joint_pid.joint, cmd.data.set_joint_pid.p, cmd.data.set_joint_pid.i);
            break;

        case CMD_RESET_PID: // {"T":109}
            ESP_LOGI(TAG, "Reset PID");
            servo_controller_reset_pid();
            break;

        case CMD_MOVE_INIT: // {"T":100}
            ESP_LOGI(TAG, "Move init");
            servo_controller_move_init();
            break;

        case CMD_SERVO_RAD_FEEDBACK: // {"T":105}
            ESP_LOGI(TAG, "Servo rad feedback");
            servo_controller_get_feedback();
            break;

        case CMD_SCAN_FILES: // {"T":200}
            ESP_LOGI(TAG, "Scan files");
            files_controller_scan_contents();
            break;

        case CMD_CREATE_FILE: // {"T":201,"name":"file.txt","content":"inputContentHere."}
            ESP_LOGI(TAG, "Create file: name=%s", cmd.data.create_file.name);
            files_controller_create_file(cmd.data.create_file.name, cmd.data.create_file.content);
            break;

        case CMD_READ_FILE: // {"T":202,"name":"file.txt"}
            ESP_LOGI(TAG, "Read file: name=%s", cmd.data.read_file.name);
            {
                char content[1024];
                esp_err_t result = files_controller_read_file(cmd.data.read_file.name, content, sizeof(content));
                if (result == ESP_OK) {
                    ESP_LOGI(TAG, "File content: %s", content);
                } else {
                    ESP_LOGE(TAG, "Failed to read file: %s", esp_err_to_name(result));
                }
            }
            break;

        case CMD_DELETE_FILE: // {"T":203,"name":"file.txt"}
            ESP_LOGI(TAG, "Delete file: name=%s", cmd.data.delete_file.name);
            files_controller_delete_file(cmd.data.delete_file.name);
            break;

        case CMD_APPEND_LINE: // {"T":204,"name":"file.txt","content":"inputContentHere."}
            ESP_LOGI(TAG, "Append line: name=%s", cmd.data.append_line.name);
            files_controller_append_line(cmd.data.append_line.name, cmd.data.append_line.content);
            break;

        case CMD_INSERT_LINE: // {"T":205,"name":"file.txt","lineNum":3,"content":"content"}
            ESP_LOGI(TAG, "Insert line: name=%s, lineNum=%d", cmd.data.insert_line.name, cmd.data.insert_line.lineNum);
            files_controller_insert_line(cmd.data.insert_line.name, cmd.data.insert_line.lineNum, cmd.data.insert_line.content);
            break;

        case CMD_REPLACE_LINE: // {"T":206,"name":"file.txt","lineNum":3,"content":"Content"}
            ESP_LOGI(TAG, "Replace line: name=%s, lineNum=%d", cmd.data.replace_line.name, cmd.data.replace_line.lineNum);
            files_controller_replace_line(cmd.data.replace_line.name, cmd.data.replace_line.lineNum, cmd.data.replace_line.content);
            break;

        case CMD_READ_LINE: // {"T":207,"name":"file.txt","lineNum":3}
            ESP_LOGI(TAG, "Read line: name=%s, lineNum=%d", cmd.data.read_line.name, cmd.data.read_line.lineNum);
            {
                char content[256];
                esp_err_t result = files_controller_read_line(cmd.data.read_line.name, cmd.data.read_line.lineNum, content, sizeof(content));
                if (result == ESP_OK) {
                    ESP_LOGI(TAG, "Line content: %s", content);
                } else {
                    ESP_LOGE(TAG, "Failed to read line: %s", esp_err_to_name(result));
                }
            }
            break;

        case CMD_DELETE_LINE: // {"T":208,"name":"file.txt","lineNum":3}
            ESP_LOGI(TAG, "Delete line: name=%s, lineNum=%d", cmd.data.delete_line.name, cmd.data.delete_line.lineNum);
            files_controller_delete_line(cmd.data.delete_line.name, cmd.data.delete_line.lineNum);
            break;

        case CMD_TORQUE_CTRL: // {"T":210,"cmd":1}
            ESP_LOGI(TAG, "Torque control: cmd=%d", cmd.data.torque_ctrl.cmd);
            servo_controller_torque_control(cmd.data.torque_ctrl.cmd == 1);
            break;

        case CMD_CREATE_MISSION: // {"T":220,"name":"mission_a","intro":"test mission created in flash."}
            ESP_LOGI(TAG, "Create mission: name=%s", cmd.data.create_mission.name);
            create_mission(cmd.data.create_mission.name, cmd.data.create_mission.intro);
            break;

        case CMD_MISSION_CONTENT: // {"T":221,"name":"mission_a"}
            ESP_LOGI(TAG, "Mission content: name=%s", cmd.data.mission_content.name);
            mission_content(cmd.data.mission_content.name);
            break;

        case CMD_APPEND_STEP_JSON: // {"T":222,"name":"mission_a","step":"{\"T\":104,\"x\":235,\"y\":0,\"z\":234,\"t\":3.14,\"spd\":0.25}"}
            ESP_LOGI(TAG, "Append step JSON: name=%s", cmd.data.append_step_json.name);
            append_step_json(cmd.data.append_step_json.name, cmd.data.append_step_json.step);
            break;

        case CMD_APPEND_STEP_FB: // {"T":223,"name":"mission_a","spd":0.25}
            ESP_LOGI(TAG, "Append step FB: name=%s, spd=%.2f", cmd.data.append_step_fb.name, cmd.data.append_step_fb.spd);
            append_step_feedback(cmd.data.append_step_fb.name, cmd.data.append_step_fb.spd);
            break;

        case CMD_APPEND_DELAY: // {"T":224,"name":"mission_a","delay":3000}
            ESP_LOGI(TAG, "Append delay: name=%s, delay=%d", cmd.data.append_delay.name, cmd.data.append_delay.delay);
            append_delay_cmd(cmd.data.append_delay.name, cmd.data.append_delay.delay);
            break;

        case CMD_INSERT_STEP_JSON: // {"T":225,"name":"mission_a","stepNum":3,"step":"{\"T\":104,\"x\":235,\"y\":0,\"z\":234,\"t\":3.14,\"spd\":0.25}"}
            ESP_LOGI(TAG, "Insert step JSON: name=%s, stepNum=%d", cmd.data.insert_step_json.name, cmd.data.insert_step_json.stepNum);
            insert_step_json(cmd.data.insert_step_json.name, cmd.data.insert_step_json.stepNum, cmd.data.insert_step_json.step);
            break;

        case CMD_INSERT_STEP_FB: // {"T":226,"name":"mission_a","stepNum":3,"spd":0.25}
            ESP_LOGI(TAG, "Insert step FB: name=%s, stepNum=%d, spd=%.2f", 
                     cmd.data.insert_step_fb.name, cmd.data.insert_step_fb.stepNum, cmd.data.insert_step_fb.spd);
            insert_step_feedback(cmd.data.insert_step_fb.name, cmd.data.insert_step_fb.stepNum, cmd.data.insert_step_fb.spd);
            break;

        case CMD_INSERT_DELAY: // {"T":227,"stepNum":3,"delay":3000}
            ESP_LOGI(TAG, "Insert delay: stepNum=%d, delay=%d", cmd.data.insert_delay.stepNum, cmd.data.insert_delay.delay);
            insert_delay_cmd("", cmd.data.insert_delay.stepNum, cmd.data.insert_delay.delay);
            break;

        case CMD_REPLACE_STEP_JSON: // {"T":228,"name":"mission_a","stepNum":3,"step":"{\"T\":114,\"led\":255}"}
            ESP_LOGI(TAG, "Replace step JSON: name=%s, stepNum=%d", cmd.data.replace_step_json.name, cmd.data.replace_step_json.stepNum);
            replace_step_json(cmd.data.replace_step_json.name, cmd.data.replace_step_json.stepNum, cmd.data.replace_step_json.step);
            break;

        case CMD_REPLACE_STEP_FB: // {"T":229,"name":"mission_a","stepNum":3,"spd":0.25}
            ESP_LOGI(TAG, "Replace step FB: name=%s, stepNum=%d, spd=%.2f", 
                     cmd.data.replace_step_fb.name, cmd.data.replace_step_fb.stepNum, cmd.data.replace_step_fb.spd);
            replace_step_feedback(cmd.data.replace_step_fb.name, cmd.data.replace_step_fb.stepNum, cmd.data.replace_step_fb.spd);
            break;

        case CMD_REPLACE_DELAY: // {"T":230,"name":"mission_a","stepNum":3,"delay":3000}
            ESP_LOGI(TAG, "Replace delay: name=%s, stepNum=%d, delay=%d", 
                     cmd.data.replace_delay.name, cmd.data.replace_delay.stepNum, cmd.data.replace_delay.delay);
            replace_delay_cmd(cmd.data.replace_delay.name, cmd.data.replace_delay.stepNum, cmd.data.replace_delay.delay);
            break;

        case CMD_DELETE_STEP: // {"T":231,"name":"mission_a","stepNum":3}
            ESP_LOGI(TAG, "Delete step: name=%s, stepNum=%d", cmd.data.delete_step.name, cmd.data.delete_step.stepNum);
            delete_step(cmd.data.delete_step.name, cmd.data.delete_step.stepNum);
            break;

        case CMD_MOVE_TO_STEP: // {"T":241,"name":"mission_a","stepNum":3}
            ESP_LOGI(TAG, "Move to step: name=%s, stepNum=%d", cmd.data.move_to_step.name, cmd.data.move_to_step.stepNum);
            move_to_step(cmd.data.move_to_step.name, cmd.data.move_to_step.stepNum);
            break;

        case CMD_MISSION_PLAY: // {"T":242,"name":"mission_a","times":3}
            ESP_LOGI(TAG, "Mission play: name=%s, times=%d", cmd.data.mission_play.name, cmd.data.mission_play.times);
            mission_play(cmd.data.mission_play.name, cmd.data.mission_play.times);
            break;

        case CMD_BROADCAST_FOLLOWER: // {"T":300,"mode":1,"mac":"FF:FF:FF:FF:FF:FF"}
            ESP_LOGI(TAG, "Broadcast follower: mode=%d, mac=%s", cmd.data.broadcast_follower.mode, cmd.data.broadcast_follower.mac);
            esp_now_controller_set_broadcast_mode(cmd.data.broadcast_follower.mode, cmd.data.broadcast_follower.mac);
            break;

        case CMD_ESP_NOW_CONFIG: // {"T":301,"mode":3}
            ESP_LOGI(TAG, "ESP-NOW config: mode=%d", cmd.data.esp_now_config.mode);
            esp_now_controller_set_mode(cmd.data.esp_now_config.mode);
            break;

        case CMD_GET_MAC_ADDRESS: // {"T":302}
            ESP_LOGI(TAG, "Get MAC address");
            {
                char mac_str[18];
                if (esp_now_controller_get_mac_address(mac_str) == ESP_OK) {
                    ESP_LOGI(TAG, "Device MAC address: %s", mac_str);
                } else {
                    ESP_LOGE(TAG, "Failed to get MAC address");
                }
            }
            break;

        case CMD_ESP_NOW_ADD_FOLLOWER: // {"T":303,"mac":"FF:FF:FF:FF:FF:FF"}
            ESP_LOGI(TAG, "ESP-NOW add follower: mac=%s", cmd.data.esp_now_add_follower.mac);
            {
                uint8_t mac[6];
                if (esp_now_string_to_mac(cmd.data.esp_now_add_follower.mac, mac) == ESP_OK) {
                    esp_now_controller_add_peer(mac);
                } else {
                    ESP_LOGE(TAG, "Invalid MAC address format: %s", cmd.data.esp_now_add_follower.mac);
                }
            }
            break;

        case CMD_ESP_NOW_REMOVE_FOLLOWER: // {"T":304,"mac":"FF:FF:FF:FF:FF:FF"}
            ESP_LOGI(TAG, "ESP-NOW remove follower: mac=%s", cmd.data.esp_now_remove_follower.mac);
            {
                uint8_t mac[6];
                if (esp_now_string_to_mac(cmd.data.esp_now_remove_follower.mac, mac) == ESP_OK) {
                    esp_now_controller_remove_peer(mac);
                } else {
                    ESP_LOGE(TAG, "Invalid MAC address format: %s", cmd.data.esp_now_remove_follower.mac);
                }
            }
            break;

        case CMD_ESP_NOW_GROUP_CTRL: // {"T":305,"dev":0,"b":0,"s":0,"e":1.57,"h":1.57,"cmd":0,"megs":"hello!"}
            ESP_LOGI(TAG, "ESP-NOW group control: dev=%d, b=%.2f, s=%.2f, e=%.2f, h=%.2f, cmd=%d, megs=%s", 
                     cmd.data.esp_now_group_ctrl.dev, cmd.data.esp_now_group_ctrl.b, cmd.data.esp_now_group_ctrl.s, 
                     cmd.data.esp_now_group_ctrl.e, cmd.data.esp_now_group_ctrl.h, cmd.data.esp_now_group_ctrl.cmd, 
                     cmd.data.esp_now_group_ctrl.megs);
            {
                esp_now_message_t msg;
                msg.dev_code = cmd.data.esp_now_group_ctrl.dev;
                msg.base = cmd.data.esp_now_group_ctrl.b;
                msg.shoulder = cmd.data.esp_now_group_ctrl.s;
                msg.elbow = cmd.data.esp_now_group_ctrl.e;
                msg.hand = cmd.data.esp_now_group_ctrl.h;
                msg.cmd = cmd.data.esp_now_group_ctrl.cmd;
                strncpy(msg.message, cmd.data.esp_now_group_ctrl.megs, sizeof(msg.message) - 1);
                msg.message[sizeof(msg.message) - 1] = '\0';
                esp_now_controller_send_message(&msg);
            }
            break;

        case CMD_ESP_NOW_SINGLE: // {"T":306,"mac":"FF:FF:FF:FF:FF:FF","dev":0,"b":0,"s":0,"e":1.57,"h":1.57,"cmd":0,"megs":"hello!"}
            ESP_LOGI(TAG, "ESP-NOW single: mac=%s, dev=%d, b=%.2f, s=%.2f, e=%.2f, h=%.2f, cmd=%d, megs=%s", 
                     cmd.data.esp_now_single.mac, cmd.data.esp_now_single.dev, cmd.data.esp_now_single.b, 
                     cmd.data.esp_now_single.s, cmd.data.esp_now_single.e, cmd.data.esp_now_single.h, 
                     cmd.data.esp_now_single.cmd, cmd.data.esp_now_single.megs);
            {
                // First add the peer if not already added
                uint8_t mac[6];
                if (esp_now_string_to_mac(cmd.data.esp_now_single.mac, mac) == ESP_OK) {
                    esp_now_controller_add_peer(mac);
                    
                    // Then send the message
                    esp_now_message_t msg;
                    msg.dev_code = cmd.data.esp_now_single.dev;
                    msg.base = cmd.data.esp_now_single.b;
                    msg.shoulder = cmd.data.esp_now_single.s;
                    msg.elbow = cmd.data.esp_now_single.e;
                    msg.hand = cmd.data.esp_now_single.h;
                    msg.cmd = cmd.data.esp_now_single.cmd;
                    strncpy(msg.message, cmd.data.esp_now_single.megs, sizeof(msg.message) - 1);
                    msg.message[sizeof(msg.message) - 1] = '\0';
                    esp_now_controller_send_message(&msg);
                } else {
                    ESP_LOGE(TAG, "Invalid MAC address format: %s", cmd.data.esp_now_single.mac);
                }
            }
            break;

        case CMD_WIFI_ON_BOOT: // {"T":401,"cmd":3}
            ESP_LOGI(TAG, "WiFi on boot: cmd=%d", cmd.data.wifi_on_boot.cmd);
            wifi_controller_set_boot_mode(cmd.data.wifi_on_boot.cmd);
            break;

        case CMD_SET_AP: // {"T":402,"ssid":"RoArm-M2","password":"12345678"}
            ESP_LOGI(TAG, "Set AP: ssid=%s", cmd.data.set_ap.ssid);
            wifi_controller_set_ap_config(cmd.data.set_ap.ssid, cmd.data.set_ap.password);
            break;

        case CMD_SET_STA: // {"T":403,"ssid":"JSBZY-2.4G","password":"waveshare0755"}
            ESP_LOGI(TAG, "Set STA: ssid=%s", cmd.data.set_sta.ssid);
            wifi_controller_set_sta_config(cmd.data.set_sta.ssid, cmd.data.set_sta.password);
            break;

        case CMD_WIFI_APSTA: // {"T":404,"ap_ssid":"RoArm-M2","ap_password":"12345678","sta_ssid":"JSBZY-2.4G","sta_password":"waveshare0755"}
            ESP_LOGI(TAG, "WiFi APSTA: ap_ssid=%s, sta_ssid=%s", cmd.data.wifi_apsta.ap_ssid, cmd.data.wifi_apsta.sta_ssid);
            wifi_controller_set_ap_sta_config(cmd.data.wifi_apsta.ap_ssid, cmd.data.wifi_apsta.ap_password,
                                            cmd.data.wifi_apsta.sta_ssid, cmd.data.wifi_apsta.sta_password);
            break;

        case CMD_WIFI_INFO: // {"T":405}
            ESP_LOGI(TAG, "WiFi info");
            wifi_controller_get_info();
            break;

        case CMD_WIFI_CONFIG_CREATE_BY_STATUS: // {"T":406}
            ESP_LOGI(TAG, "WiFi config create by status");
            wifi_controller_create_config_by_status();
            break;

        case CMD_WIFI_CONFIG_CREATE_BY_INPUT: // {"T":407,"mode":3,"ap_ssid":"RoArm-M2","ap_password":"12345678","sta_ssid":"JSBZY-2.4G","sta_password":"waveshare0755"}
            ESP_LOGI(TAG, "WiFi config create by input: mode=%d", cmd.data.wifi_config_create_by_input.mode);
            wifi_controller_create_config_by_input(cmd.data.wifi_config_create_by_input.mode,
                                                 cmd.data.wifi_config_create_by_input.ap_ssid,
                                                 cmd.data.wifi_config_create_by_input.ap_password,
                                                 cmd.data.wifi_config_create_by_input.sta_ssid,
                                                 cmd.data.wifi_config_create_by_input.sta_password);
            break;

        case CMD_WIFI_STOP: // {"T":408}
            ESP_LOGI(TAG, "WiFi stop");
            wifi_controller_stop();
            break;

        case CMD_SET_SERVO_ID: // {"T":501,"raw":1,"new":11}
            ESP_LOGI(TAG, "Set servo ID: raw=%d, new=%d", cmd.data.set_servo_id.raw, cmd.data.set_servo_id.new_id);
            servo_controller_change_id(cmd.data.set_servo_id.raw, cmd.data.set_servo_id.new_id);
            break;

        case CMD_SET_MIDDLE: // {"T":502,"id":11}
            ESP_LOGI(TAG, "Set middle: id=%d", cmd.data.set_middle.id);
            servo_controller_set_middle_position(cmd.data.set_middle.id);
            break;

        case CMD_SET_SERVO_PID: // {"T":503,"id":14,"p":16}
            ESP_LOGI(TAG, "Set servo PID: id=%d, p=%.2f", cmd.data.set_servo_pid.id, cmd.data.set_servo_pid.p);
            servo_controller_set_servo_pid(cmd.data.set_servo_pid.id, cmd.data.set_servo_pid.p);
            break;

        case CMD_REBOOT: // {"T":600}
            ESP_LOGI(TAG, "Reboot");
            esp_restart();
            break;

        case CMD_FREE_FLASH_SPACE: // {"T":601}
            ESP_LOGI(TAG, "Free flash space");
            system_info_get_flash_space();
            break;

        case CMD_BOOT_MISSION_INFO: // {"T":602}
            ESP_LOGI(TAG, "Boot mission info");
            system_info_get_boot_mission();
            break;

        case CMD_RESET_BOOT_MISSION: // {"T":603}
            ESP_LOGI(TAG, "Reset boot mission");
            system_info_reset_boot_mission();
            break;

        case CMD_NVS_CLEAR: // {"T":604}
            ESP_LOGI(TAG, "NVS clear");
            nvs_flash_erase();
            vTaskDelay(pdMS_TO_TICKS(1000));
            nvs_flash_init();
            break;

        case CMD_INFO_PRINT: // {"T":605,"cmd":1}
            ESP_LOGI(TAG, "Info print: cmd=%d", cmd.data.info_print.cmd);
            system_info_set_print_mode(cmd.data.info_print.cmd);
            break;

        case CMD_RESET_EMERGENCY: // {"T":999}
            ESP_LOGI(TAG, "Reset emergency");
            motion_module_reset_emergency();
            break;

        default:
            ESP_LOGW(TAG, "Unknown command type: %d", cmd.type);
            break;
    }

    // Update last command time
    last_cmd_time = xTaskGetTickCount() * portTICK_PERIOD_MS;

    return ESP_OK;
}
