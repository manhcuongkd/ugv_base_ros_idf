#ifndef JSON_PARSER_H
#define JSON_PARSER_H

#include <esp_err.h>
#include <stdint.h>
#include <stdbool.h>
#include <cJSON.h>
#include "ugv_config.h"

#ifdef __cplusplus
extern "C" {
#endif

// Feedback command definitions - matches original Arduino project
#define FEEDBACK_BASE_INFO 1001
#define FEEDBACK_IMU_DATA 1002
#define FEEDBACK_IMU_OFFSET 1006
#define CMD_ESP_NOW_RECV 1003
#define CMD_ESP_NOW_SEND 1004
#define CMD_BUS_SERVO_ERROR 1005

// Command type definitions - matches original Arduino project
#define CMD_EMERGENCY_STOP 0
#define CMD_SPEED_CTRL 1
#define CMD_SET_MOTOR_PID 2
#define CMD_OLED_CTRL 3
#define CMD_MODULE_TYPE 4
#define CMD_OLED_DEFAULT -3
#define CMD_PWM_INPUT 11
#define CMD_ROS_CTRL 13
#define CMD_MOVE_INIT 100
#define CMD_SINGLE_JOINT_CTRL 101
#define CMD_JOINTS_RAD_CTRL 102
#define CMD_SINGLE_AXIS_CTRL 103
#define CMD_XYZT_GOAL_CTRL 104
#define CMD_XYZT_DIRECT_CTRL 1041
#define CMD_SERVO_RAD_FEEDBACK 105
#define CMD_EOAT_HAND_CTRL 106
#define CMD_EOAT_GRAB_TORQUE 107
#define CMD_SET_JOINT_PID 108
#define CMD_RESET_PID 109
#define CMD_SET_NEW_X 110
#define CMD_DELAY_MILLIS 111
#define CMD_DYNAMIC_ADAPTATION 112
#define CMD_SWITCH_CTRL 113
#define CMD_LIGHT_CTRL 114
#define CMD_SWITCH_OFF 115
#define CMD_SINGLE_JOINT_ANGLE 121
#define CMD_JOINTS_ANGLE_CTRL 122
#define CMD_CONSTANT_CTRL 123
#define CMD_EOAT_TYPE 124
#define CMD_CONFIG_EOAT 125
#define CMD_GET_IMU_DATA 126
#define CMD_CALI_IMU_STEP 127
#define CMD_GET_IMU_OFFSET 128
#define CMD_SET_IMU_OFFSET 129
#define CMD_BASE_FEEDBACK 130
#define CMD_BASE_FEEDBACK_FLOW 131
#define CMD_LED_CTRL 132
#define CMD_GIMBAL_CTRL_SIMPLE 133
#define CMD_GIMBAL_CTRL_MOVE 134
#define CMD_GIMBAL_CTRL_STOP 135
#define CMD_HEART_BEAT_SET 136
#define CMD_GIMBAL_STEADY 137
#define CMD_SET_SPD_RATE 138
#define CMD_GET_SPD_RATE 139
#define CMD_SAVE_SPD_RATE 140
#define CMD_GIMBAL_USER_CTRL 141
#define CMD_FEEDBACK_FLOW_INTERVAL 142
#define CMD_UART_ECHO_MODE 143
#define CMD_ARM_CTRL_UI 144
#define CMD_SCAN_FILES 200
#define CMD_CREATE_FILE 201
#define CMD_READ_FILE 202
#define CMD_DELETE_FILE 203
#define CMD_APPEND_LINE 204
#define CMD_INSERT_LINE 205
#define CMD_REPLACE_LINE 206
#define CMD_READ_LINE 207
#define CMD_DELETE_LINE 208
#define CMD_TORQUE_CTRL 210
#define CMD_CREATE_MISSION 220
#define CMD_MISSION_CONTENT 221
#define CMD_APPEND_STEP_JSON 222
#define CMD_APPEND_STEP_FB 223
#define CMD_APPEND_DELAY 224
#define CMD_INSERT_STEP_JSON 225
#define CMD_INSERT_STEP_FB 226
#define CMD_INSERT_DELAY 227
#define CMD_REPLACE_STEP_JSON 228
#define CMD_REPLACE_STEP_FB 229
#define CMD_REPLACE_DELAY 230
#define CMD_DELETE_STEP 231
#define CMD_MOVE_TO_STEP 241
#define CMD_MISSION_PLAY 242
#define CMD_BROADCAST_FOLLOWER 300
#define CMD_ESP_NOW_CONFIG 301
#define CMD_GET_MAC_ADDRESS 302
#define CMD_ESP_NOW_ADD_FOLLOWER 303
#define CMD_ESP_NOW_REMOVE_FOLLOWER 304
#define CMD_ESP_NOW_GROUP_CTRL 305
#define CMD_ESP_NOW_SINGLE 306
#define CMD_WIFI_ON_BOOT 401
#define CMD_SET_AP 402
#define CMD_SET_STA 403
#define CMD_WIFI_APSTA 404
#define CMD_WIFI_INFO 405
#define CMD_WIFI_CONFIG_CREATE_BY_STATUS 406
#define CMD_WIFI_CONFIG_CREATE_BY_INPUT 407
#define CMD_WIFI_STOP 408
#define CMD_SET_SERVO_ID 501
#define CMD_SET_MIDDLE 502
#define CMD_SET_SERVO_PID 503
#define CMD_REBOOT 600
#define CMD_FREE_FLASH_SPACE 601
#define CMD_BOOT_MISSION_INFO 602
#define CMD_RESET_BOOT_MISSION 603
#define CMD_NVS_CLEAR 604
#define CMD_INFO_PRINT 605
#define CMD_MM_TYPE_SET 900
#define CMD_RESET_EMERGENCY 999

// JSON command structure - matches original Arduino format
typedef struct {
    uint16_t type;         // Command type (T field)
    union {
        // Speed control command - {"T":1,"L":0.5,"R":0.5}
        struct {
            float L;           // Left motor speed
            float R;           // Right motor speed
        } speed_ctrl;
        
        // PWM input command - {"T":11,"L":164,"R":164}
        struct {
            int16_t L;         // Left motor PWM
            int16_t R;         // Right motor PWM
        } pwm_input;
        
        // ROS control command - {"T":13,"X":0.1,"Z":0.3}
        struct {
            float X;           // Linear speed (m/s)
            float Z;           // Angular speed (rad/s)
        } ros_ctrl;
        
        // PID parameters command - {"T":2,"P":200,"I":2500,"D":0,"L":255}
        struct {
            float P;           // Proportional gain
            float I;           // Integral gain
            float D;           // Derivative gain
            float L;           // Output limit
        } pid_params;
        
        // OLED control command - {"T":3,"lineNum":0,"Text":"Hello"}
        struct {
            int lineNum;       // Line number
            char Text[64];     // Text content
        } oled_ctrl;
        
        // Module type command - {"T":4,"cmd":0}
        struct {
            int cmd;           // Module type
        } module_type;
        
        // IMU data command - {"T":126}
        struct {
            // No parameters
        } get_imu_data;
        
        // IMU calibration command - {"T":127}
        struct {
            // No parameters
        } cali_imu_step;
        
        // IMU offset command - {"T":128}
        struct {
            // No parameters
        } get_imu_offset;
        
        // Set IMU offset command - {"T":129,"gx":0,"gy":0,"gz":0,"ax":0,"ay":0,"az":0,"cx":0,"cy":0,"cz":0}
        struct {
            float gx, gy, gz;  // Gyro offsets
            float ax, ay, az;  // Accel offsets
            float cx, cy, cz;  // Compass offsets
        } set_imu_offset;
        
        // Base feedback command - {"T":130}
        struct {
            // No parameters
        } base_feedback;
        
        // Base feedback flow command - {"T":131,"cmd":1}
        struct {
            int cmd;           // Feedback mode
        } base_feedback_flow;
        
        // Feedback flow interval command - {"T":142,"cmd":0}
        struct {
            int cmd;           // Interval in ms
        } feedback_flow_interval;
        
        // UART echo mode command - {"T":143,"cmd":0}
        struct {
            int cmd;           // Echo mode
        } uart_echo_mode;
        
        // LED control command - {"T":132,"IO4":255,"IO5":255}
        struct {
            int IO4;           // LED 1 PWM
            int IO5;           // LED 2 PWM
        } led_ctrl;
        
        // Gimbal control simple command - {"T":133,"X":45,"Y":45,"SPD":0,"ACC":0}
        struct {
            float X;           // Pan angle
            float Y;           // Tilt angle
            float SPD;         // Speed
            float ACC;         // Acceleration
        } gimbal_ctrl_simple;
        
        // Gimbal control move command - {"T":134,"X":45,"Y":45,"SX":300,"SY":300}
        struct {
            float X;           // Pan angle
            float Y;           // Tilt angle
            float SX;          // Pan speed
            float SY;          // Tilt speed
        } gimbal_ctrl_move;
        
        // Gimbal control stop command - {"T":135}
        struct {
            // No parameters
        } gimbal_ctrl_stop;
        
        // Heart beat set command - {"T":136,"cmd":3000}
        struct {
            int cmd;           // Heartbeat delay
        } heart_beat_set;
        
        // Gimbal steady command - {"T":137,"s":1,"y":0}
        struct {
            int s;             // Steady mode
            int y;             // Yaw mode
        } gimbal_steady;
        
        // Set speed rate command - {"T":138,"L":1,"R":1}
        struct {
            float L;           // Left speed rate
            float R;           // Right speed rate
        } set_spd_rate;
        
        // Get speed rate command - {"T":139}
        struct {
            // No parameters
        } get_spd_rate;
        
        // Save speed rate command - {"T":140}
        struct {
            // No parameters
        } save_spd_rate;
        
        // Gimbal user control command - {"T":141,"X":0,"Y":0,"SPD":300}
        struct {
            float X;           // Pan control
            float Y;           // Tilt control
            float SPD;         // Speed
        } gimbal_user_ctrl;
        
        // Arm control UI command - {"T":144,"E":100,"Z":0,"R":0}
        struct {
            float E;           // Elbow
            float Z;           // Z position
            float R;           // Roll
        } arm_ctrl_ui;
        
        // Single joint control command - {"T":101,"joint":0,"rad":0,"spd":0,"acc":10}
        struct {
            int joint;         // Joint number
            float rad;         // Angle in radians
            float spd;         // Speed
            float acc;         // Acceleration
        } single_joint_ctrl;
        
        // Joints rad control command - {"T":102,"base":0,"shoulder":0,"elbow":1.57,"hand":1.57,"spd":0,"acc":10}
        struct {
            float base;        // Base angle
            float shoulder;    // Shoulder angle
            float elbow;       // Elbow angle
            float hand;        // Hand angle
            float spd;         // Speed
            float acc;         // Acceleration
        } joints_rad_ctrl;
        
        // Single axis control command - {"T":103,"axis":2,"pos":0,"spd":0.25}
        struct {
            int axis;          // Axis number
            float pos;         // Position
            float spd;         // Speed
        } single_axis_ctrl;
        
        // XYZT goal control command - {"T":104,"x":235,"y":0,"z":234,"t":3.14,"spd":0.25}
        struct {
            float x;           // X position
            float y;           // Y position
            float z;           // Z position
            float t;           // T angle
            float spd;         // Speed
        } xyzt_goal_ctrl;
        
        // XYZT direct control command - {"T":1041,"x":235,"y":0,"z":234,"t":3.14}
        struct {
            float x;           // X position
            float y;           // Y position
            float z;           // Z position
            float t;           // T angle
        } xyzt_direct_ctrl;
        
        // Servo rad feedback command - {"T":105}
        struct {
            // No parameters
        } servo_rad_feedback;
        
        // EOAT hand control command - {"T":106,"cmd":1.57,"spd":0,"acc":0}
        struct {
            float cmd;         // Hand command
            float spd;         // Speed
            float acc;         // Acceleration
        } eoat_hand_ctrl;
        
        // EOAT grab torque command - {"T":107,"tor":200}
        struct {
            float tor;         // Torque
        } eoat_grab_torque;
        
        // Set joint PID command - {"T":108,"joint":3,"p":16,"i":0}
        struct {
            int joint;         // Joint number
            float p;           // P gain
            float i;           // I gain
        } set_joint_pid;
        
        // Reset PID command - {"T":109}
        struct {
            // No parameters
        } reset_pid;
        
        // Set new X command - {"T":110,"xAxisAngle":0}
        struct {
            float xAxisAngle;  // X axis angle
        } set_new_x;
        
        // Delay millis command - {"T":111,"cmd":3000}
        struct {
            int cmd;           // Delay in ms
        } delay_millis;
        
        // Dynamic adaptation command - {"T":112,"mode":1,"b":60,"s":110,"e":50,"h":50}
        struct {
            int mode;          // Mode
            float b;           // Base torque
            float s;           // Shoulder torque
            float e;           // Elbow torque
            float h;           // Hand torque
        } dynamic_adaptation;
        
        // Switch control command - {"T":113,"pwm_a":-255,"pwm_b":-255}
        struct {
            int pwm_a;         // PWM A
            int pwm_b;         // PWM B
        } switch_ctrl;
        
        // Light control command - {"T":114,"led":255}
        struct {
            int led;           // LED value
        } light_ctrl;
        
        // Switch off command - {"T":115}
        struct {
            // No parameters
        } switch_off;
        
        // Single joint angle command - {"T":121,"joint":1,"angle":0,"spd":10,"acc":10}
        struct {
            int joint;         // Joint number
            float angle;       // Angle in degrees
            float spd;         // Speed
            float acc;         // Acceleration
        } single_joint_angle;
        
        // Joints angle control command - {"T":122,"b":0,"s":0,"e":90,"h":180,"spd":10,"acc":10}
        struct {
            float b;           // Base angle
            float s;           // Shoulder angle
            float e;           // Elbow angle
            float h;           // Hand angle
            float spd;         // Speed
            float acc;         // Acceleration
        } joints_angle_ctrl;
        
        // Constant control command - {"T":123,"m":0,"axis":0,"cmd":0,"spd":3}
        struct {
            int m;             // Mode
            int axis;          // Axis
            int cmd;           // Command
            float spd;         // Speed
        } constant_ctrl;
        
        // EOAT type command - {"T":124,"mode":0}
        struct {
            int mode;          // EOAT mode
        } eoat_type;
        
        // Config EOAT command - {"T":125,"pos":3,"ea":0,"eb":20}
        struct {
            int pos;           // Position
            float ea;          // EA value
            float eb;          // EB value
        } config_eoat;
        
        // Move init command - {"T":100}
        struct {
            // No parameters
        } move_init;
        
        // Scan files command - {"T":200}
        struct {
            // No parameters
        } scan_files;
        
        // Create file command - {"T":201,"name":"file.txt","content":"inputContentHere."}
        struct {
            char name[32];     // File name
            char content[256]; // File content
        } create_file;
        
        // Read file command - {"T":202,"name":"file.txt"}
        struct {
            char name[32];     // File name
        } read_file;
        
        // Delete file command - {"T":203,"name":"file.txt"}
        struct {
            char name[32];     // File name
        } delete_file;
        
        // Append line command - {"T":204,"name":"file.txt","content":"inputContentHere."}
        struct {
            char name[32];     // File name
            char content[256]; // Line content
        } append_line;
        
        // Insert line command - {"T":205,"name":"file.txt","lineNum":3,"content":"content"}
        struct {
            char name[32];     // File name
            int lineNum;       // Line number
            char content[256]; // Line content
        } insert_line;
        
        // Replace line command - {"T":206,"name":"file.txt","lineNum":3,"content":"Content"}
        struct {
            char name[32];     // File name
            int lineNum;       // Line number
            char content[256]; // Line content
        } replace_line;
        
        // Read line command - {"T":207,"name":"file.txt","lineNum":3}
        struct {
            char name[32];     // File name
            int lineNum;       // Line number
        } read_line;
        
        // Delete line command - {"T":208,"name":"file.txt","lineNum":3}
        struct {
            char name[32];     // File name
            int lineNum;       // Line number
        } delete_line;
        
        // Torque control command - {"T":210,"cmd":1}
        struct {
            int cmd;           // Torque command
        } torque_ctrl;
        
        // Create mission command - {"T":220,"name":"mission_a","intro":"test mission created in flash."}
        struct {
            char name[32];     // Mission name
            char intro[128];   // Mission intro
        } create_mission;
        
        // Mission content command - {"T":221,"name":"mission_a"}
        struct {
            char name[32];     // Mission name
        } mission_content;
        
        // Append step JSON command - {"T":222,"name":"mission_a","step":"{\"T\":104,\"x\":235,\"y\":0,\"z\":234,\"t\":3.14,\"spd\":0.25}"}
        struct {
            char name[32];     // Mission name
            char step[256];    // Step JSON
        } append_step_json;
        
        // Append step FB command - {"T":223,"name":"mission_a","spd":0.25}
        struct {
            char name[32];     // Mission name
            float spd;         // Speed
        } append_step_fb;
        
        // Append delay command - {"T":224,"name":"mission_a","delay":3000}
        struct {
            char name[32];     // Mission name
            int delay;         // Delay in ms
        } append_delay;
        
        // Insert step JSON command - {"T":225,"name":"mission_a","stepNum":3,"step":"{\"T\":104,\"x\":235,\"y\":0,\"z\":234,\"t\":3.14,\"spd\":0.25}"}
        struct {
            char name[32];     // Mission name
            int stepNum;       // Step number
            char step[256];    // Step JSON
        } insert_step_json;
        
        // Insert step FB command - {"T":226,"name":"mission_a","stepNum":3,"spd":0.25}
        struct {
            char name[32];     // Mission name
            int stepNum;       // Step number
            float spd;         // Speed
        } insert_step_fb;
        
        // Insert delay command - {"T":227,"stepNum":3,"delay":3000}
        struct {
            int stepNum;       // Step number
            int delay;         // Delay in ms
        } insert_delay;
        
        // Replace step JSON command - {"T":228,"name":"mission_a","stepNum":3,"step":"{\"T\":114,\"led\":255}"}
        struct {
            char name[32];     // Mission name
            int stepNum;       // Step number
            char step[256];    // Step JSON
        } replace_step_json;
        
        // Replace step FB command - {"T":229,"name":"mission_a","stepNum":3,"spd":0.25}
        struct {
            char name[32];     // Mission name
            int stepNum;       // Step number
            float spd;         // Speed
        } replace_step_fb;
        
        // Replace delay command - {"T":230,"name":"mission_a","stepNum":3,"delay":3000}
        struct {
            char name[32];     // Mission name
            int stepNum;       // Step number
            int delay;         // Delay in ms
        } replace_delay;
        
        // Delete step command - {"T":231,"name":"mission_a","stepNum":3}
        struct {
            char name[32];     // Mission name
            int stepNum;       // Step number
        } delete_step;
        
        // Move to step command - {"T":241,"name":"mission_a","stepNum":3}
        struct {
            char name[32];     // Mission name
            int stepNum;       // Step number
        } move_to_step;
        
        // Mission play command - {"T":242,"name":"mission_a","times":3}
        struct {
            char name[32];     // Mission name
            int times;         // Repeat times
        } mission_play;
        
        // Broadcast follower command - {"T":300,"mode":1,"mac":"FF:FF:FF:FF:FF:FF"}
        struct {
            int mode;          // Mode
            char mac[18];      // MAC address
        } broadcast_follower;
        
        // ESP-NOW config command - {"T":301,"mode":3}
        struct {
            int mode;          // ESP-NOW mode
        } esp_now_config;
        
        // Get MAC address command - {"T":302}
        struct {
            // No parameters
        } get_mac_address;
        
        // ESP-NOW add follower command - {"T":303,"mac":"FF:FF:FF:FF:FF:FF"}
        struct {
            char mac[18];      // MAC address
        } esp_now_add_follower;
        
        // ESP-NOW remove follower command - {"T":304,"mac":"FF:FF:FF:FF:FF:FF"}
        struct {
            char mac[18];      // MAC address
        } esp_now_remove_follower;
        
        // ESP-NOW group control command - {"T":305,"dev":0,"b":0,"s":0,"e":1.57,"h":1.57,"cmd":0,"megs":"hello!"}
        struct {
            int dev;           // Device
            float b;           // Base
            float s;           // Shoulder
            float e;           // Elbow
            float h;           // Hand
            int cmd;           // Command
            char megs[64];     // Message
        } esp_now_group_ctrl;
        
        // ESP-NOW single command - {"T":306,"mac":"FF:FF:FF:FF:FF:FF","dev":0,"b":0,"s":0,"e":1.57,"h":1.57,"cmd":0,"megs":"hello!"}
        struct {
            char mac[18];      // MAC address
            int dev;           // Device
            float b;           // Base
            float s;           // Shoulder
            float e;           // Elbow
            float h;           // Hand
            int cmd;           // Command
            char megs[64];     // Message
        } esp_now_single;
        
        // ESP-NOW receive feedback - {"T":1003,"mac":"FF:FF:FF:FF:FF:FF","megs":"hello!"}
        struct {
            char mac[18];       // MAC address
            char megs[64];      // Message
        } esp_now_recv;
        
        // ESP-NOW send status feedback - {"T":1004,"mac":"FF:FF:FF:FF:FF:FF","status":1,"megs":"xxx"}
        struct {
            char mac[18];       // MAC address
            int status;         // Status code
            char megs[64];      // Message
        } esp_now_send;
        
        // Bus servo error feedback - {"T":1005,"id":1,"status":1}
        struct {
            int id;             // Servo ID
            int status;         // Error status
        } bus_servo_error;
        
        // WiFi on boot command - {"T":401,"cmd":3}
        struct {
            int cmd;           // WiFi mode
        } wifi_on_boot;
        
        // Set AP command - {"T":402,"ssid":"RoArm-M2","password":"12345678"}
        struct {
            char ssid[32];     // SSID
            char password[64]; // Password
        } set_ap;
        
        // Set STA command - {"T":403,"ssid":"JSBZY-2.4G","password":"waveshare0755"}
        struct {
            char ssid[32];     // SSID
            char password[64]; // Password
        } set_sta;
        
        // WiFi APSTA command - {"T":404,"ap_ssid":"RoArm-M2","ap_password":"12345678","sta_ssid":"JSBZY-2.4G","sta_password":"waveshare0755"}
        struct {
            char ap_ssid[32];     // AP SSID
            char ap_password[64]; // AP Password
            char sta_ssid[32];    // STA SSID
            char sta_password[64]; // STA Password
        } wifi_apsta;
        
        // WiFi info command - {"T":405}
        struct {
            // No parameters
        } wifi_info;
        
        // WiFi config create by status command - {"T":406}
        struct {
            // No parameters
        } wifi_config_create_by_status;
        
        // WiFi config create by input command - {"T":407,"mode":3,"ap_ssid":"RoArm-M2","ap_password":"12345678","sta_ssid":"JSBZY-2.4G","sta_password":"waveshare0755"}
        struct {
            int mode;          // WiFi mode
            char ap_ssid[32];     // AP SSID
            char ap_password[64]; // AP Password
            char sta_ssid[32];    // STA SSID
            char sta_password[64]; // STA Password
        } wifi_config_create_by_input;
        
        // WiFi stop command - {"T":408}
        struct {
            // No parameters
        } wifi_stop;
        
        // Set servo ID command - {"T":501,"raw":1,"new":11}
        struct {
            int raw;           // Raw ID
            int new_id;        // New ID
        } set_servo_id;
        
        // Set middle command - {"T":502,"id":11}
        struct {
            int id;            // Servo ID
        } set_middle;
        
        // Set servo PID command - {"T":503,"id":14,"p":16}
        struct {
            int id;            // Servo ID
            float p;           // P gain
        } set_servo_pid;
        
        // Reboot command - {"T":600}
        struct {
            // No parameters
        } reboot;
        
        // Free flash space command - {"T":601}
        struct {
            // No parameters
        } free_flash_space;
        
        // Boot mission info command - {"T":602}
        struct {
            // No parameters
        } boot_mission_info;
        
        // Reset boot mission command - {"T":603}
        struct {
            // No parameters
        } reset_boot_mission;
        
        // NVS clear command - {"T":604}
        struct {
            // No parameters
        } nvs_clear;
        
        // Info print command - {"T":605,"cmd":1}
        struct {
            int cmd;           // Print mode
        } info_print;
        
        // MM type set command - {"T":900,"main":1,"module":0}
        struct {
            int main;          // Main type
            int module;        // Module type
        } mm_type_set;
        
        // Emergency stop command - {"T":0}
        struct {
            // No parameters
        } emergency_stop;
        
        // Reset emergency command - {"T":999}
        struct {
            // No parameters
        } reset_emergency;
        
        // OLED default command - {"T":-3}
        struct {
            // No parameters
        } oled_default;
        
    } data;
} json_command_t;

// JSON feedback structure
typedef struct {
    uint16_t type;         // Feedback type
    union {
        // Base info feedback - {"T":1001,"L":0,"R":0,"gx":0,"gy":0,"gz":0,"ax":0,"ay":0,"az":0,"mx":0,"my":0,"mz":0,"odl":0,"odr":0,"v":11.0}
        struct {
            float L, R;        // Motor speeds
            float gx, gy, gz;  // Gyro data
            float ax, ay, az;  // Accel data
            float mx, my, mz;  // Mag data
            float odl, odr;    // Odometry
            float v;           // Voltage
        } base_info;
        
        // IMU data feedback - {"T":1002,"gx":0,"gy":0,"gz":0,"ax":0,"ay":0,"az":0,"mx":0,"my":0,"mz":0}
        struct {
            float gx, gy, gz;  // Gyro data
            float ax, ay, az;  // Accel data
            float mx, my, mz;  // Mag data
        } imu_data;
        
        // ESP-NOW receive feedback - {"T":1003,"mac":"FF:FF:FF:FF:FF:FF","megs":"hello!"}
        struct {
            char mac[18];      // MAC address
            char megs[64];     // Message
        } esp_now_recv;
        
        // IMU offset feedback - {"T":129,"gx":0,"gy":0,"gz":0,"ax":0,"ay":0,"az":0,"cx":0,"cy":0,"cz":0}
        struct {
            float gx, gy, gz;  // Gyro offsets
            float ax, ay, az;  // Accel offsets
            float cx, cy, cz;  // Compass offsets
        } imu_offset;
        
        // ESP-NOW send feedback - {"T":1004,"mac":"FF:FF:FF:FF:FF:FF","status":1,"megs":"xxx"}
        struct {
            char mac[18];      // MAC address
            int status;        // Status
            char megs[64];     // Message
        } esp_now_send;
        
        // Bus servo error feedback - {"T":1005,"id":1,"status":1}
        struct {
            int id;            // Servo ID
            int status;        // Error status
        } bus_servo_error;
        
    } data;
} json_feedback_t;

// Function declarations
esp_err_t json_parser_init(void);
esp_err_t json_parser_parse_command(const char *json_str, json_command_t *cmd);
esp_err_t json_parser_send_feedback(const json_feedback_t *feedback);
esp_err_t json_parser_send_base_feedback(void);

#ifdef __cplusplus
}
#endif

#endif // JSON_PARSER_H