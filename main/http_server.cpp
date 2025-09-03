#include "../inc/http_server.h"
#include <esp_http_server.h>
#include <esp_log.h>
#include <freertos/FreeRTOS.h>
#include <freertos/task.h>
#include <string.h>
#include <cJSON.h>

static const char *TAG = "HTTP_Server";

// HTTP server configuration
#define HTTP_SERVER_PORT 80
#define MAX_URI_HANDLERS 16

// Global variables
static httpd_handle_t server = NULL;
static bool server_initialized = false;

// Private function prototypes
static esp_err_t root_get_handler(httpd_req_t *req);
static esp_err_t api_status_get_handler(httpd_req_t *req);
static esp_err_t api_command_post_handler(httpd_req_t *req);
static esp_err_t api_motion_post_handler(httpd_req_t *req);
static esp_err_t api_servo_post_handler(httpd_req_t *req);
static esp_err_t api_system_post_handler(httpd_req_t *req);
static esp_err_t api_arm_post_handler(httpd_req_t *req);
static esp_err_t api_gimbal_post_handler(httpd_req_t *req);
static esp_err_t api_mission_post_handler(httpd_req_t *req);
static esp_err_t api_files_post_handler(httpd_req_t *req);
static esp_err_t api_wifi_post_handler(httpd_req_t *req);
static esp_err_t api_espnow_post_handler(httpd_req_t *req);
static esp_err_t api_imu_post_handler(httpd_req_t *req);
static esp_err_t api_config_post_handler(httpd_req_t *req);
static esp_err_t not_found_handler(httpd_req_t *req, httpd_err_code_t err);
static esp_err_t parse_json_body(httpd_req_t *req, cJSON **json);
static esp_err_t send_json_response(httpd_req_t *req, cJSON *json);

esp_err_t http_server_init(void)
{
    if (server_initialized) {
        ESP_LOGW(TAG, "HTTP server already initialized");
        return ESP_OK;
    }

    ESP_LOGI(TAG, "Initializing HTTP server...");

    // HTTP server configuration
    httpd_config_t config = HTTPD_DEFAULT_CONFIG();
    config.server_port = HTTP_SERVER_PORT;
    config.max_uri_handlers = MAX_URI_HANDLERS;
    config.stack_size = 8192;

    // Start HTTP server
    esp_err_t ret = httpd_start(&server, &config);
    if (ret != ESP_OK) {
        ESP_LOGE(TAG, "Failed to start HTTP server: %s", esp_err_to_name(ret));
        return ret;
    }

    // Register URI handlers
    httpd_uri_t root_uri = {
        .uri = "/",
        .method = HTTP_GET,
        .handler = root_get_handler,
        .user_ctx = NULL
    };
    httpd_register_uri_handler(server, &root_uri);

    httpd_uri_t status_uri = {
        .uri = "/api/status",
        .method = HTTP_GET,
        .handler = api_status_get_handler,
        .user_ctx = NULL
    };
    httpd_register_uri_handler(server, &status_uri);

    httpd_uri_t command_uri = {
        .uri = "/api/command",
        .method = HTTP_POST,
        .handler = api_command_post_handler,
        .user_ctx = NULL
    };
    httpd_register_uri_handler(server, &command_uri);

    httpd_uri_t motion_uri = {
        .uri = "/api/motion",
        .method = HTTP_POST,
        .handler = api_motion_post_handler,
        .user_ctx = NULL
    };
    httpd_register_uri_handler(server, &motion_uri);

    httpd_uri_t servo_uri = {
        .uri = "/api/servo",
        .method = HTTP_POST,
        .handler = api_servo_post_handler,
        .user_ctx = NULL
    };
    httpd_register_uri_handler(server, &servo_uri);

    httpd_uri_t system_uri = {
        .uri = "/api/system",
        .method = HTTP_POST,
        .handler = api_system_post_handler,
        .user_ctx = NULL
    };
    httpd_register_uri_handler(server, &system_uri);

    httpd_uri_t arm_uri = {
        .uri = "/api/arm",
        .method = HTTP_POST,
        .handler = api_arm_post_handler,
        .user_ctx = NULL
    };
    httpd_register_uri_handler(server, &arm_uri);

    httpd_uri_t gimbal_uri = {
        .uri = "/api/gimbal",
        .method = HTTP_POST,
        .handler = api_gimbal_post_handler,
        .user_ctx = NULL
    };
    httpd_register_uri_handler(server, &gimbal_uri);

    httpd_uri_t mission_uri = {
        .uri = "/api/mission",
        .method = HTTP_POST,
        .handler = api_mission_post_handler,
        .user_ctx = NULL
    };
    httpd_register_uri_handler(server, &mission_uri);

    httpd_uri_t files_uri = {
        .uri = "/api/files",
        .method = HTTP_POST,
        .handler = api_files_post_handler,
        .user_ctx = NULL
    };
    httpd_register_uri_handler(server, &files_uri);

    httpd_uri_t wifi_uri = {
        .uri = "/api/wifi",
        .method = HTTP_POST,
        .handler = api_wifi_post_handler,
        .user_ctx = NULL
    };
    httpd_register_uri_handler(server, &wifi_uri);

    httpd_uri_t espnow_uri = {
        .uri = "/api/espnow",
        .method = HTTP_POST,
        .handler = api_espnow_post_handler,
        .user_ctx = NULL
    };
    httpd_register_uri_handler(server, &espnow_uri);

    httpd_uri_t imu_uri = {
        .uri = "/api/imu",
        .method = HTTP_POST,
        .handler = api_imu_post_handler,
        .user_ctx = NULL
    };
    httpd_register_uri_handler(server, &imu_uri);

    httpd_uri_t config_uri = {
        .uri = "/api/config",
        .method = HTTP_POST,
        .handler = api_config_post_handler,
        .user_ctx = NULL
    };
    httpd_register_uri_handler(server, &config_uri);

    // Register 404 handler
    httpd_register_err_handler(server, HTTPD_404_NOT_FOUND, not_found_handler);

    server_initialized = true;
    ESP_LOGI(TAG, "HTTP server initialized successfully on port %d", HTTP_SERVER_PORT);
    return ESP_OK;
}

esp_err_t http_server_deinit(void)
{
    if (!server_initialized) {
        return ESP_OK;
    }

    ESP_LOGI(TAG, "Deinitializing HTTP server...");

    if (server) {
        httpd_stop(server);
        server = NULL;
    }

    server_initialized = false;
    ESP_LOGI(TAG, "HTTP server deinitialized");
    return ESP_OK;
}

// Private functions
static esp_err_t root_get_handler(httpd_req_t *req)
{
    ESP_LOGI(TAG, "Root page requested");

    // Send simple text response
    const char *response = "RaspRover HTTP API Server\n\n"
                          "Available endpoints:\n"
                          "GET  /api/status     - Get system status\n"
                          "POST /api/command    - Send motion command\n"
                          "POST /api/motion     - Send motion parameters\n"
                          "POST /api/servo      - Control servo\n"
                          "POST /api/system     - System commands\n"
                          "POST /api/arm        - Robotic arm control\n"
                          "POST /api/gimbal     - Gimbal control\n"
                          "POST /api/mission    - Mission management\n"
                          "POST /api/files      - File operations\n"
                          "POST /api/wifi       - WiFi configuration\n"
                          "POST /api/espnow     - ESP-NOW control\n"
                          "POST /api/imu        - IMU operations\n"
                          "POST /api/config     - System configuration\n";

    httpd_resp_set_type(req, "text/plain");
    httpd_resp_send(req, response, strlen(response));
    return ESP_OK;
}

static esp_err_t api_status_get_handler(httpd_req_t *req)
{
    ESP_LOGI(TAG, "Status API requested");

    cJSON *response = cJSON_CreateObject();
    cJSON_AddStringToObject(response, "status", "online");
    cJSON_AddNumberToObject(response, "battery", 85);
    cJSON_AddStringToObject(response, "wifi", "connected");
    cJSON_AddNumberToObject(response, "uptime", esp_timer_get_time() / 1000000);

    esp_err_t ret = send_json_response(req, response);
    cJSON_Delete(response);
    return ret;
}

static esp_err_t api_command_post_handler(httpd_req_t *req)
{
    ESP_LOGI(TAG, "Command API requested");

    cJSON *json = NULL;
    esp_err_t ret = parse_json_body(req, &json);
    if (ret != ESP_OK) {
        return ret;
    }

    // Parse command
    cJSON *command = cJSON_GetObjectItem(json, "command");
    cJSON *speed = cJSON_GetObjectItem(json, "speed");
    
    if (!cJSON_IsString(command)) {
        cJSON_Delete(json);
        return httpd_resp_send_err(req, HTTPD_400_BAD_REQUEST, "Invalid command format");
    }

    const char *cmd_str = command->valuestring;
    int speed_val = speed && cJSON_IsNumber(speed) ? (int)speed->valuedouble : 50;

    ESP_LOGI(TAG, "Received command: %s, speed: %d", cmd_str, speed_val);

    // Send response
    cJSON *response = cJSON_CreateObject();
    cJSON_AddStringToObject(response, "status", "success");
    cJSON_AddStringToObject(response, "message", "Command executed");
    cJSON_AddStringToObject(response, "command", cmd_str);
    cJSON_AddNumberToObject(response, "speed", speed_val);

    ret = send_json_response(req, response);
    cJSON_Delete(response);
    cJSON_Delete(json);
    return ret;
}

static esp_err_t api_motion_post_handler(httpd_req_t *req)
{
    ESP_LOGI(TAG, "Motion API requested");

    cJSON *json = NULL;
    esp_err_t ret = parse_json_body(req, &json);
    if (ret != ESP_OK) {
        return ret;
    }

    // Parse motion parameters
    cJSON *linear_x = cJSON_GetObjectItem(json, "linear_x");
    cJSON *angular_z = cJSON_GetObjectItem(json, "angular_z");
    
    if (!cJSON_IsNumber(linear_x) || !cJSON_IsNumber(angular_z)) {
        cJSON_Delete(json);
        return httpd_resp_send_err(req, HTTPD_400_BAD_REQUEST, "Invalid motion parameters");
    }

    float linear = (float)linear_x->valuedouble;
    float angular = (float)angular_z->valuedouble;

    ESP_LOGI(TAG, "Motion command: linear_x=%.2f, angular_z=%.2f", linear, angular);

    // Send response
    cJSON *response = cJSON_CreateObject();
    cJSON_AddStringToObject(response, "status", "success");
    cJSON_AddStringToObject(response, "message", "Motion command executed");
    cJSON_AddNumberToObject(response, "linear_x", linear);
    cJSON_AddNumberToObject(response, "angular_z", angular);

    ret = send_json_response(req, response);
    cJSON_Delete(response);
    cJSON_Delete(json);
    return ret;
}

static esp_err_t api_servo_post_handler(httpd_req_t *req)
{
    ESP_LOGI(TAG, "Servo API requested");

    cJSON *json = NULL;
    esp_err_t ret = parse_json_body(req, &json);
    if (ret != ESP_OK) {
        return ret;
    }

    // Parse servo parameters
    cJSON *servo_id = cJSON_GetObjectItem(json, "servo_id");
    cJSON *position = cJSON_GetObjectItem(json, "position");
    
    if (!cJSON_IsNumber(servo_id) || !cJSON_IsNumber(position)) {
        cJSON_Delete(json);
        return httpd_resp_send_err(req, HTTPD_400_BAD_REQUEST, "Invalid servo parameters");
    }

    int id = (int)servo_id->valuedouble;
    int pos = (int)position->valuedouble;

    ESP_LOGI(TAG, "Servo command: ID=%d, Position=%d", id, pos);

    // Send response
    cJSON *response = cJSON_CreateObject();
    cJSON_AddStringToObject(response, "status", "success");
    cJSON_AddStringToObject(response, "message", "Servo command executed");
    cJSON_AddNumberToObject(response, "servo_id", id);
    cJSON_AddNumberToObject(response, "position", pos);

    ret = send_json_response(req, response);
    cJSON_Delete(response);
    cJSON_Delete(json);
    return ret;
}

static esp_err_t api_system_post_handler(httpd_req_t *req)
{
    ESP_LOGI(TAG, "System API requested");

    cJSON *json = NULL;
    esp_err_t ret = parse_json_body(req, &json);
    if (ret != ESP_OK) {
        return ret;
    }

    // Parse system command
    cJSON *command = cJSON_GetObjectItem(json, "command");
    
    if (!cJSON_IsString(command)) {
        cJSON_Delete(json);
        return httpd_resp_send_err(req, HTTPD_400_BAD_REQUEST, "Invalid system command");
    }

    const char *cmd_str = command->valuestring;
    ESP_LOGI(TAG, "System command: %s", cmd_str);

    // Send response
    cJSON *response = cJSON_CreateObject();
    cJSON_AddStringToObject(response, "status", "success");
    cJSON_AddStringToObject(response, "message", "System command executed");
    cJSON_AddStringToObject(response, "command", cmd_str);

    ret = send_json_response(req, response);
    cJSON_Delete(response);
    cJSON_Delete(json);
    return ret;
}

// Robotic Arm Control API
static esp_err_t api_arm_post_handler(httpd_req_t *req)
{
    ESP_LOGI(TAG, "Arm API requested");

    cJSON *json = NULL;
    esp_err_t ret = parse_json_body(req, &json);
    if (ret != ESP_OK) {
        return ret;
    }

    // Parse arm command
    cJSON *command = cJSON_GetObjectItem(json, "command");
    cJSON *joint = cJSON_GetObjectItem(json, "joint");
    cJSON *position = cJSON_GetObjectItem(json, "position");
    cJSON *speed = cJSON_GetObjectItem(json, "speed");
    
    if (!cJSON_IsString(command)) {
        cJSON_Delete(json);
        return httpd_resp_send_err(req, HTTPD_400_BAD_REQUEST, "Invalid arm command format");
    }

    const char *cmd_str = command->valuestring;
    int joint_id = joint && cJSON_IsNumber(joint) ? (int)joint->valuedouble : 0;
    float pos = position && cJSON_IsNumber(position) ? (float)position->valuedouble : 0.0f;
    float spd = speed && cJSON_IsNumber(speed) ? (float)speed->valuedouble : 0.25f;

    ESP_LOGI(TAG, "Arm command: %s, joint: %d, position: %.2f, speed: %.2f", 
             cmd_str, joint_id, pos, spd);

    // Send response
    cJSON *response = cJSON_CreateObject();
    cJSON_AddStringToObject(response, "status", "success");
    cJSON_AddStringToObject(response, "message", "Arm command executed");
    cJSON_AddStringToObject(response, "command", cmd_str);
    cJSON_AddNumberToObject(response, "joint", joint_id);
    cJSON_AddNumberToObject(response, "position", pos);
    cJSON_AddNumberToObject(response, "speed", spd);

    ret = send_json_response(req, response);
    cJSON_Delete(response);
    cJSON_Delete(json);
    return ret;
}

// Gimbal Control API
static esp_err_t api_gimbal_post_handler(httpd_req_t *req)
{
    ESP_LOGI(TAG, "Gimbal API requested");

    cJSON *json = NULL;
    esp_err_t ret = parse_json_body(req, &json);
    if (ret != ESP_OK) {
        return ret;
    }

    // Parse gimbal command
    cJSON *command = cJSON_GetObjectItem(json, "command");
    cJSON *x_angle = cJSON_GetObjectItem(json, "x_angle");
    cJSON *y_angle = cJSON_GetObjectItem(json, "y_angle");
    cJSON *speed = cJSON_GetObjectItem(json, "speed");
    
    if (!cJSON_IsString(command)) {
        cJSON_Delete(json);
        return httpd_resp_send_err(req, HTTPD_400_BAD_REQUEST, "Invalid gimbal command format");
    }

    const char *cmd_str = command->valuestring;
    float x = x_angle && cJSON_IsNumber(x_angle) ? (float)x_angle->valuedouble : 0.0f;
    float y = y_angle && cJSON_IsNumber(y_angle) ? (float)y_angle->valuedouble : 0.0f;
    float spd = speed && cJSON_IsNumber(speed) ? (float)speed->valuedouble : 300.0f;

    ESP_LOGI(TAG, "Gimbal command: %s, X: %.2f, Y: %.2f, Speed: %.2f", 
             cmd_str, x, y, spd);

    // Send response
    cJSON *response = cJSON_CreateObject();
    cJSON_AddStringToObject(response, "status", "success");
    cJSON_AddStringToObject(response, "message", "Gimbal command executed");
    cJSON_AddStringToObject(response, "command", cmd_str);
    cJSON_AddNumberToObject(response, "x_angle", x);
    cJSON_AddNumberToObject(response, "y_angle", y);
    cJSON_AddNumberToObject(response, "speed", spd);

    ret = send_json_response(req, response);
    cJSON_Delete(response);
    cJSON_Delete(json);
    return ret;
}

// Mission Management API
static esp_err_t api_mission_post_handler(httpd_req_t *req)
{
    ESP_LOGI(TAG, "Mission API requested");

    cJSON *json = NULL;
    esp_err_t ret = parse_json_body(req, &json);
    if (ret != ESP_OK) {
        return ret;
    }

    // Parse mission command
    cJSON *command = cJSON_GetObjectItem(json, "command");
    cJSON *mission_name = cJSON_GetObjectItem(json, "mission_name");
    cJSON *step_content = cJSON_GetObjectItem(json, "step_content");
    cJSON *line_num = cJSON_GetObjectItem(json, "line_num");
    
    if (!cJSON_IsString(command)) {
        cJSON_Delete(json);
        return httpd_resp_send_err(req, HTTPD_400_BAD_REQUEST, "Invalid mission command format");
    }

    const char *cmd_str = command->valuestring;
    const char *name = mission_name && cJSON_IsString(mission_name) ? mission_name->valuestring : "";
    const char *step = step_content && cJSON_IsString(step_content) ? step_content->valuestring : "";
    int line = line_num && cJSON_IsNumber(line_num) ? (int)line_num->valuedouble : 0;

    ESP_LOGI(TAG, "Mission command: %s, name: %s, step: %s, line: %d", 
             cmd_str, name, step, line);

    // Send response
    cJSON *response = cJSON_CreateObject();
    cJSON_AddStringToObject(response, "status", "success");
    cJSON_AddStringToObject(response, "message", "Mission command executed");
    cJSON_AddStringToObject(response, "command", cmd_str);
    cJSON_AddStringToObject(response, "mission_name", name);
    cJSON_AddStringToObject(response, "step_content", step);
    cJSON_AddNumberToObject(response, "line_num", line);

    ret = send_json_response(req, response);
    cJSON_Delete(response);
    cJSON_Delete(json);
    return ret;
}

// File Operations API
static esp_err_t api_files_post_handler(httpd_req_t *req)
{
    ESP_LOGI(TAG, "Files API requested");

    cJSON *json = NULL;
    esp_err_t ret = parse_json_body(req, &json);
    if (ret != ESP_OK) {
        return ret;
    }

    // Parse file command
    cJSON *command = cJSON_GetObjectItem(json, "command");
    cJSON *filename = cJSON_GetObjectItem(json, "filename");
    cJSON *content = cJSON_GetObjectItem(json, "content");
    cJSON *line_num = cJSON_GetObjectItem(json, "line_num");
    
    if (!cJSON_IsString(command)) {
        cJSON_Delete(json);
        return httpd_resp_send_err(req, HTTPD_400_BAD_REQUEST, "Invalid file command format");
    }

    const char *cmd_str = command->valuestring;
    const char *name = filename && cJSON_IsString(filename) ? filename->valuestring : "";
    const char *cont = content && cJSON_IsString(content) ? content->valuestring : "";
    int line = line_num && cJSON_IsNumber(line_num) ? (int)line_num->valuedouble : 0;

    ESP_LOGI(TAG, "File command: %s, filename: %s, content: %s, line: %d", 
             cmd_str, name, cont, line);

    // Send response
    cJSON *response = cJSON_CreateObject();
    cJSON_AddStringToObject(response, "status", "success");
    cJSON_AddStringToObject(response, "message", "File command executed");
    cJSON_AddStringToObject(response, "command", cmd_str);
    cJSON_AddStringToObject(response, "filename", name);
    cJSON_AddStringToObject(response, "content", cont);
    cJSON_AddNumberToObject(response, "line_num", line);

    ret = send_json_response(req, response);
    cJSON_Delete(response);
    cJSON_Delete(json);
    return ret;
}

// WiFi Configuration API
static esp_err_t api_wifi_post_handler(httpd_req_t *req)
{
    ESP_LOGI(TAG, "WiFi API requested");

    cJSON *json = NULL;
    esp_err_t ret = parse_json_body(req, &json);
    if (ret != ESP_OK) {
        return ret;
    }

    // Parse WiFi command
    cJSON *command = cJSON_GetObjectItem(json, "command");
    cJSON *ssid = cJSON_GetObjectItem(json, "ssid");
    cJSON *password = cJSON_GetObjectItem(json, "password");
    cJSON *mode = cJSON_GetObjectItem(json, "mode");
    
    if (!cJSON_IsString(command)) {
        cJSON_Delete(json);
        return httpd_resp_send_err(req, HTTPD_400_BAD_REQUEST, "Invalid WiFi command format");
    }

    const char *cmd_str = command->valuestring;
    const char *wifi_ssid = ssid && cJSON_IsString(ssid) ? ssid->valuestring : "";
    const char *wifi_pass = password && cJSON_IsString(password) ? password->valuestring : "";
    int wifi_mode = mode && cJSON_IsNumber(mode) ? (int)mode->valuedouble : 3;

    ESP_LOGI(TAG, "WiFi command: %s, SSID: %s, Mode: %d", cmd_str, wifi_ssid, wifi_mode);

    // Send response
    cJSON *response = cJSON_CreateObject();
    cJSON_AddStringToObject(response, "status", "success");
    cJSON_AddStringToObject(response, "message", "WiFi command executed");
    cJSON_AddStringToObject(response, "command", cmd_str);
    cJSON_AddStringToObject(response, "ssid", wifi_ssid);
    cJSON_AddNumberToObject(response, "mode", wifi_mode);

    ret = send_json_response(req, response);
    cJSON_Delete(response);
    cJSON_Delete(json);
    return ret;
}

// ESP-NOW Control API
static esp_err_t api_espnow_post_handler(httpd_req_t *req)
{
    ESP_LOGI(TAG, "ESP-NOW API requested");

    cJSON *json = NULL;
    esp_err_t ret = parse_json_body(req, &json);
    if (ret != ESP_OK) {
        return ret;
    }

    // Parse ESP-NOW command
    cJSON *command = cJSON_GetObjectItem(json, "command");
    cJSON *mac_address = cJSON_GetObjectItem(json, "mac_address");
    cJSON *mode = cJSON_GetObjectItem(json, "mode");
    cJSON *message = cJSON_GetObjectItem(json, "message");
    
    if (!cJSON_IsString(command)) {
        cJSON_Delete(json);
        return httpd_resp_send_err(req, HTTPD_400_BAD_REQUEST, "Invalid ESP-NOW command format");
    }

    const char *cmd_str = command->valuestring;
    const char *mac = mac_address && cJSON_IsString(mac_address) ? mac_address->valuestring : "";
    int esp_mode = mode && cJSON_IsNumber(mode) ? (int)mode->valuedouble : 3;
    const char *msg = message && cJSON_IsString(message) ? message->valuestring : "";

    ESP_LOGI(TAG, "ESP-NOW command: %s, MAC: %s, Mode: %d, Message: %s", 
             cmd_str, mac, esp_mode, msg);

    // Send response
    cJSON *response = cJSON_CreateObject();
    cJSON_AddStringToObject(response, "status", "success");
    cJSON_AddStringToObject(response, "message", "ESP-NOW command executed");
    cJSON_AddStringToObject(response, "command", cmd_str);
    cJSON_AddStringToObject(response, "mac_address", mac);
    cJSON_AddNumberToObject(response, "mode", esp_mode);
    cJSON_AddStringToObject(response, "message", msg);

    ret = send_json_response(req, response);
    cJSON_Delete(response);
    cJSON_Delete(json);
    return ret;
}

// IMU Operations API
static esp_err_t api_imu_post_handler(httpd_req_t *req)
{
    ESP_LOGI(TAG, "IMU API requested");

    cJSON *json = NULL;
    esp_err_t ret = parse_json_body(req, &json);
    if (ret != ESP_OK) {
        return ret;
    }

    // Parse IMU command
    cJSON *command = cJSON_GetObjectItem(json, "command");
    cJSON *calibration = cJSON_GetObjectItem(json, "calibration");
    cJSON *offsets = cJSON_GetObjectItem(json, "offsets");
    
    if (!cJSON_IsString(command)) {
        cJSON_Delete(json);
        return httpd_resp_send_err(req, HTTPD_400_BAD_REQUEST, "Invalid IMU command format");
    }

    const char *cmd_str = command->valuestring;
    bool cal = calibration && cJSON_IsBool(calibration) ? cJSON_IsTrue(calibration) : false;
    bool off = offsets && cJSON_IsBool(offsets) ? cJSON_IsTrue(offsets) : false;

    ESP_LOGI(TAG, "IMU command: %s, calibration: %s, offsets: %s", 
             cmd_str, cal ? "true" : "false", off ? "true" : "false");

    // Send response
    cJSON *response = cJSON_CreateObject();
    cJSON_AddStringToObject(response, "status", "success");
    cJSON_AddStringToObject(response, "message", "IMU command executed");
    cJSON_AddStringToObject(response, "command", cmd_str);
    cJSON_AddBoolToObject(response, "calibration", cal);
    cJSON_AddBoolToObject(response, "offsets", off);

    ret = send_json_response(req, response);
    cJSON_Delete(response);
    cJSON_Delete(json);
    return ret;
}

// System Configuration API
static esp_err_t api_config_post_handler(httpd_req_t *req)
{
    ESP_LOGI(TAG, "Config API requested");

    cJSON *json = NULL;
    esp_err_t ret = parse_json_body(req, &json);
    if (ret != ESP_OK) {
        return ret;
    }

    // Parse config command
    cJSON *command = cJSON_GetObjectItem(json, "command");
    cJSON *main_type = cJSON_GetObjectItem(json, "main_type");
    cJSON *module_type = cJSON_GetObjectItem(json, "module_type");
    cJSON *eoat_type = cJSON_GetObjectItem(json, "eoat_type");
    
    if (!cJSON_IsString(command)) {
        cJSON_Delete(json);
        return httpd_resp_send_err(req, HTTPD_400_BAD_REQUEST, "Invalid config command format");
    }

    const char *cmd_str = command->valuestring;
    int main = main_type && cJSON_IsNumber(main_type) ? (int)main_type->valuedouble : 1;
    int module = module_type && cJSON_IsNumber(module_type) ? (int)module_type->valuedouble : 0;
    int eoat = eoat_type && cJSON_IsNumber(eoat_type) ? (int)eoat_type->valuedouble : 0;

    ESP_LOGI(TAG, "Config command: %s, main_type: %d, module_type: %d, eoat_type: %d", 
             cmd_str, main, module, eoat);

    // Send response
    cJSON *response = cJSON_CreateObject();
    cJSON_AddStringToObject(response, "status", "success");
    cJSON_AddStringToObject(response, "message", "Config command executed");
    cJSON_AddStringToObject(response, "command", cmd_str);
    cJSON_AddNumberToObject(response, "main_type", main);
    cJSON_AddNumberToObject(response, "module_type", module);
    cJSON_AddNumberToObject(response, "eoat_type", eoat);

    ret = send_json_response(req, response);
    cJSON_Delete(response);
    cJSON_Delete(json);
    return ret;
}

static esp_err_t not_found_handler(httpd_req_t *req, httpd_err_code_t err)
{
    ESP_LOGW(TAG, "404 - URI not found: %s", req->uri);
    
    cJSON *response = cJSON_CreateObject();
    cJSON_AddStringToObject(response, "error", "Not Found");
    cJSON_AddStringToObject(response, "message", "The requested resource was not found");
    cJSON_AddStringToObject(response, "uri", req->uri);

    esp_err_t ret = send_json_response(req, response);
    cJSON_Delete(response);
    return ret;
}

static esp_err_t parse_json_body(httpd_req_t *req, cJSON **json)
{
    char *content = NULL;
    size_t recv_size = req->content_len;
    
    if (recv_size > 1024) {
        ESP_LOGE(TAG, "Content length too long: %d", recv_size);
        return httpd_resp_send_err(req, HTTPD_400_BAD_REQUEST, "Content too long");
    }

    content = (char *)malloc(recv_size + 1);
    if (content == NULL) {
        ESP_LOGE(TAG, "Failed to allocate memory for content");
        return httpd_resp_send_err(req, HTTPD_500_INTERNAL_SERVER_ERROR, "Internal server error");
    }

    int ret = httpd_req_recv(req, content, recv_size);
    if (ret <= 0) {
        free(content);
        if (ret == HTTPD_SOCK_ERR_TIMEOUT) {
            return httpd_resp_send_err(req, HTTPD_408_REQ_TIMEOUT, "Request timeout");
        }
        return httpd_resp_send_err(req, HTTPD_500_INTERNAL_SERVER_ERROR, "Failed to receive request");
    }

    content[ret] = '\0';
    ESP_LOGD(TAG, "Received content: %s", content);

    *json = cJSON_Parse(content);
    free(content);
    
    if (*json == NULL) {
        ESP_LOGE(TAG, "Failed to parse JSON");
        return httpd_resp_send_err(req, HTTPD_400_BAD_REQUEST, "Invalid JSON");
    }

    return ESP_OK;
}

static esp_err_t send_json_response(httpd_req_t *req, cJSON *json)
{
    char *json_str = cJSON_Print(json);
    if (json_str == NULL) {
        ESP_LOGE(TAG, "Failed to print JSON");
        return httpd_resp_send_err(req, HTTPD_500_INTERNAL_SERVER_ERROR, "Internal server error");
    }

    httpd_resp_set_type(req, "application/json");
    httpd_resp_set_status(req, "200 OK");
    httpd_resp_send(req, json_str, strlen(json_str));
    
    free(json_str);
    return ESP_OK;
}
