#include "../inc/esp_now_controller.h"
#include <esp_now.h>
#include <esp_wifi.h>
#include <esp_log.h>
#include <inttypes.h>
#include <esp_timer.h>
#include <freertos/FreeRTOS.h>
#include <freertos/task.h>
#include <freertos/queue.h>
#include <string.h>
#include <cJSON.h>

static const char *TAG = "ESP_NOW_Controller";

// Global variables
static bool esp_now_initialized = false;
static esp_now_control_t esp_now_control;
static QueueHandle_t esp_now_queue = NULL;
static TaskHandle_t esp_now_task_handle = NULL;
static TaskHandle_t flow_control_task_handle = NULL;

// Private function prototypes
void esp_now_controller_task(void *pvParameters);
void esp_now_controller_flow_control_task(void *pvParameters);
void esp_now_controller_on_data_sent(const uint8_t *mac_addr, esp_now_send_status_t status);
void esp_now_controller_on_data_recv(const uint8_t *mac_addr, const uint8_t *data, int data_len);

esp_err_t esp_now_controller_init(void)
{
    if (esp_now_initialized) {
        ESP_LOGW(TAG, "ESP-NOW controller already initialized");
        return ESP_OK;
    }

    ESP_LOGI(TAG, "Initializing ESP-NOW controller...");

    // Initialize WiFi (required for ESP-NOW)
    wifi_init_config_t cfg = WIFI_INIT_CONFIG_DEFAULT();
    esp_err_t ret = esp_wifi_init(&cfg);
    if (ret != ESP_OK) {
        ESP_LOGE(TAG, "Failed to initialize WiFi: %s", esp_err_to_name(ret));
        return ret;
    }

    // Initialize WiFi in AP mode for ESP-NOW
    wifi_config_t ap_config = {};
    strcpy((char*)ap_config.ap.ssid, "RaspRover_ESP_NOW");
    strcpy((char*)ap_config.ap.password, "12345678");
    ap_config.ap.ssid_len = strlen("RaspRover_ESP_NOW");
    ap_config.ap.channel = ESP_NOW_CHANNEL;
    ap_config.ap.authmode = WIFI_AUTH_WPA2_PSK;
    ap_config.ap.ssid_hidden = 0;
    ap_config.ap.max_connection = 4;
    ap_config.ap.beacon_interval = 100;
    ap_config.ap.pairwise_cipher = WIFI_CIPHER_TYPE_CCMP;
    ap_config.ap.ftm_responder = false;

    ret = esp_wifi_set_mode(WIFI_MODE_AP);
    if (ret != ESP_OK) {
        ESP_LOGE(TAG, "Failed to set WiFi mode: %s", esp_err_to_name(ret));
        return ret;
    }

    ret = esp_wifi_set_config(WIFI_IF_AP, &ap_config);
    if (ret != ESP_OK) {
        ESP_LOGE(TAG, "Failed to set WiFi config: %s", esp_err_to_name(ret));
        return ret;
    }

    ret = esp_wifi_start();
    if (ret != ESP_OK) {
        ESP_LOGE(TAG, "Failed to start WiFi: %s", esp_err_to_name(ret));
        return ret;
    }

    // Initialize ESP-NOW
    ret = esp_now_init();
    if (ret != ESP_OK) {
        ESP_LOGE(TAG, "Failed to initialize ESP-NOW: %s", esp_err_to_name(ret));
        return ret;
    }

    // Register callbacks
    ret = esp_now_register_recv_cb(esp_now_controller_on_data_recv);
    if (ret != ESP_OK) {
        ESP_LOGE(TAG, "Failed to register receive callback: %s", esp_err_to_name(ret));
        return ret;
    }

    ret = esp_now_register_send_cb(esp_now_controller_on_data_sent);
    if (ret != ESP_OK) {
        ESP_LOGE(TAG, "Failed to register send callback: %s", esp_err_to_name(ret));
        return ret;
    }

    // Create queue for ESP-NOW messages
    esp_now_queue = xQueueCreate(ESP_NOW_QUEUE_SIZE, sizeof(esp_now_message_t));
    if (esp_now_queue == NULL) {
        ESP_LOGE(TAG, "Failed to create ESP-NOW queue");
        return ESP_ERR_NO_MEM;
    }

    // Initialize control structure
    memset(&esp_now_control, 0, sizeof(esp_now_control_t));
    esp_now_control.mode = ESP_NOW_MODE_NONE;
    esp_now_control.follower_count = 0;
    esp_now_control.flow_control_interval_ms = ESP_NOW_FLOW_CONTROL_INTERVAL;
    esp_now_control.ctrl_by_broadcast = true;  // Arduino default

    // Mark as initialized before creating tasks (prevents timing errors)
    esp_now_initialized = true;

    // Create ESP-NOW task
    BaseType_t task_created = xTaskCreate(
        esp_now_controller_task,
        "esp_now_task",
        ESP_NOW_TASK_STACK_SIZE,
        NULL,
        5,
        &esp_now_task_handle
    );

    if (task_created != pdPASS) {
        ESP_LOGE(TAG, "Failed to create ESP-NOW task");
        vQueueDelete(esp_now_queue);
        return ESP_ERR_NO_MEM;
    }

    // Create flow control task
    task_created = xTaskCreate(
        esp_now_controller_flow_control_task,
        "flow_control_task",
        ESP_NOW_TASK_STACK_SIZE,
        NULL,
        4,
        &flow_control_task_handle
    );

    if (task_created != pdPASS) {
        ESP_LOGE(TAG, "Failed to create flow control task");
        vTaskDelete(esp_now_task_handle);
        vQueueDelete(esp_now_queue);
        return ESP_ERR_NO_MEM;
    }

    ESP_LOGI(TAG, "ESP-NOW controller initialized successfully");
    return ESP_OK;
}

esp_err_t esp_now_controller_deinit(void)
{
    if (!esp_now_initialized) {
        return ESP_OK;
    }

    ESP_LOGI(TAG, "Deinitializing ESP-NOW controller...");

    // Delete tasks
    if (esp_now_task_handle != NULL) {
        vTaskDelete(esp_now_task_handle);
        esp_now_task_handle = NULL;
    }

    if (flow_control_task_handle != NULL) {
        vTaskDelete(flow_control_task_handle);
        flow_control_task_handle = NULL;
    }

    // Delete queue
    if (esp_now_queue != NULL) {
        vQueueDelete(esp_now_queue);
        esp_now_queue = NULL;
    }

    // Deinitialize ESP-NOW
    esp_err_t ret = esp_now_deinit();
    if (ret != ESP_OK) {
        ESP_LOGW(TAG, "Failed to deinitialize ESP-NOW: %s", esp_err_to_name(ret));
    }

    // Stop WiFi
    ret = esp_wifi_stop();
    if (ret != ESP_OK) {
        ESP_LOGW(TAG, "Failed to stop WiFi: %s", esp_err_to_name(ret));
    }

    esp_now_initialized = false;
    ESP_LOGI(TAG, "ESP-NOW controller deinitialized");
    return ESP_OK;
}

esp_err_t esp_now_controller_add_peer(const uint8_t *mac_addr)
{
    if (!esp_now_initialized) {
        ESP_LOGE(TAG, "ESP-NOW controller not initialized");
        return ESP_ERR_INVALID_STATE;
    }

    if (mac_addr == NULL) {
        return ESP_ERR_INVALID_ARG;
    }

    if (esp_now_control.follower_count >= MAX_FOLLOWERS) {
        ESP_LOGW(TAG, "Maximum follower count reached");
        return ESP_ERR_NO_MEM;
    }

    esp_now_peer_info_t peer_info = {};
    memset(&peer_info, 0, sizeof(peer_info));
    memcpy(peer_info.peer_addr, mac_addr, 6);
    peer_info.channel = ESP_NOW_CHANNEL;
    peer_info.encrypt = false;

    esp_err_t ret = esp_now_add_peer(&peer_info);
    if (ret != ESP_OK) {
        ESP_LOGE(TAG, "Failed to add peer: %s", esp_err_to_name(ret));
        return ret;
    }

    // Add to local follower list
    memcpy(esp_now_control.followers[esp_now_control.follower_count].mac, mac_addr, 6);
    esp_now_control.followers[esp_now_control.follower_count].active = true;
    esp_now_control.followers[esp_now_control.follower_count].last_seen = esp_timer_get_time() / 1000;
    esp_now_control.follower_count++;

    ESP_LOGI(TAG, "Peer added successfully: %02x:%02x:%02x:%02x:%02x:%02x",
             mac_addr[0], mac_addr[1], mac_addr[2], mac_addr[3], mac_addr[4], mac_addr[5]);

    return ESP_OK;
}

esp_err_t esp_now_controller_remove_peer(const uint8_t *mac_addr)
{
    if (!esp_now_initialized) {
        ESP_LOGE(TAG, "ESP-NOW controller not initialized");
        return ESP_ERR_INVALID_STATE;
    }

    if (mac_addr == NULL) {
        return ESP_ERR_INVALID_ARG;
    }

    esp_err_t ret = esp_now_del_peer(mac_addr);
    if (ret != ESP_OK) {
        ESP_LOGE(TAG, "Failed to remove peer: %s", esp_err_to_name(ret));
        return ret;
    }

    // Remove from local peer list
    for (int i = 0; i < esp_now_control.follower_count; i++) {
        if (memcmp(esp_now_control.followers[i].mac, mac_addr, 6) == 0) {
            esp_now_control.followers[i].active = false;
            // Move last follower to this position
            if (i < esp_now_control.follower_count - 1) {
                memcpy(&esp_now_control.followers[i], &esp_now_control.followers[esp_now_control.follower_count - 1], sizeof(esp_now_peer_t));
            }
            esp_now_control.follower_count--;
            break;
        }
    }

    ESP_LOGI(TAG, "Peer removed successfully: %02x:%02x:%02x:%02x:%02x:%02x",
             mac_addr[0], mac_addr[1], mac_addr[2], mac_addr[3], mac_addr[4], mac_addr[5]);

    return ESP_OK;
}

esp_err_t esp_now_controller_send_data(const uint8_t *mac_addr, const uint8_t *data, size_t data_len)
{
    if (!esp_now_initialized) {
        ESP_LOGE(TAG, "ESP-NOW controller not initialized");
        return ESP_ERR_INVALID_STATE;
    }

    if (mac_addr == NULL || data == NULL || data_len == 0) {
        return ESP_ERR_INVALID_ARG;
    }

    esp_err_t ret = esp_now_send(mac_addr, data, data_len);
    if (ret != ESP_OK) {
        ESP_LOGE(TAG, "Failed to send data: %s", esp_err_to_name(ret));
        return ret;
    }

    ESP_LOGI(TAG, "Data sent successfully to %02x:%02x:%02x:%02x:%02x:%02x (%d bytes)",
             mac_addr[0], mac_addr[1], mac_addr[2], mac_addr[3], mac_addr[4], mac_addr[5], data_len);

    return ESP_OK;
}

esp_err_t esp_now_controller_broadcast_data(const uint8_t *data, size_t data_len)
{
    if (!esp_now_initialized) {
        ESP_LOGE(TAG, "ESP-NOW controller not initialized");
        return ESP_ERR_INVALID_STATE;
    }

    if (data == NULL || data_len == 0) {
        return ESP_ERR_INVALID_ARG;
    }

    esp_err_t ret = ESP_OK;
    int success_count = 0;

    // Send to all active followers
    for (int i = 0; i < esp_now_control.follower_count; i++) {
        if (esp_now_control.followers[i].active) {
            ret = esp_now_send(esp_now_control.followers[i].mac, data, data_len);
            if (ret == ESP_OK) {
                success_count++;
            } else {
                ESP_LOGW(TAG, "Failed to send to follower %d: %s", i, esp_err_to_name(ret));
            }
        }
    }

    ESP_LOGI(TAG, "Broadcast completed: %d/%d followers successful", success_count, esp_now_control.follower_count);
    return (success_count > 0) ? ESP_OK : ESP_FAIL;
}

esp_err_t esp_now_controller_send_message(const esp_now_message_t *message)
{
    if (!esp_now_initialized) {
        ESP_LOGE(TAG, "ESP-NOW controller not initialized");
        return ESP_ERR_INVALID_STATE;
    }

    if (message == NULL) {
        return ESP_ERR_INVALID_ARG;
    }

    // Add message to queue for processing
    if (xQueueSend(esp_now_queue, message, pdMS_TO_TICKS(100)) != pdTRUE) {
        ESP_LOGW(TAG, "Failed to queue message - queue full");
        return ESP_ERR_TIMEOUT;
    }

    ESP_LOGI(TAG, "Message queued for sending");
    return ESP_OK;
}

esp_err_t esp_now_controller_send_json_command(const char *json_cmd)
{
    if (!esp_now_initialized) {
        ESP_LOGE(TAG, "ESP-NOW controller not initialized");
        return ESP_ERR_INVALID_STATE;
    }

    if (json_cmd == NULL) {
        return ESP_ERR_INVALID_ARG;
    }

    // Parse JSON command
    cJSON *json = cJSON_Parse(json_cmd);
    if (json == NULL) {
        ESP_LOGE(TAG, "Failed to parse JSON command");
        return ESP_ERR_INVALID_ARG;
    }

    // Create message structure
    esp_now_message_t msg = {};
    memset(&msg, 0, sizeof(msg));
    
    // Extract values from JSON
    cJSON *dev_code_item = cJSON_GetObjectItem(json, "dev_code");
    if (cJSON_IsNumber(dev_code_item)) {
        msg.dev_code = dev_code_item->valueint;
    }

    cJSON *base_item = cJSON_GetObjectItem(json, "base");
    if (cJSON_IsNumber(base_item)) {
        msg.base = base_item->valuedouble;
    }

    cJSON *shoulder_item = cJSON_GetObjectItem(json, "shoulder");
    if (cJSON_IsNumber(shoulder_item)) {
        msg.shoulder = shoulder_item->valuedouble;
    }

    cJSON *elbow_item = cJSON_GetObjectItem(json, "elbow");
    if (cJSON_IsNumber(elbow_item)) {
        msg.elbow = elbow_item->valuedouble;
    }

    cJSON *hand_item = cJSON_GetObjectItem(json, "hand");
    if (cJSON_IsNumber(hand_item)) {
        msg.hand = hand_item->valueint;
    }

    cJSON *cmd_item = cJSON_GetObjectItem(json, "cmd");
    if (cJSON_IsNumber(cmd_item)) {
        msg.cmd = cmd_item->valueint;
    }

    cJSON *message_item = cJSON_GetObjectItem(json, "message");
    if (cJSON_IsString(message_item)) {
        strncpy(msg.message, message_item->valuestring, sizeof(msg.message) - 1);
        msg.message[sizeof(msg.message) - 1] = '\0';
    }

    cJSON_Delete(json);

    // Send the message
    return esp_now_controller_send_message(&msg);
}

esp_err_t esp_now_controller_set_mode(uint8_t mode)
{
    if (!esp_now_initialized) {
        ESP_LOGE(TAG, "ESP-NOW controller not initialized");
        return ESP_ERR_INVALID_STATE;
    }

    esp_now_control.mode = mode;
    ESP_LOGI(TAG, "ESP-NOW mode set to %d", mode);
    return ESP_OK;
}

esp_err_t esp_now_controller_set_flow_control_interval(uint32_t interval_ms)
{
    if (!esp_now_initialized) {
        ESP_LOGE(TAG, "ESP-NOW controller not initialized");
        return ESP_ERR_INVALID_STATE;
    }

    esp_now_control.flow_control_interval_ms = interval_ms;
    ESP_LOGI(TAG, "Flow control interval set to %" PRIu32 " ms", interval_ms);
    return ESP_OK;
}

esp_err_t esp_now_controller_get_peer_count(uint8_t *count)
{
    if (!esp_now_initialized) {
        ESP_LOGE(TAG, "ESP-NOW controller not initialized");
        return ESP_ERR_INVALID_STATE;
    }

    if (count == NULL) {
        return ESP_ERR_INVALID_ARG;
    }

    *count = esp_now_control.follower_count;
    return ESP_OK;
}

esp_err_t esp_now_controller_get_peer_info(uint8_t index, esp_now_peer_t *peer_info)
{
    if (!esp_now_initialized) {
        ESP_LOGE(TAG, "ESP-NOW controller not initialized");
        return ESP_ERR_INVALID_STATE;
    }

    if (peer_info == NULL || index >= esp_now_control.follower_count) {
        return ESP_ERR_INVALID_ARG;
    }

    memcpy(peer_info, &esp_now_control.followers[index], sizeof(esp_now_peer_t));
    return ESP_OK;
}

esp_err_t esp_now_controller_get_status(esp_now_control_t *status)
{
    if (!esp_now_initialized) {
        ESP_LOGE(TAG, "ESP-NOW controller not initialized");
        return ESP_ERR_INVALID_STATE;
    }

    if (status == NULL) {
        return ESP_ERR_INVALID_ARG;
    }

    memcpy(status, &esp_now_control, sizeof(esp_now_control_t));
    return ESP_OK;
}

// Private functions
void esp_now_controller_on_data_sent(const uint8_t *mac_addr, esp_now_send_status_t status)
{
    if (status == ESP_NOW_SEND_SUCCESS) {
        ESP_LOGI(TAG, "Data sent successfully");
    } else {
        ESP_LOGW(TAG, "Data send failed: %d", status);
    }
}

void esp_now_controller_on_data_recv(const uint8_t *mac_addr, const uint8_t *data, int data_len)
{
    if (data == NULL || data_len != sizeof(esp_now_message_t)) {
        ESP_LOGW(TAG, "Invalid data received: %d bytes", data_len);
        return;
    }

    // Parse received message
    const esp_now_message_t *msg = (const esp_now_message_t *)data;
    
    // Arduino compatibility: Only process if in follower mode
    if (esp_now_control.mode != ESP_NOW_MODE_FOLLOWER) {
        return;
    }
    
    // Send receive feedback for command 3 (Arduino compatibility)
    if (msg->cmd == 3) {
        char mac_str[18];
        char *mac_string = esp_now_mac_to_string(mac_addr);
        if (mac_string) {
            strncpy(mac_str, mac_string, 18);
            mac_str[17] = '\0';
            free(mac_string);
            
            esp_now_controller_send_json_feedback(CMD_ESP_NOW_RECV, 0, msg->message, mac_str);
        }
    }
    
    // Arduino compatibility: Check broadcast control and whitelist
    if (!esp_now_control.ctrl_by_broadcast) {
        if (memcmp(mac_addr, esp_now_control.mac_whitelist_broadcast, 6) != 0) {
            return;
        }
    }
    
    ESP_LOGI(TAG, "Received ESP-NOW message: dev_code=%d, base=%.2f, shoulder=%.2f, elbow=%.2f, hand=%.2f, cmd=%d",
             msg->dev_code, msg->base, msg->shoulder, msg->elbow, msg->hand, msg->cmd);

    // Add to queue for processing
    if (xQueueSend(esp_now_queue, msg, pdMS_TO_TICKS(100)) != pdTRUE) {
        ESP_LOGW(TAG, "Failed to queue received message - queue full");
    }
}

void esp_now_controller_task(void *pvParameters)
{
    esp_now_message_t msg;
    
    ESP_LOGI(TAG, "ESP-NOW task started");
    
    while (1) {
        // Wait for messages from queue
        if (xQueueReceive(esp_now_queue, &msg, pdMS_TO_TICKS(1000)) == pdTRUE) {
            ESP_LOGI(TAG, "Processing message: dev_code=%d, cmd=%d", msg.dev_code, msg.cmd);
            
            // Process message based on command type (Arduino compatible)
            switch (msg.cmd) {
                case 0: {
                    // All joint absolute control (Arduino compatibility)
                    ESP_LOGI(TAG, "RoArm joint control: base=%.2f, shoulder=%.2f, elbow=%.2f, hand=%.2f", 
                             msg.base, msg.shoulder, msg.elbow, msg.hand);
                    // TODO: Call RoArmM2_allJointAbsCtrl(msg.base, msg.shoulder, msg.elbow, msg.hand, 0, 0);
                    break;
                }
                case 1: {
                    // JSON command processing (Arduino compatibility)
                    ESP_LOGI(TAG, "Processing JSON command: %s", msg.message);
                    // TODO: Call jsonCmdReceiveHandler();
                    break;
                }
                case 2: {
                    // JSON command with immediate execution (Arduino compatibility)
                    ESP_LOGI(TAG, "Processing immediate JSON command: %s", msg.message);
                    // TODO: Set runNewJsonCmd = true;
                    break;
                }
                case 3: {
                    // Message command (already handled in receive callback)
                    ESP_LOGI(TAG, "Message command: %s", msg.message);
                    break;
                }
                default:
                    ESP_LOGW(TAG, "Unknown command: %d", msg.cmd);
                    break;
            }
        }
        
        // Yield to other tasks
        vTaskDelay(pdMS_TO_TICKS(10));
    }
}

void esp_now_controller_flow_control_task(void *pvParameters)
{
    ESP_LOGI(TAG, "ESP-NOW flow control task started");
    
    while (1) {
        // Send flow control message periodically
        if (esp_now_control.flow_control_interval_ms > 0) {
            esp_now_message_t flow_msg = {};
            memset(&flow_msg, 0, sizeof(flow_msg));
            flow_msg.cmd = 255; // Flow control command
            strcpy(flow_msg.message, "FLOW_CONTROL");
            
            // Broadcast flow control message
            esp_now_controller_broadcast_data((uint8_t*)&flow_msg, sizeof(esp_now_message_t));
            
            ESP_LOGI(TAG, "Flow control message sent");
        }
        
        // Wait for next interval
        vTaskDelay(pdMS_TO_TICKS(esp_now_control.flow_control_interval_ms));
    }
}

esp_err_t esp_now_controller_set_broadcast_mode(uint8_t mode, const char *mac) {
    if (!esp_now_initialized) {
        ESP_LOGE(TAG, "ESP-NOW controller not initialized");
        return ESP_ERR_INVALID_STATE;
    }

    ESP_LOGI(TAG, "Setting broadcast mode: %d, MAC: %s", mode, mac);

    // Set broadcast mode
    esp_now_control.mode = mode;

    // If MAC is provided, add it as a follower
    if (mac && strlen(mac) > 0) {
        uint8_t mac_bytes[6];
        if (esp_now_string_to_mac(mac, mac_bytes) == ESP_OK) {
            esp_now_controller_add_peer(mac_bytes);
        } else {
            ESP_LOGE(TAG, "Invalid MAC address format: %s", mac);
            return ESP_ERR_INVALID_ARG;
        }
    }

    ESP_LOGI(TAG, "Broadcast mode set successfully");
    return ESP_OK;
}

esp_err_t esp_now_controller_get_mac_address(char *mac_str) {
    if (!esp_now_initialized) {
        ESP_LOGE(TAG, "ESP-NOW controller not initialized");
        return ESP_ERR_INVALID_STATE;
    }

    if (mac_str == NULL) {
        ESP_LOGE(TAG, "Invalid MAC string buffer");
        return ESP_ERR_INVALID_ARG;
    }

    // Get this device's MAC address
    uint8_t mac[6];
    esp_now_controller_get_this_mac(mac);
    
    // Convert to string
    char *mac_string = esp_now_mac_to_string(mac);
    if (mac_string) {
        strncpy(mac_str, mac_string, 18);
        mac_str[17] = '\0'; // Ensure null termination
        free(mac_string);
        
        ESP_LOGI(TAG, "Device MAC address: %s", mac_str);
        return ESP_OK;
    } else {
        ESP_LOGE(TAG, "Failed to convert MAC to string");
        return ESP_FAIL;
    }
}

esp_err_t esp_now_controller_send_recv_feedback(const char *mac, const char *message) {
    if (!esp_now_initialized) {
        ESP_LOGE(TAG, "ESP-NOW controller not initialized");
        return ESP_ERR_INVALID_STATE;
    }
    
    if (mac == NULL || message == NULL) {
        ESP_LOGE(TAG, "Invalid parameters");
        return ESP_ERR_INVALID_ARG;
    }
    
    // Create feedback JSON
    cJSON *json = cJSON_CreateObject();
    cJSON_AddNumberToObject(json, "T", CMD_ESP_NOW_RECV);
    cJSON_AddStringToObject(json, "mac", mac);
    cJSON_AddStringToObject(json, "megs", message);
    
    char *json_string = cJSON_Print(json);
    if (json_string) {
        // Send via UART (matching original Arduino behavior)
        printf("%s\n", json_string);
        free(json_string);
    }
    
    cJSON_Delete(json);
    ESP_LOGI(TAG, "ESP-NOW receive feedback sent: MAC=%s, Message=%s", mac, message);
    return ESP_OK;
}

esp_err_t esp_now_controller_send_send_feedback(const char *mac, int status, const char *message) {
    if (!esp_now_initialized) {
        ESP_LOGE(TAG, "ESP-NOW controller not initialized");
        return ESP_ERR_INVALID_STATE;
    }
    
    if (mac == NULL || message == NULL) {
        ESP_LOGE(TAG, "Invalid parameters");
        return ESP_ERR_INVALID_ARG;
    }
    
    // Create feedback JSON
    cJSON *json = cJSON_CreateObject();
    cJSON_AddNumberToObject(json, "T", CMD_ESP_NOW_SEND);
    cJSON_AddStringToObject(json, "mac", mac);
    cJSON_AddNumberToObject(json, "status", status);
    cJSON_AddStringToObject(json, "megs", message);
    
    char *json_string = cJSON_Print(json);
    if (json_string) {
        // Send via UART (matching original Arduino behavior)
        printf("%s\n", json_string);
        free(json_string);
    }
    
    cJSON_Delete(json);
    ESP_LOGI(TAG, "ESP-NOW send feedback sent: MAC=%s, Status=%d, Message=%s", mac, status, message);
    return ESP_OK;
}

esp_err_t esp_now_string_to_mac(const char *mac_string, uint8_t *mac_bytes) {
    if (!mac_string || !mac_bytes) {
        return ESP_ERR_INVALID_ARG;
    }
    
    ESP_LOGI(TAG, "Converting MAC string to bytes: %s", mac_string);
    
    // Parse MAC address string (format: "XX:XX:XX:XX:XX:XX")
    int values[6];
    if (sscanf(mac_string, "%02x:%02x:%02x:%02x:%02x:%02x",
               &values[0], &values[1], &values[2], &values[3], &values[4], &values[5]) != 6) {
        ESP_LOGE(TAG, "Invalid MAC address format: %s", mac_string);
        return ESP_ERR_INVALID_ARG;
    }
    
    for (int i = 0; i < 6; i++) {
        mac_bytes[i] = (uint8_t)values[i];
    }
    
    ESP_LOGI(TAG, "MAC converted successfully");
    return ESP_OK;
}

esp_err_t esp_now_controller_get_this_mac(uint8_t *mac) {
    if (!mac) {
        return ESP_ERR_INVALID_ARG;
    }
    
    ESP_LOGI(TAG, "Getting this device MAC address");
    
    // Get WiFi MAC address
    esp_err_t ret = esp_wifi_get_mac(WIFI_IF_STA, mac);
    if (ret != ESP_OK) {
        ESP_LOGE(TAG, "Failed to get MAC address: %s", esp_err_to_name(ret));
        return ret;
    }
    
    ESP_LOGI(TAG, "MAC address retrieved successfully");
    return ESP_OK;
}

char* esp_now_mac_to_string(const uint8_t *mac) {
    if (!mac) {
        return NULL;
    }
    
    ESP_LOGI(TAG, "Converting MAC bytes to string");
    
    // Allocate memory for MAC string
    char *mac_string = (char*)malloc(18); // "XX:XX:XX:XX:XX:XX" + null terminator
    if (!mac_string) {
        ESP_LOGE(TAG, "Failed to allocate memory for MAC string");
        return NULL;
    }
    
    // Format MAC address
    snprintf(mac_string, 18, "%02x:%02x:%02x:%02x:%02x:%02x",
             mac[0], mac[1], mac[2], mac[3], mac[4], mac[5]);
    
    ESP_LOGI(TAG, "MAC string created: %s", mac_string);
    return mac_string;
}

// ===== ARDUINO-COMPATIBLE API IMPLEMENTATION =====

/**
 * @brief Change ESP-NOW mode (Arduino compatible)
 * @param input_mode Mode to set (0=none, 1=flow-leader-group, 2=flow-leader-single, 3=follower)
 * @return ESP_OK on success, error code otherwise
 */
esp_err_t esp_now_controller_change_mode(uint8_t input_mode) {
    if (!esp_now_initialized) {
        ESP_LOGE(TAG, "ESP-NOW controller not initialized");
        return ESP_ERR_INVALID_STATE;
    }

    esp_now_control.mode = input_mode;
    
    const char *mode_names[] = {"none", "flow-leader(group)", "flow-leader(single)", "follower"};
    // mode_displays for future OLED integration
    // const char *mode_displays[] = {"ESP-NOW: NONE", "ESP-NOW: F-LEADER-B", "ESP-NOW: F-LEADER-S", "ESP-NOW: > FOLLOWER <"};
    
    if (input_mode <= 3) {
        ESP_LOGI(TAG, "ESP-NOW mode: %s", mode_names[input_mode]);
        
        // Send JSON feedback (Arduino compatibility)
        esp_now_controller_send_info_feedback(mode_names[input_mode]);
        
        // TODO: Update OLED display
        // screenLine_3 = mode_displays[input_mode];
        // oled_update();
    }
    
    if (input_mode == ESP_NOW_MODE_FLOW_LEADER_GROUP || input_mode == ESP_NOW_MODE_FLOW_LEADER_SINGLE) {
        esp_now_control.current_message.cmd = 0;
    }
    
    return ESP_OK;
}

/**
 * @brief Register new follower to peer (Arduino compatible)
 * @param input_mac MAC address string in format "XX:XX:XX:XX:XX:XX"
 * @return ESP_OK on success, error code otherwise
 */
esp_err_t esp_now_controller_register_new_follower(const char *input_mac) {
    if (!esp_now_initialized) {
        ESP_LOGE(TAG, "ESP-NOW controller not initialized");
        return ESP_ERR_INVALID_STATE;
    }
    
    if (!input_mac || strlen(input_mac) != 17) {
        esp_now_controller_send_json_feedback(CMD_ESP_NOW_SEND, 3, "invalid MAC address format.", NULL);
        return ESP_ERR_INVALID_ARG;
    }
    
    uint8_t mac_array[6];
    esp_err_t ret = esp_now_string_to_mac(input_mac, mac_array);
    if (ret != ESP_OK) {
        esp_now_controller_send_json_feedback(CMD_ESP_NOW_SEND, 3, "invalid MAC address format.", NULL);
        return ret;
    }
    
    // Store as single follower device (Arduino compatibility)
    memcpy(esp_now_control.single_follower_dev, mac_array, 6);
    
    ret = esp_now_controller_add_peer(mac_array);
    if (ret != ESP_OK) {
        esp_now_controller_send_json_feedback(CMD_ESP_NOW_SEND, 4, "Failed to add peer.", input_mac);
        return ret;
    }
    
    esp_now_controller_send_json_feedback(CMD_ESP_NOW_SEND, 5, "add peer.", input_mac);
    return ESP_OK;
}

/**
 * @brief Delete follower (Arduino compatible)
 * @param input_mac MAC address string to remove
 * @return ESP_OK on success, error code otherwise
 */
esp_err_t esp_now_controller_delete_follower(const char *input_mac) {
    if (!esp_now_initialized) {
        ESP_LOGE(TAG, "ESP-NOW controller not initialized");
        return ESP_ERR_INVALID_STATE;
    }
    
    if (!input_mac || strlen(input_mac) != 17) {
        esp_now_controller_send_json_feedback(CMD_ESP_NOW_SEND, 3, "invalid MAC address format.", NULL);
        return ESP_ERR_INVALID_ARG;
    }
    
    uint8_t mac_array[6];
    esp_err_t ret = esp_now_string_to_mac(input_mac, mac_array);
    if (ret != ESP_OK) {
        esp_now_controller_send_json_feedback(CMD_ESP_NOW_SEND, 3, "invalid MAC address format.", NULL);
        return ret;
    }
    
    ret = esp_now_controller_remove_peer(mac_array);
    esp_now_controller_send_json_feedback(CMD_ESP_NOW_SEND, 6, "delete peer.", input_mac);
    
    return ESP_OK;
}

/**
 * @brief Send message to group (Arduino compatible)
 * @param dev_code Device code
 * @param b Base value
 * @param s Shoulder value
 * @param e Elbow value
 * @param h Hand value
 * @param cmd Command code
 * @param message Message string
 * @return ESP_OK on success, error code otherwise
 */
esp_err_t esp_now_controller_group_send(uint8_t dev_code, float b, float s, float e, float h, uint8_t cmd, const char *message) {
    if (!esp_now_initialized) {
        ESP_LOGE(TAG, "ESP-NOW controller not initialized");
        return ESP_ERR_INVALID_STATE;
    }
    
    esp_now_message_t msg = {};
    memset(&msg, 0, sizeof(msg));
    
    msg.dev_code = dev_code;
    msg.base = b;
    msg.shoulder = s;
    msg.elbow = e;
    msg.hand = h;
    msg.cmd = cmd;
    
    if (message) {
        strncpy(msg.message, message, sizeof(msg.message) - 1);
        msg.message[sizeof(msg.message) - 1] = '\0';
    }
    
    esp_err_t ret = esp_now_controller_broadcast_data((uint8_t*)&msg, sizeof(esp_now_message_t));
    int status = (ret == ESP_OK) ? 8 : 7;
    const char *status_msg = (ret == ESP_OK) ? "sent with success." : "error sending the data.";
    
    esp_now_controller_send_json_feedback(CMD_ESP_NOW_SEND, status, status_msg, NULL);
    
    return ret;
}

/**
 * @brief Send message to single device (Arduino compatible)
 * @param input_mac Target MAC address
 * @param dev_code Device code
 * @param b Base value
 * @param s Shoulder value
 * @param e Elbow value
 * @param h Hand value
 * @param cmd Command code
 * @param message Message string
 * @return ESP_OK on success, error code otherwise
 */
esp_err_t esp_now_controller_single_dev_send(const char *input_mac, uint8_t dev_code, float b, float s, float e, float h, uint8_t cmd, const char *message) {
    if (!esp_now_initialized) {
        ESP_LOGE(TAG, "ESP-NOW controller not initialized");
        return ESP_ERR_INVALID_STATE;
    }
    
    if (!input_mac || strlen(input_mac) != 17) {
        esp_now_controller_send_json_feedback(CMD_ESP_NOW_SEND, 3, "invalid MAC address format.", NULL);
        return ESP_ERR_INVALID_ARG;
    }
    
    uint8_t mac_array[6];
    esp_err_t ret = esp_now_string_to_mac(input_mac, mac_array);
    if (ret != ESP_OK) {
        esp_now_controller_send_json_feedback(CMD_ESP_NOW_SEND, 3, "invalid MAC address format.", NULL);
        return ret;
    }
    
    esp_now_message_t msg = {};
    memset(&msg, 0, sizeof(msg));
    
    msg.dev_code = dev_code;
    msg.base = b;
    msg.shoulder = s;
    msg.elbow = e;
    msg.hand = h;
    msg.cmd = cmd;
    
    if (message) {
        strncpy(msg.message, message, sizeof(msg.message) - 1);
        msg.message[sizeof(msg.message) - 1] = '\0';
    }
    
    // Store single follower device (Arduino compatibility)
    memcpy(esp_now_control.single_follower_dev, mac_array, 6);
    
    ret = esp_now_controller_send_data(mac_array, (uint8_t*)&msg, sizeof(esp_now_message_t));
    int status = (ret == ESP_OK) ? 8 : 7;
    const char *status_msg = (ret == ESP_OK) ? "sent with success." : "error sending the data.";
    
    esp_now_controller_send_json_feedback(CMD_ESP_NOW_SEND, status, status_msg, NULL);
    
    return ret;
}

/**
 * @brief Single device flow control (Arduino compatible)
 * @return ESP_OK on success, error code otherwise
 */
esp_err_t esp_now_controller_single_dev_flow_ctrl(void) {
    if (!esp_now_initialized) {
        ESP_LOGE(TAG, "ESP-NOW controller not initialized");
        return ESP_ERR_INVALID_STATE;
    }
    
    esp_now_message_t msg = {};
    memset(&msg, 0, sizeof(msg));
    
    // TODO: Get current joint values from servo controller
    // msg.base = radB;
    // msg.shoulder = radS;
    // msg.elbow = radE;  
    // msg.hand = radG;
    
    esp_err_t ret = esp_now_controller_send_data(esp_now_control.single_follower_dev, (uint8_t*)&msg, sizeof(esp_now_message_t));
    int status = (ret == ESP_OK) ? 8 : 7;
    const char *status_msg = (ret == ESP_OK) ? "sent with success." : "error sending the data.";
    
    esp_now_controller_send_json_feedback(CMD_ESP_NOW_SEND, status, status_msg, NULL);
    
    return ret;
}

/**
 * @brief Group devices flow control (Arduino compatible)
 * @return ESP_OK on success, error code otherwise
 */
esp_err_t esp_now_controller_group_devs_flow_ctrl(void) {
    if (!esp_now_initialized) {
        ESP_LOGE(TAG, "ESP-NOW controller not initialized");
        return ESP_ERR_INVALID_STATE;
    }
    
    esp_now_message_t msg = {};
    memset(&msg, 0, sizeof(msg));
    
    // TODO: Get current joint values from servo controller
    // msg.base = radB;
    // msg.shoulder = radS;
    // msg.elbow = radE;
    // msg.hand = radG;
    
    esp_err_t ret = esp_now_controller_broadcast_data((uint8_t*)&msg, sizeof(esp_now_message_t));
    int status = (ret == ESP_OK) ? 8 : 7;
    const char *status_msg = (ret == ESP_OK) ? "sent with success." : "error sending the data.";
    
    esp_now_controller_send_json_feedback(CMD_ESP_NOW_SEND, status, status_msg, NULL);
    
    return ret;
}

/**
 * @brief Change broadcast mode (Arduino compatible)
 * @param input_mode True to enable broadcast control, false to disable
 * @param input_mac MAC address for whitelist
 * @return ESP_OK on success, error code otherwise
 */
esp_err_t esp_now_controller_change_broadcast_mode(bool input_mode, const char *input_mac) {
    if (!esp_now_initialized) {
        ESP_LOGE(TAG, "ESP-NOW controller not initialized");
        return ESP_ERR_INVALID_STATE;
    }
    
    esp_now_control.ctrl_by_broadcast = input_mode;
    
    if (input_mac && strlen(input_mac) == 17) {
        uint8_t mac_array[6];
        esp_err_t ret = esp_now_string_to_mac(input_mac, mac_array);
        if (ret == ESP_OK) {
            memcpy(esp_now_control.mac_whitelist_broadcast, mac_array, 6);
        }
    }
    
    const char *status_msg = input_mode ? 
        "it can be ctrl by esp-now broadcast cmd." : 
        "it won't be ctrl by esp-now broadcast cmd.";
    
    ESP_LOGI(TAG, "%s", status_msg);
    esp_now_controller_send_info_feedback(status_msg);
    
    return ESP_OK;
}

/**
 * @brief Get this device MAC address (Arduino compatible)
 * @param mac_str Buffer to store MAC address string (minimum 18 chars)
 */
void esp_now_controller_get_this_dev_mac_address(char *mac_str) {
    if (!mac_str) {
        return;
    }
    
    uint8_t mac[6];
    esp_err_t ret = esp_now_controller_get_this_mac(mac);
    if (ret == ESP_OK) {
        char *mac_string = esp_now_mac_to_string(mac);
        if (mac_string) {
            strncpy(mac_str, mac_string, 18);
            mac_str[17] = '\0';
            free(mac_string);
            
            // Store in control structure
            strncpy(esp_now_control.this_dev_mac_str, mac_str, 18);
            memcpy(esp_now_control.this_dev_mac, mac, 6);
            
            ESP_LOGI(TAG, "Device MAC: %s", mac_str);
        }
    }
}

/**
 * @brief Send JSON feedback message (Arduino compatible)
 * @param type Message type (CMD_ESP_NOW_SEND, CMD_ESP_NOW_RECV)
 * @param status Status code
 * @param message Status message
 * @param mac MAC address (optional)
 */
void esp_now_controller_send_json_feedback(int type, int status, const char *message, const char *mac) {
    cJSON *json = cJSON_CreateObject();
    if (!json) {
        return;
    }
    
    cJSON_AddNumberToObject(json, "T", type);
    cJSON_AddNumberToObject(json, "status", status);
    cJSON_AddStringToObject(json, "megs", message ? message : "");
    
    if (mac) {
        cJSON_AddStringToObject(json, "mac", mac);
    }
    
    char *json_string = cJSON_Print(json);
    if (json_string) {
        printf("%s\n", json_string);  // Arduino Serial.println equivalent
        free(json_string);
    }
    
    cJSON_Delete(json);
}

/**
 * @brief Send info feedback message (Arduino compatible)
 * @param info Info message
 */
void esp_now_controller_send_info_feedback(const char *info) {
    cJSON *json = cJSON_CreateObject();
    if (!json) {
        return;
    }
    
    cJSON_AddStringToObject(json, "info", info ? info : "");
    
    char *json_string = cJSON_Print(json);
    if (json_string) {
        printf("%s\n", json_string);  // Arduino Serial.println equivalent
        free(json_string);
    }
    
    cJSON_Delete(json);
}
