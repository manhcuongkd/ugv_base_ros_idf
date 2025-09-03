#include "../inc/esp_now_controller.h"
#include <esp_now.h>
#include <esp_wifi.h>
#include <esp_log.h>
#include <esp_timer.h>
#include <freertos/FreeRTOS.h>
#include <freertos/task.h>
#include <freertos/queue.h>
#include <string.h>
#include <cJSON.h>

static const char *TAG = "ESP_NOW_Controller";

// ESP-NOW Configuration
// Channel is defined in header
#define ESP_NOW_MAX_PEERS 10
#define ESP_NOW_QUEUE_SIZE 10
#define ESP_NOW_TASK_STACK_SIZE 4096
#define ESP_NOW_FLOW_CONTROL_INTERVAL 1000 // ms

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

    // Initialize WiFi in AP mode for ESP-NOW
    wifi_config_t ap_config = {0};
    strcpy((char*)ap_config.ap.ssid, "RaspRover_ESP_NOW");
    strcpy((char*)ap_config.ap.password, "12345678");
    ap_config.ap.channel = ESP_NOW_CHANNEL;
    ap_config.ap.authmode = WIFI_AUTH_WPA2_PSK;
    ap_config.ap.max_connection = 4;
    ap_config.ap.beacon_interval = 100;

    esp_err_t ret = esp_wifi_set_mode(WIFI_MODE_AP);
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

    esp_now_initialized = true;
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

    esp_now_peer_info_t peer_info = {0};
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
    esp_now_message_t msg = {0};
    
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
    ESP_LOGI(TAG, "Flow control interval set to %u ms", interval_ms);
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
void esp_now_controller_on_data_sent(const esp_now_send_cb_t *cb, esp_now_send_status_t status)
{
    if (status == ESP_NOW_SEND_SUCCESS) {
        ESP_LOGI(TAG, "Data sent successfully");
    } else {
        ESP_LOGW(TAG, "Data send failed: %d", status);
    }
}

void esp_now_controller_on_data_recv(const esp_now_recv_cb_t *cb, const uint8_t *data, int data_len)
{
    if (data == NULL || data_len != sizeof(esp_now_message_t)) {
        ESP_LOGW(TAG, "Invalid data received: %d bytes", data_len);
        return;
    }

    // Parse received message
    const esp_now_message_t *msg = (const esp_now_message_t *)data;
    
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
            
            // Process message based on command type
            switch (msg.cmd) {
                case 0: // No command
                    break;
                case 1: // Move base
                    ESP_LOGI(TAG, "Moving base to %.2f", msg.base);
                    break;
                case 2: // Move shoulder
                    ESP_LOGI(TAG, "Moving shoulder to %.2f", msg.shoulder);
                    break;
                case 3: // Move elbow
                    ESP_LOGI(TAG, "Moving elbow to %.2f", msg.elbow);
                    break;
                case 4: // Move hand
                    ESP_LOGI(TAG, "Moving hand to %.2f", msg.hand);
                    break;
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
            esp_now_message_t flow_msg = {0};
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
