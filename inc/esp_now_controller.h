#ifndef ESP_NOW_CONTROLLER_H
#define ESP_NOW_CONTROLLER_H

#include <esp_err.h>
#include <esp_now.h>
#include <esp_wifi.h>
#include <freertos/FreeRTOS.h>
#include <freertos/queue.h>
#include <freertos/task.h>
#include "json_parser.h"

// ESP-NOW Configuration
#define ESP_NOW_CHANNEL 0
#define ESP_NOW_ENCRYPT false
#define MAX_FOLLOWERS 10
#define MAX_MESSAGE_SIZE 250

// ESP-NOW Modes
#define ESP_NOW_MODE_NONE 0
#define ESP_NOW_MODE_FLOW_LEADER_GROUP 1
#define ESP_NOW_MODE_FLOW_LEADER_SINGLE 2
#define ESP_NOW_MODE_FOLLOWER 3

// ESP-NOW Message Structure
typedef struct {
    uint8_t dev_code;
    float base;
    float shoulder;
    float elbow;
    float hand;
    uint8_t cmd;
    char message[210];
} esp_now_message_t;

// ESP-NOW Peer Information
typedef struct {
    uint8_t mac[6];
    char mac_str[18];
    bool active;
    uint32_t last_seen;
} esp_now_peer_t;

// ESP-NOW Control Structure
typedef struct {
    uint8_t mode;
    uint8_t this_dev_mac[6];
    char this_dev_mac_str[18];
    esp_now_peer_t followers[MAX_FOLLOWERS];
    uint8_t follower_count;
    esp_now_message_t current_message;
    bool message_ready;
    uint32_t flow_control_interval_ms;
} esp_now_control_t;

// Function Prototypes
esp_err_t esp_now_controller_init(void);
esp_err_t esp_now_controller_deinit(void);
esp_err_t esp_now_controller_set_mode(uint8_t mode);
esp_err_t esp_now_controller_add_peer(const uint8_t *mac);
esp_err_t esp_now_controller_remove_peer(const uint8_t *mac);
esp_err_t esp_now_controller_send_message(const esp_now_message_t *msg);
esp_err_t esp_now_controller_send_json_command(const char *json_str);
esp_err_t esp_now_controller_get_peers(esp_now_peer_t *peers, uint8_t *count);
esp_err_t esp_now_controller_get_status(esp_now_control_t *status);
esp_err_t esp_now_controller_set_flow_control_interval(uint32_t interval_ms);

// Utility functions
char* esp_now_mac_to_string(const uint8_t *mac);
esp_err_t esp_now_string_to_mac(const char *mac_str, uint8_t *mac);
void esp_now_controller_get_this_mac(uint8_t *mac);

// Internal functions
void esp_now_controller_task(void *pvParameters);
void esp_now_controller_flow_control_task(void *pvParameters);
void esp_now_controller_on_data_sent(const esp_now_send_cb_t *cb, esp_now_send_status_t status);
void esp_now_controller_on_data_recv(const esp_now_recv_cb_t *cb, const uint8_t *data, int data_len);

#endif // ESP_NOW_CONTROLLER_H
