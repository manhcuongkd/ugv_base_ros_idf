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
#define ESP_NOW_CHANNEL                 0
#define ESP_NOW_ENCRYPT                 false
#define MAX_FOLLOWERS                   10
#define MAX_MESSAGE_SIZE                250
#define ESP_NOW_MAX_PEERS               10
#define ESP_NOW_QUEUE_SIZE              10
#define ESP_NOW_TASK_STACK_SIZE         4096
#define ESP_NOW_FLOW_CONTROL_INTERVAL   1000    // ms

// ESP-NOW Modes
#define ESP_NOW_MODE_NONE 0
#define ESP_NOW_MODE_FLOW_LEADER_GROUP 1
#define ESP_NOW_MODE_FLOW_LEADER_SINGLE 2
#define ESP_NOW_MODE_FOLLOWER 3

// Arduino-compatible command types (check for conflicts with json_parser.h)
#ifndef CMD_ESP_NOW_SEND
#define CMD_ESP_NOW_SEND                1001
#endif
#ifndef CMD_ESP_NOW_RECV
#define CMD_ESP_NOW_RECV                1002
#endif

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
    bool ctrl_by_broadcast;                    // Arduino compatibility
    uint8_t mac_whitelist_broadcast[6];        // Arduino compatibility
    uint8_t single_follower_dev[6];            // Arduino compatibility
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

// Arduino-compatible API functions
esp_err_t esp_now_controller_change_mode(uint8_t input_mode);
esp_err_t esp_now_controller_register_new_follower(const char *input_mac);
esp_err_t esp_now_controller_delete_follower(const char *input_mac);
esp_err_t esp_now_controller_group_send(uint8_t dev_code, float b, float s, float e, float h, uint8_t cmd, const char *message);
esp_err_t esp_now_controller_single_dev_send(const char *input_mac, uint8_t dev_code, float b, float s, float e, float h, uint8_t cmd, const char *message);
esp_err_t esp_now_controller_single_dev_flow_ctrl(void);
esp_err_t esp_now_controller_group_devs_flow_ctrl(void);
esp_err_t esp_now_controller_change_broadcast_mode(bool input_mode, const char *input_mac);
void esp_now_controller_get_this_dev_mac_address(char *mac_str);

// JSON feedback functions (Arduino compatibility)
void esp_now_controller_send_json_feedback(int type, int status, const char *message, const char *mac);
void esp_now_controller_send_info_feedback(const char *info);

// Utility functions
char* esp_now_mac_to_string(const uint8_t *mac);
esp_err_t esp_now_string_to_mac(const char *mac_str, uint8_t *mac);
esp_err_t esp_now_controller_get_this_mac(uint8_t *mac);

// Internal functions
void esp_now_controller_task(void *pvParameters);
void esp_now_controller_flow_control_task(void *pvParameters);
void esp_now_controller_on_data_sent(const esp_now_send_cb_t *cb, esp_now_send_status_t status);
void esp_now_controller_on_data_recv(const esp_now_recv_cb_t *cb, const uint8_t *data, int data_len);

/**
 * @brief Set broadcast mode and follower MAC
 * @param mode Broadcast mode
 * @param mac Follower MAC address
 * @return ESP_OK on success, error code otherwise
 */
esp_err_t esp_now_controller_set_broadcast_mode(uint8_t mode, const char *mac);

/**
 * @brief Get this device's MAC address
 * @param mac_str Buffer for MAC address string
 * @return ESP_OK on success, error code otherwise
 */
esp_err_t esp_now_controller_get_mac_address(char *mac_str);

/**
 * @brief Send ESP-NOW receive feedback
 * @param mac MAC address of sender
 * @param message Received message
 * @return ESP_OK on success, error code otherwise
 */
esp_err_t esp_now_controller_send_recv_feedback(const char *mac, const char *message);

/**
 * @brief Send ESP-NOW send status feedback
 * @param mac MAC address of recipient
 * @param status Send status code
 * @param message Status message
 * @return ESP_OK on success, error code otherwise
 */
esp_err_t esp_now_controller_send_send_feedback(const char *mac, int status, const char *message);

#endif // ESP_NOW_CONTROLLER_H
