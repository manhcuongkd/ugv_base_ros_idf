#ifndef WIFI_CONTROLLER_H
#define WIFI_CONTROLLER_H

#include <stdint.h>
#include <stdbool.h>
#include <esp_err.h>
#include <esp_wifi.h>
#include <esp_event.h>
#include "ugv_config.h"

#ifdef __cplusplus
extern "C" {
#endif

// WiFi configuration - using ESP-IDF's built-in types
typedef struct {
    wifi_sta_config_t sta_config;  // Station configuration
    wifi_ap_config_t ap_config;    // Access point configuration
    uint8_t sta_max_retry;         // Maximum connection retries
    uint32_t retry_delay_ms;       // Delay between retries
    bool enable_ap;                 // Enable access point mode
    bool enable_sta;                // Enable station mode
} wifi_controller_config_t;

// WiFi connection status - using ESP-IDF's built-in types
typedef wifi_ap_record_t wifi_scan_result_t;  // Reuse ESP-IDF's scan result type

// WiFi connection info - using ESP-IDF's built-in types
// Use esp_netif_ip_info_t for IP information and wifi_ap_record_t for connection details

// WiFi scan result - using ESP-IDF's built-in types
// wifi_scan_result_t is already defined above as wifi_ap_record_t

// Function prototypes

/**
 * @brief Initialize WiFi controller
 * @return ESP_OK on success, error code otherwise
 */
esp_err_t wifi_controller_init(void);

/**
 * @brief Deinitialize WiFi controller
 * @return ESP_OK on success, error code otherwise
 */
esp_err_t wifi_controller_deinit(void);

/**
 * @brief Configure WiFi settings
 * @param config Pointer to WiFi configuration
 * @return ESP_OK on success, error code otherwise
 */
esp_err_t wifi_controller_configure(wifi_controller_config_t *config);

/**
 * @brief Start WiFi station mode
 * @return ESP_OK on success, error code otherwise
 */
esp_err_t wifi_controller_start_sta(void);

/**
 * @brief Start WiFi access point mode
 * @return ESP_OK on success, error code otherwise
 */
esp_err_t wifi_controller_start_ap(void);

/**
 * @brief Start WiFi in both modes (AP+STA)
 * @return ESP_OK on success, error code otherwise
 */
esp_err_t wifi_controller_start_ap_sta(void);

/**
 * @brief Stop WiFi
 * @return ESP_OK on success, error code otherwise
 */
esp_err_t wifi_controller_stop(void);

/**
 * @brief Connect to WiFi network
 * @param ssid WiFi SSID
 * @param password WiFi password
 * @return ESP_OK on success, error code otherwise
 */
esp_err_t wifi_controller_connect(const char *ssid, const char *password);

/**
 * @brief Disconnect from WiFi network
 * @return ESP_OK on success, error code otherwise
 */
esp_err_t wifi_controller_disconnect(void);

/**
 * @brief Get WiFi connection status
 * @return WiFi status (using ESP-IDF's built-in status)
 */
bool wifi_controller_is_connected(void);

/**
 * @brief Get WiFi connection information
 * @param info Pointer to connection info structure
 * @return ESP_OK on success, error code otherwise
 */
esp_err_t wifi_controller_get_connection_info(esp_netif_ip_info_t *ip_info);

/**
 * @brief Scan for available WiFi networks
 * @param results Buffer for scan results
 * @param max_results Maximum number of results
 * @param result_count Pointer to actual result count
 * @return ESP_OK on success, error code otherwise
 */
esp_err_t wifi_controller_scan(wifi_scan_result_t *results, uint8_t max_results, uint8_t *result_count);

/**
 * @brief Get WiFi MAC address
 * @param mac Buffer for MAC address
 * @return ESP_OK on success, error code otherwise
 */
esp_err_t wifi_controller_get_mac(uint8_t *mac);

/**
 * @brief Set WiFi MAC address
 * @param mac MAC address
 * @return ESP_OK on success, error code otherwise
 */
esp_err_t wifi_controller_set_mac(const uint8_t *mac);

/**
 * @brief Get WiFi channel
 * @param channel Pointer to channel number
 * @return ESP_OK on success, error code otherwise
 */
esp_err_t wifi_controller_get_channel(uint8_t *channel);

/**
 * @brief Set WiFi channel
 * @param channel Channel number
 * @return ESP_OK on success, error code otherwise
 */
esp_err_t wifi_controller_set_channel(uint8_t channel);

/**
 * @brief Get WiFi transmit power
 * @param power Pointer to transmit power
 * @return ESP_OK on success, error code otherwise
 */
esp_err_t wifi_controller_get_tx_power(int8_t *power);

/**
 * @brief Set WiFi transmit power
 * @param power Transmit power
 * @return ESP_OK on success, error code otherwise
 */
esp_err_t wifi_controller_set_tx_power(int8_t power);

/**
 * @brief Enable/disable WiFi power save
 * @param enable True to enable, false to disable
 * @return ESP_OK on success, error code otherwise
 */
esp_err_t wifi_controller_set_power_save(bool enable);

/**
 * @brief Get WiFi power save status
 * @param enabled Pointer to power save status
 * @return ESP_OK on success, error code otherwise
 */
esp_err_t wifi_controller_get_power_save(bool *enabled);

/**
 * @brief Set WiFi country code
 * @param country_code Country code (e.g., "US", "EU")
 * @return ESP_OK on success, error code otherwise
 */
esp_err_t wifi_controller_set_country(const char *country_code);

/**
 * @brief Get WiFi country code
 * @param country_code Buffer for country code
 * @return ESP_OK on success, error code otherwise
 */
esp_err_t wifi_controller_get_country(char *country_code);

/**
 * @brief Set WiFi protocol mode
 * @param protocol Protocol mode (802.11b/g/n)
 * @return ESP_OK on success, error code otherwise
 */
esp_err_t wifi_controller_set_protocol(uint8_t protocol);

/**
 * @brief Get WiFi protocol mode
 * @param protocol Pointer to protocol mode
 * @return ESP_OK on success, error code otherwise
 */
esp_err_t wifi_controller_get_protocol(uint8_t *protocol);

/**
 * @brief Set WiFi AP configuration
 * @param ssid AP SSID
 * @param password AP password
 * @return ESP_OK on success, error code otherwise
 */
esp_err_t wifi_controller_set_ap_config(const char *ssid, const char *password);

/**
 * @brief Set WiFi STA configuration
 * @param ssid STA SSID
 * @param password STA password
 * @return ESP_OK on success, error code otherwise
 */
esp_err_t wifi_controller_set_sta_config(const char *ssid, const char *password);

/**
 * @brief Set WiFi AP+STA configuration
 * @param ap_ssid AP SSID
 * @param ap_password AP password
 * @param sta_ssid STA SSID
 * @param sta_password STA password
 * @return ESP_OK on success, error code otherwise
 */
esp_err_t wifi_controller_set_ap_sta_config(const char *ap_ssid, const char *ap_password,
                                           const char *sta_ssid, const char *sta_password);

/**
 * @brief Set WiFi boot mode
 * @param mode Boot mode (0=AP, 1=STA, 2=AP+STA, 3=OFF)
 * @return ESP_OK on success, error code otherwise
 */
esp_err_t wifi_controller_set_boot_mode(uint8_t mode);

/**
 * @brief Get WiFi information
 * @return ESP_OK on success, error code otherwise
 */
esp_err_t wifi_controller_get_info(void);

/**
 * @brief Create WiFi config by current status
 * @return ESP_OK on success, error code otherwise
 */
esp_err_t wifi_controller_create_config_by_status(void);

/**
 * @brief Create WiFi config by input parameters
 * @param mode WiFi mode
 * @param ap_ssid AP SSID
 * @param ap_password AP password
 * @param sta_ssid STA SSID
 * @param sta_password STA password
 * @return ESP_OK on success, error code otherwise
 */
esp_err_t wifi_controller_create_config_by_input(uint8_t mode, const char *ap_ssid, const char *ap_password,
                                                const char *sta_ssid, const char *sta_password);

/**
 * @brief Set WiFi bandwidth
 * @param bandwidth Bandwidth (20/40 MHz)
 * @return ESP_OK on success, error code otherwise
 */
esp_err_t wifi_controller_set_bandwidth(wifi_bandwidth_t bandwidth);

/**
 * @brief Get WiFi bandwidth
 * @param bandwidth Pointer to bandwidth
 * @return ESP_OK on success, error code otherwise
 */
esp_err_t wifi_controller_get_bandwidth(wifi_bandwidth_t *bandwidth);

/**
 * @brief Register WiFi event handler
 * @param handler Event handler function
 * @param arg Handler argument
 * @return ESP_OK on success, error code otherwise
 */
esp_err_t wifi_controller_register_event_handler(esp_event_handler_t handler, void *arg);

/**
 * @brief Unregister WiFi event handler
 * @param handler Event handler function
 * @return ESP_OK on success, error code otherwise
 */
esp_err_t wifi_controller_unregister_event_handler(esp_event_handler_t handler);

/**
 * @brief Get WiFi statistics
 * @param stats Pointer to WiFi statistics
 * @return ESP_OK on success, error code otherwise
 */
esp_err_t wifi_controller_get_stats(void *stats);

/**
 * @brief Reset WiFi statistics
 * @return ESP_OK on success, error code otherwise
 */
esp_err_t wifi_controller_reset_stats(void);

/**
 * @brief Check if WiFi is connected
 * @return True if connected, false otherwise
 */
bool wifi_controller_is_connected(void);

/**
 * @brief Check if AP mode is active
 * @return True if AP active, false otherwise
 */
bool wifi_controller_is_ap_active(void);

/**
 * @brief Get default WiFi configuration
 * @param config Pointer to WiFi configuration
 * @return ESP_OK on success, error code otherwise
 */
esp_err_t wifi_controller_get_default_config(wifi_controller_config_t *config);

// Default configuration values
#define DEFAULT_WIFI_MAX_RETRY        5
#define DEFAULT_WIFI_RETRY_DELAY_MS   1000
#define DEFAULT_WIFI_AP_MAX_CONN      4
#define DEFAULT_WIFI_AP_CHANNEL       1
#define DEFAULT_WIFI_AP_HIDDEN        false

// WiFi event types - using ESP-IDF's built-in types directly
// No need to redefine, use WIFI_EVENT_STA_CONNECTED, WIFI_EVENT_AP_START, etc. directly

#ifdef __cplusplus
}
#endif

#endif // WIFI_CONTROLLER_H
