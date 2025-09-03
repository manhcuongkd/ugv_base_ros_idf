#include "../inc/wifi_controller.h"
#include <esp_wifi.h>
#include <esp_event.h>
#include <esp_log.h>
#include <esp_netif.h>
#include <freertos/FreeRTOS.h>
#include <freertos/task.h>
#include <freertos/event_groups.h>
#include <string.h>

static const char *TAG = "WiFi_Controller";

// WiFi configuration
#define WIFI_SSID_MAX_LEN 32
#define WIFI_PASSWORD_MAX_LEN 64
#define WIFI_MAXIMUM_RETRY 5
#define WIFI_CONNECTED_BIT BIT0
#define WIFI_FAIL_BIT BIT1

// Global variables
static EventGroupHandle_t wifi_event_group;
static esp_netif_t *sta_netif = NULL;
static esp_netif_t *ap_netif = NULL;
static wifi_mode_t current_mode = WIFI_MODE_NULL;
static bool wifi_initialized = false;

// Private function prototypes
static void wifi_event_handler(void *arg, esp_event_base_t event_base, int32_t event_id, void *event_data);

esp_err_t wifi_controller_init(void)
{
    if (wifi_initialized) {
        ESP_LOGW(TAG, "WiFi controller already initialized");
        return ESP_OK;
    }

    ESP_LOGI(TAG, "Initializing WiFi controller...");

    // Initialize TCP/IP stack
    ESP_ERROR_CHECK(esp_netif_init());
    
    // Create default event loop
    ESP_ERROR_CHECK(esp_event_loop_create_default());

    // Create WiFi event group
    wifi_event_group = xEventGroupCreate();
    if (wifi_event_group == NULL) {
        ESP_LOGE(TAG, "Failed to create WiFi event group");
        return ESP_ERR_NO_MEM;
    }

    // Create network interfaces
    sta_netif = esp_netif_create_default_wifi_sta();
    ap_netif = esp_netif_create_default_wifi_ap();

    // Initialize WiFi
    wifi_init_config_t cfg = WIFI_INIT_CONFIG_DEFAULT();
    ESP_ERROR_CHECK(esp_wifi_init(&cfg));

    // Register event handlers
    ESP_ERROR_CHECK(esp_event_handler_instance_register(WIFI_EVENT, ESP_EVENT_ANY_ID, &wifi_event_handler, NULL, NULL));
    ESP_ERROR_CHECK(esp_event_handler_instance_register(IP_EVENT, IP_EVENT_STA_GOT_IP, &wifi_event_handler, NULL, NULL));

    // Set WiFi mode to null initially
    ESP_ERROR_CHECK(esp_wifi_set_mode(WIFI_MODE_NULL));
    ESP_ERROR_CHECK(esp_wifi_start());

    wifi_initialized = true;
    ESP_LOGI(TAG, "WiFi controller initialized successfully");
    return ESP_OK;
}

esp_err_t wifi_controller_deinit(void)
{
    if (!wifi_initialized) {
        return ESP_OK;
    }

    ESP_LOGI(TAG, "Deinitializing WiFi controller...");

    // Stop WiFi
    esp_wifi_stop();
    esp_wifi_deinit();

    // Unregister event handlers
    esp_event_handler_instance_unregister(WIFI_EVENT, ESP_EVENT_ANY_ID, NULL);
    esp_event_handler_instance_unregister(IP_EVENT, IP_EVENT_STA_GOT_IP, NULL);

    // Delete event group
    if (wifi_event_group) {
        vEventGroupDelete(wifi_event_group);
        wifi_event_group = NULL;
    }

    // Destroy network interfaces
    if (sta_netif) {
        esp_netif_destroy_default_wifi(sta_netif);
        sta_netif = NULL;
    }
    if (ap_netif) {
        esp_netif_destroy_default_wifi(ap_netif);
        ap_netif = NULL;
    }

    wifi_initialized = false;
    ESP_LOGI(TAG, "WiFi controller deinitialized");
    return ESP_OK;
}

esp_err_t wifi_controller_connect_sta(const char *ssid, const char *password)
{
    if (!wifi_initialized) {
        ESP_LOGE(TAG, "WiFi controller not initialized");
        return ESP_ERR_INVALID_STATE;
    }

    if (ssid == NULL || strlen(ssid) == 0) {
        ESP_LOGE(TAG, "Invalid SSID");
        return ESP_ERR_INVALID_ARG;
    }

    ESP_LOGI(TAG, "Connecting to WiFi SSID: %s", ssid);

    // Configure station mode
    wifi_config_t wifi_config = {0};
    strncpy((char *)wifi_config.sta.ssid, ssid, WIFI_SSID_MAX_LEN - 1);
    
    if (password != NULL && strlen(password) > 0) {
        strncpy((char *)wifi_config.sta.password, password, WIFI_PASSWORD_MAX_LEN - 1);
    }

    // Set WiFi mode to station
    ESP_ERROR_CHECK(esp_wifi_set_mode(WIFI_MODE_STA));
    ESP_ERROR_CHECK(esp_wifi_set_config(WIFI_IF_STA, &wifi_config));
    
    current_mode = WIFI_MODE_STA;
    ESP_LOGI(TAG, "WiFi station mode configured");
    return ESP_OK;
}

esp_err_t wifi_controller_start_ap(const char *ssid, const char *password, uint8_t channel)
{
    if (!wifi_initialized) {
        ESP_LOGE(TAG, "WiFi controller not initialized");
        return ESP_ERR_INVALID_STATE;
    }

    if (ssid == NULL || strlen(ssid) == 0) {
        ESP_LOGE(TAG, "Invalid SSID");
        return ESP_ERR_INVALID_ARG;
    }

    ESP_LOGI(TAG, "Starting WiFi AP with SSID: %s", ssid);

    // Configure AP mode
    wifi_config_t wifi_config = {0};
    strncpy((char *)wifi_config.ap.ssid, ssid, WIFI_SSID_MAX_LEN - 1);
    
    if (password != NULL && strlen(password) >= 8) {
        strncpy((char *)wifi_config.ap.password, password, WIFI_PASSWORD_MAX_LEN - 1);
        wifi_config.ap.authmode = WIFI_AUTH_WPA_WPA2_PSK;
    } else {
        wifi_config.ap.authmode = WIFI_AUTH_OPEN;
    }

    wifi_config.ap.channel = channel;
    wifi_config.ap.max_connection = 4;
    wifi_config.ap.beacon_interval = 100;

    // Set WiFi mode to AP
    ESP_ERROR_CHECK(esp_wifi_set_mode(WIFI_MODE_AP));
    ESP_ERROR_CHECK(esp_wifi_set_config(WIFI_IF_AP, &wifi_config));
    
    current_mode = WIFI_MODE_AP;
    ESP_LOGI(TAG, "WiFi AP mode configured");
    return ESP_OK;
}

esp_err_t wifi_controller_disconnect(void)
{
    if (!wifi_initialized) {
        return ESP_OK;
    }

    ESP_LOGI(TAG, "Disconnecting WiFi...");

    // Clear event bits
    xEventGroupClearBits(wifi_event_group, WIFI_CONNECTED_BIT | WIFI_FAIL_BIT);

    // Disconnect from AP
    esp_wifi_disconnect();

    // Set mode to null
    ESP_ERROR_CHECK(esp_wifi_set_mode(WIFI_MODE_NULL));
    current_mode = WIFI_MODE_NULL;

    ESP_LOGI(TAG, "WiFi disconnected");
    return ESP_OK;
}

// Private functions
static void wifi_event_handler(void *arg, esp_event_base_t event_base, int32_t event_id, void *event_data)
{
    if (event_base == WIFI_EVENT && event_id == WIFI_EVENT_STA_START) {
        ESP_LOGI(TAG, "WiFi station started");
        esp_wifi_connect();
    } else if (event_base == WIFI_EVENT && event_id == WIFI_EVENT_STA_DISCONNECTED) {
        ESP_LOGI(TAG, "WiFi station disconnected");
        xEventGroupSetBits(wifi_event_group, WIFI_FAIL_BIT);
    } else if (event_base == IP_EVENT && event_id == IP_EVENT_STA_GOT_IP) {
        ip_event_got_ip_t *event = (ip_event_got_ip_t *)event_data;
        ESP_LOGI(TAG, "Got IP: " IPSTR, IP2STR(&event->ip_info.ip));
        xEventGroupSetBits(wifi_event_group, WIFI_CONNECTED_BIT);
        xEventGroupClearBits(wifi_event_group, WIFI_FAIL_BIT);
    } else if (event_base == WIFI_EVENT && event_id == WIFI_EVENT_AP_START) {
        ESP_LOGI(TAG, "WiFi AP started");
    } else if (event_base == WIFI_EVENT && event_id == WIFI_EVENT_AP_STOP) {
        ESP_LOGI(TAG, "WiFi AP stopped");
    }
}
