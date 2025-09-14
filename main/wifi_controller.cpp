#include "../inc/wifi_controller.h"
#include <esp_wifi.h>
#include <esp_event.h>
#include <esp_log.h>
#include <esp_netif.h>
#include <nvs_flash.h>
#include <nvs.h>
#include <freertos/FreeRTOS.h>
#include <freertos/task.h>
#include <freertos/event_groups.h>
#include <string.h>

static const char *TAG = "WiFi_Controller";
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
    wifi_config_t wifi_config = {};
    memset(&wifi_config, 0, sizeof(wifi_config));
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
    wifi_config_t wifi_config = {};
    memset(&wifi_config, 0, sizeof(wifi_config));
    strncpy((char *)wifi_config.ap.ssid, ssid, WIFI_SSID_MAX_LEN - 1);
    wifi_config.ap.ssid_len = strlen(ssid);
    
    if (password != NULL && strlen(password) >= 8) {
        strncpy((char *)wifi_config.ap.password, password, WIFI_PASSWORD_MAX_LEN - 1);
        wifi_config.ap.authmode = WIFI_AUTH_WPA_WPA2_PSK;
    } else {
        wifi_config.ap.authmode = WIFI_AUTH_OPEN;
    }

    wifi_config.ap.channel = channel;
    wifi_config.ap.ssid_hidden = 0;
    wifi_config.ap.max_connection = 4;
    wifi_config.ap.beacon_interval = 100;
    wifi_config.ap.pairwise_cipher = WIFI_CIPHER_TYPE_CCMP;
    wifi_config.ap.ftm_responder = false;

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

esp_err_t wifi_controller_set_ap_config(const char *ssid, const char *password) {
    if (!wifi_initialized) {
        ESP_LOGE(TAG, "WiFi controller not initialized");
        return ESP_ERR_INVALID_STATE;
    }

    ESP_LOGI(TAG, "Setting AP config: SSID=%s", ssid);

    // Configure AP
    wifi_config_t wifi_config = {};
    strncpy((char*)wifi_config.ap.ssid, ssid, sizeof(wifi_config.ap.ssid) - 1);
    wifi_config.ap.ssid_len = strlen(ssid);
    wifi_config.ap.channel = 1;
    wifi_config.ap.password[0] = '\0';
    if (password && strlen(password) > 0) {
        strncpy((char*)wifi_config.ap.password, password, sizeof(wifi_config.ap.password) - 1);
        wifi_config.ap.authmode = WIFI_AUTH_WPA_WPA2_PSK;
    } else {
        wifi_config.ap.authmode = WIFI_AUTH_OPEN;
    }
    wifi_config.ap.max_connection = 4;

    esp_err_t ret = esp_wifi_set_config(WIFI_IF_AP, &wifi_config);
    if (ret != ESP_OK) {
        ESP_LOGE(TAG, "Failed to set AP config: %s", esp_err_to_name(ret));
        return ret;
    }

    return ESP_OK;
}

esp_err_t wifi_controller_set_sta_config(const char *ssid, const char *password) {
    if (!wifi_initialized) {
        ESP_LOGE(TAG, "WiFi controller not initialized");
        return ESP_ERR_INVALID_STATE;
    }

    ESP_LOGI(TAG, "Setting STA config: SSID=%s", ssid);

    // Configure STA
    wifi_config_t wifi_config = {};
    strncpy((char*)wifi_config.sta.ssid, ssid, sizeof(wifi_config.sta.ssid) - 1);
    if (password) {
        strncpy((char*)wifi_config.sta.password, password, sizeof(wifi_config.sta.password) - 1);
    }

    esp_err_t ret = esp_wifi_set_config(WIFI_IF_STA, &wifi_config);
    if (ret != ESP_OK) {
        ESP_LOGE(TAG, "Failed to set STA config: %s", esp_err_to_name(ret));
        return ret;
    }

    return ESP_OK;
}

esp_err_t wifi_controller_set_ap_sta_config(const char *ap_ssid, const char *ap_password,
                                           const char *sta_ssid, const char *sta_password) {
    if (!wifi_initialized) {
        ESP_LOGE(TAG, "WiFi controller not initialized");
        return ESP_ERR_INVALID_STATE;
    }

    ESP_LOGI(TAG, "Setting AP+STA config: AP_SSID=%s, STA_SSID=%s", ap_ssid, sta_ssid);

    // Configure AP
    wifi_config_t ap_config = {};
    strncpy((char*)ap_config.ap.ssid, ap_ssid, sizeof(ap_config.ap.ssid) - 1);
    ap_config.ap.ssid_len = strlen(ap_ssid);
    ap_config.ap.channel = 1;
    if (ap_password && strlen(ap_password) > 0) {
        strncpy((char*)ap_config.ap.password, ap_password, sizeof(ap_config.ap.password) - 1);
        ap_config.ap.authmode = WIFI_AUTH_WPA_WPA2_PSK;
    } else {
        ap_config.ap.authmode = WIFI_AUTH_OPEN;
    }
    ap_config.ap.max_connection = 4;

    // Configure STA
    wifi_config_t sta_config = {};
    strncpy((char*)sta_config.sta.ssid, sta_ssid, sizeof(sta_config.sta.ssid) - 1);
    if (sta_password) {
        strncpy((char*)sta_config.sta.password, sta_password, sizeof(sta_config.sta.password) - 1);
    }

    // Set AP config
    esp_err_t ret = esp_wifi_set_config(WIFI_IF_AP, &ap_config);
    if (ret != ESP_OK) {
        ESP_LOGE(TAG, "Failed to set AP config: %s", esp_err_to_name(ret));
        return ret;
    }

    // Set STA config
    ret = esp_wifi_set_config(WIFI_IF_STA, &sta_config);
    if (ret != ESP_OK) {
        ESP_LOGE(TAG, "Failed to set STA config: %s", esp_err_to_name(ret));
        return ret;
    }

    return ESP_OK;
}

esp_err_t wifi_controller_set_boot_mode(uint8_t mode) {
    if (!wifi_initialized) {
        ESP_LOGE(TAG, "WiFi controller not initialized");
        return ESP_ERR_INVALID_STATE;
    }

    ESP_LOGI(TAG, "Setting WiFi boot mode: %d", mode);

    // Save boot mode to NVS for next boot
    nvs_handle_t nvs_handle;
    esp_err_t ret = nvs_open("wifi_config", NVS_READWRITE, &nvs_handle);
    if (ret != ESP_OK) {
        ESP_LOGE(TAG, "Failed to open NVS: %s", esp_err_to_name(ret));
        return ret;
    }

    ret = nvs_set_u8(nvs_handle, "boot_mode", mode);
    if (ret != ESP_OK) {
        ESP_LOGE(TAG, "Failed to save boot mode: %s", esp_err_to_name(ret));
        nvs_close(nvs_handle);
        return ret;
    }

    nvs_close(nvs_handle);
    ESP_LOGI(TAG, "WiFi boot mode saved: %d", mode);
    return ESP_OK;
}

esp_err_t wifi_controller_get_info(void) {
    if (!wifi_initialized) {
        ESP_LOGE(TAG, "WiFi controller not initialized");
        return ESP_ERR_INVALID_STATE;
    }

    ESP_LOGI(TAG, "Getting WiFi information");

    // Get current WiFi mode
    wifi_mode_t mode;
    esp_wifi_get_mode(&mode);
    ESP_LOGI(TAG, "WiFi mode: %d", mode);

    // Get connection status
    bool connected = wifi_controller_is_connected();
    ESP_LOGI(TAG, "WiFi connected: %s", connected ? "Yes" : "No");

    // Get IP information if connected
    if (connected) {
        esp_netif_ip_info_t ip_info;
        if (wifi_controller_get_connection_info(&ip_info) == ESP_OK) {
            ESP_LOGI(TAG, "IP: " IPSTR, IP2STR(&ip_info.ip));
            ESP_LOGI(TAG, "Netmask: " IPSTR, IP2STR(&ip_info.netmask));
            ESP_LOGI(TAG, "Gateway: " IPSTR, IP2STR(&ip_info.gw));
        }
    }

    // Get MAC address
    uint8_t mac[6];
    if (wifi_controller_get_mac(mac) == ESP_OK) {
        ESP_LOGI(TAG, "MAC: %02x:%02x:%02x:%02x:%02x:%02x",
                 mac[0], mac[1], mac[2], mac[3], mac[4], mac[5]);
    }

    ESP_LOGI(TAG, "WiFi information retrieved");
    return ESP_OK;
}

esp_err_t wifi_controller_create_config_by_status(void) {
    if (!wifi_initialized) {
        ESP_LOGE(TAG, "WiFi controller not initialized");
        return ESP_ERR_INVALID_STATE;
    }

    ESP_LOGI(TAG, "Creating WiFi config by current status");

    // Get current WiFi mode
    wifi_mode_t mode;
    esp_wifi_get_mode(&mode);
    
    // Get current configuration
    wifi_config_t wifi_config;
    esp_err_t ret = ESP_OK;
    
    if (mode == WIFI_MODE_AP || mode == WIFI_MODE_APSTA) {
        ret = esp_wifi_get_config(WIFI_IF_AP, &wifi_config);
        if (ret == ESP_OK) {
            ESP_LOGI(TAG, "AP Config - SSID: %s", wifi_config.ap.ssid);
        }
    }
    
    if (mode == WIFI_MODE_STA || mode == WIFI_MODE_APSTA) {
        ret = esp_wifi_get_config(WIFI_IF_STA, &wifi_config);
        if (ret == ESP_OK) {
            ESP_LOGI(TAG, "STA Config - SSID: %s", wifi_config.sta.ssid);
        }
    }

    ESP_LOGI(TAG, "WiFi config created by status");
    return ESP_OK;
}

esp_err_t wifi_controller_create_config_by_input(uint8_t mode, const char *ap_ssid, const char *ap_password,
                                                const char *sta_ssid, const char *sta_password) {
    if (!wifi_initialized) {
        ESP_LOGE(TAG, "WiFi controller not initialized");
        return ESP_ERR_INVALID_STATE;
    }

    ESP_LOGI(TAG, "Creating WiFi config by input: mode=%d", mode);

    esp_err_t ret = ESP_OK;

    // Configure based on mode
    switch (mode) {
        case 0: // AP mode
            if (ap_ssid) {
                ret = wifi_controller_set_ap_config(ap_ssid, ap_password);
            }
            break;
        case 1: // STA mode
            if (sta_ssid) {
                ret = wifi_controller_set_sta_config(sta_ssid, sta_password);
            }
            break;
        case 2: // AP+STA mode
            if (ap_ssid && sta_ssid) {
                ret = wifi_controller_set_ap_sta_config(ap_ssid, ap_password, sta_ssid, sta_password);
            }
            break;
        case 3: // OFF mode
            ret = wifi_controller_stop();
            break;
        default:
            ESP_LOGE(TAG, "Invalid WiFi mode: %d", mode);
            return ESP_ERR_INVALID_ARG;
    }

    if (ret != ESP_OK) {
        ESP_LOGE(TAG, "Failed to create WiFi config: %s", esp_err_to_name(ret));
        return ret;
    }

    ESP_LOGI(TAG, "WiFi config created by input successfully");
    return ESP_OK;
}

esp_err_t wifi_controller_stop(void) {
    if (!wifi_initialized) {
        return ESP_ERR_INVALID_STATE;
    }
    
    ESP_LOGI(TAG, "Stopping WiFi controller");
    
    // Stop WiFi
    esp_err_t ret = esp_wifi_stop();
    if (ret != ESP_OK) {
        ESP_LOGE(TAG, "Failed to stop WiFi: %s", esp_err_to_name(ret));
        return ret;
    }
    
    // Disconnect from network
    ret = esp_wifi_disconnect();
    if (ret != ESP_OK) {
        ESP_LOGE(TAG, "Failed to disconnect WiFi: %s", esp_err_to_name(ret));
        return ret;
    }
    
    ESP_LOGI(TAG, "WiFi controller stopped successfully");
    return ESP_OK;
}

bool wifi_controller_is_connected(void) {
    if (!wifi_initialized) {
        return false;
    }
    
    // Check WiFi connection status
    wifi_ap_record_t ap_info;
    esp_err_t ret = esp_wifi_sta_get_ap_info(&ap_info);
    
    return (ret == ESP_OK);
}

esp_err_t wifi_controller_get_connection_info(esp_netif_ip_info_t *ip_info) {
    if (!wifi_initialized || !ip_info) {
        return ESP_ERR_INVALID_ARG;
    }
    
    ESP_LOGI(TAG, "Getting WiFi connection info");
    
    // Get IP information
    esp_netif_t *netif = esp_netif_get_handle_from_ifkey("WIFI_STA_DEF");
    if (!netif) {
        ESP_LOGE(TAG, "Failed to get network interface");
        return ESP_FAIL;
    }
    
    esp_err_t ret = esp_netif_get_ip_info(netif, ip_info);
    if (ret != ESP_OK) {
        ESP_LOGE(TAG, "Failed to get IP info: %s", esp_err_to_name(ret));
        return ret;
    }
    
    ESP_LOGI(TAG, "Connection info retrieved: IP=%d.%d.%d.%d, Netmask=%d.%d.%d.%d, Gateway=%d.%d.%d.%d",
             IP2STR(&ip_info->ip), IP2STR(&ip_info->netmask), IP2STR(&ip_info->gw));
    
    return ESP_OK;
}

esp_err_t wifi_controller_get_mac(uint8_t *mac) {
    if (!wifi_initialized || !mac) {
        return ESP_ERR_INVALID_ARG;
    }
    
    ESP_LOGI(TAG, "Getting WiFi MAC address");
    
    // Get WiFi MAC address
    esp_err_t ret = esp_wifi_get_mac(WIFI_IF_STA, mac);
    if (ret != ESP_OK) {
        ESP_LOGE(TAG, "Failed to get MAC address: %s", esp_err_to_name(ret));
        return ret;
    }
    
    ESP_LOGI(TAG, "MAC address retrieved: %02x:%02x:%02x:%02x:%02x:%02x",
             mac[0], mac[1], mac[2], mac[3], mac[4], mac[5]);
    
    return ESP_OK;
}
