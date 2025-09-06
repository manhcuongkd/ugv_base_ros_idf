#include "../inc/system_info.h"
#include <esp_log.h>
#include <esp_system.h>
#include <esp_spiffs.h>
#include <nvs_flash.h>
#include <nvs.h>
#include <string.h>

static const char *TAG = "System_Info";

esp_err_t system_info_get_flash_space(void) {
    ESP_LOGI(TAG, "Getting flash space information");
    
    // Get flash size
    uint32_t flash_size;
    esp_err_t ret = esp_flash_get_size(NULL, &flash_size);
    if (ret != ESP_OK) {
        ESP_LOGE(TAG, "Failed to get flash size: %s", esp_err_to_name(ret));
        return ret;
    }
    ESP_LOGI(TAG, "Total flash size: %zu bytes (%.2f MB)", flash_size, flash_size / (1024.0 * 1024.0));
    
    // Get free heap
    size_t free_heap = esp_get_free_heap_size();
    ESP_LOGI(TAG, "Free heap: %zu bytes (%.2f KB)", free_heap, free_heap / 1024.0);
    
    // Get minimum free heap
    size_t min_free_heap = esp_get_minimum_free_heap_size();
    ESP_LOGI(TAG, "Minimum free heap: %zu bytes (%.2f KB)", min_free_heap, min_free_heap / 1024.0);
    
    // Get SPIFFS partition info
    const esp_partition_t *partition = esp_partition_find_first(ESP_PARTITION_TYPE_DATA, ESP_PARTITION_SUBTYPE_DATA_SPIFFS, "storage");
    if (partition) {
        ESP_LOGI(TAG, "SPIFFS partition size: %zu bytes (%.2f KB)", partition->size, partition->size / 1024.0);
        
        // Get SPIFFS usage
        size_t total = 0, used = 0;
        esp_err_t ret = esp_spiffs_info("storage", &total, &used);
        if (ret == ESP_OK) {
            ESP_LOGI(TAG, "SPIFFS usage: %zu/%zu bytes (%.1f%% used)", used, total, (used * 100.0) / total);
        }
    }
    
    ESP_LOGI(TAG, "Flash space information retrieved");
    return ESP_OK;
}

esp_err_t system_info_get_boot_mission(void) {
    ESP_LOGI(TAG, "Getting boot mission information");
    
    // Read boot mission from NVS
    nvs_handle_t nvs_handle;
    esp_err_t ret = nvs_open("system_config", NVS_READONLY, &nvs_handle);
    if (ret != ESP_OK) {
        ESP_LOGI(TAG, "No boot mission configured");
        return ESP_OK;
    }
    
    char boot_mission[64];
    size_t required_size = sizeof(boot_mission);
    ret = nvs_get_str(nvs_handle, "boot_mission", boot_mission, &required_size);
    if (ret == ESP_OK) {
        ESP_LOGI(TAG, "Boot mission: %s", boot_mission);
    } else {
        ESP_LOGI(TAG, "No boot mission configured");
    }
    
    nvs_close(nvs_handle);
    return ESP_OK;
}

esp_err_t system_info_reset_boot_mission(void) {
    ESP_LOGI(TAG, "Resetting boot mission");
    
    // Clear boot mission from NVS
    nvs_handle_t nvs_handle;
    esp_err_t ret = nvs_open("system_config", NVS_READWRITE, &nvs_handle);
    if (ret != ESP_OK) {
        ESP_LOGE(TAG, "Failed to open NVS: %s", esp_err_to_name(ret));
        return ret;
    }
    
    ret = nvs_erase_key(nvs_handle, "boot_mission");
    if (ret != ESP_OK && ret != ESP_ERR_NVS_NOT_FOUND) {
        ESP_LOGE(TAG, "Failed to erase boot mission: %s", esp_err_to_name(ret));
        nvs_close(nvs_handle);
        return ret;
    }
    
    nvs_close(nvs_handle);
    ESP_LOGI(TAG, "Boot mission reset successfully");
    return ESP_OK;
}

esp_err_t system_info_set_print_mode(uint8_t mode) {
    ESP_LOGI(TAG, "Setting info print mode: %d", mode);
    
    // Save print mode to NVS
    nvs_handle_t nvs_handle;
    esp_err_t ret = nvs_open("system_config", NVS_READWRITE, &nvs_handle);
    if (ret != ESP_OK) {
        ESP_LOGE(TAG, "Failed to open NVS: %s", esp_err_to_name(ret));
        return ret;
    }
    
    ret = nvs_set_u8(nvs_handle, "info_print_mode", mode);
    if (ret != ESP_OK) {
        ESP_LOGE(TAG, "Failed to save print mode: %s", esp_err_to_name(ret));
        nvs_close(nvs_handle);
        return ret;
    }
    
    nvs_close(nvs_handle);
    ESP_LOGI(TAG, "Info print mode set: %s", mode ? "ON" : "OFF");
    return ESP_OK;
}
