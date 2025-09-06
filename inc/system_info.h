#ifndef SYSTEM_INFO_H
#define SYSTEM_INFO_H

#include <stdint.h>
#include <stdbool.h>
#include <esp_err.h>

#ifdef __cplusplus
extern "C" {
#endif

// ============================================================================
// SYSTEM INFO FUNCTIONS
// ============================================================================

/**
 * @brief Get free flash space information
 * @return ESP_OK on success, error code otherwise
 */
esp_err_t system_info_get_flash_space(void);

/**
 * @brief Get boot mission information
 * @return ESP_OK on success, error code otherwise
 */
esp_err_t system_info_get_boot_mission(void);

/**
 * @brief Reset boot mission
 * @return ESP_OK on success, error code otherwise
 */
esp_err_t system_info_reset_boot_mission(void);

/**
 * @brief Set info print mode
 * @param mode Print mode (0=off, 1=on)
 * @return ESP_OK on success, error code otherwise
 */
esp_err_t system_info_set_print_mode(uint8_t mode);

#ifdef __cplusplus
}
#endif

#endif // SYSTEM_INFO_H
