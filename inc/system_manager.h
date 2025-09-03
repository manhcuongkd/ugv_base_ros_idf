#ifndef SYSTEM_MANAGER_H
#define SYSTEM_MANAGER_H

#include <esp_err.h>
#include <stdbool.h>
#include "ugv_config.h"

#ifdef __cplusplus
extern "C" {
#endif

// External configuration variable
extern ugv_config_t ugv_config;

// ============================================================================
// SYSTEM MANAGEMENT FUNCTIONS
// ============================================================================

// System initialization
esp_err_t system_manager_init(void);
esp_err_t system_manager_init_hardware(void);
esp_err_t system_manager_init_communication(void);

// System state queries
bool system_manager_is_initialized(void);
bool system_manager_is_emergency_stop(void);

// System control
void system_manager_set_emergency_stop(bool stop);

#ifdef __cplusplus
}
#endif

#endif // SYSTEM_MANAGER_H
