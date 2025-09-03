#ifndef UGV_ADVANCED_H
#define UGV_ADVANCED_H

#include <stdbool.h>
#include <stdint.h>
#include "ugv_config.h"

#ifdef __cplusplus
extern "C" {
#endif

// External configuration variable
extern ugv_config_t ugv_config;

// ============================================================================
// ADVANCED UGV FUNCTIONS (Matching Arduino Implementation)
// ============================================================================

// Configuration functions
void config_ee_mode_type(uint8_t mode);
void config_eoat(uint8_t mount_pos, double ea, double eb);
void config_info_print(uint8_t cmd);
void set_base_info_feedback_mode(bool enable);
void set_feedback_flow_interval(int cmd);
void set_cmd_echo(bool enable);

// Save functions
void save_speed_rate(void);
void save_main_type_module_type(uint8_t main_type, uint8_t module_type);

// System functions
void base_info_feedback(void);
void heart_beat_ctrl(void);

#ifdef __cplusplus
}
#endif

#endif // UGV_ADVANCED_H
