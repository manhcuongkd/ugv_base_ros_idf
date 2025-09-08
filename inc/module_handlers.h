#ifndef MODULE_HANDLERS_H
#define MODULE_HANDLERS_H

#include "ugv_config.h"
#include <stdbool.h>
#include <stdint.h>

// Module type constants
#define MODULE_TYPE_NULL 0
#define MODULE_TYPE_ROARM_M2 1
#define MODULE_TYPE_GIMBAL 2

// Gimbal constants
#define GIMBAL_PAN_ID 2
#define GIMBAL_TILT_ID 1
#define GIMBAL_STEADY_BIAS_MIN -45.0f
#define GIMBAL_STEADY_BIAS_MAX 45.0f

// RoArm-M2 constants
#define ROARM_BASE_SERVO_ID 11
#define ROARM_SHOULDER_DRIVING_SERVO_ID 12
#define ROARM_SHOULDER_DRIVEN_SERVO_ID 13
#define ROARM_ELBOW_SERVO_ID 14
#define ROARM_GRIPPER_SERVO_ID 15

// ESP-NOW modes
#define ESP_NOW_MODE_OFF 0
#define ESP_NOW_MODE_GROUP 1
#define ESP_NOW_MODE_SINGLE 2

#ifdef __cplusplus
extern "C" {
#endif

// External configuration variable
extern ugv_config_t ugv_config;

// ============================================================================
// MODULE TYPE SPECIFIC FUNCTIONS (Matching Arduino Implementation)
// ============================================================================

// Module type specific handling functions
void module_type_roarm_m2(void);
void module_type_gimbal(void);

// RoArm-M2 specific functions
void constant_handle(void);
void roarm_m2_info_feedback(void);
void roarm_m2_get_pos_by_servo_feedback(void);

// Gimbal specific functions
void get_gimbal_feedback(void);
void gimbal_steady(float input_bias_y);
void gimbal_steady_set(bool input_cmd, float input_y);


// Public interface functions
void module_handlers_set_constant_control(bool base_x, bool shoulder_y, bool elbow_z, bool eoat_t);
void module_handlers_set_constant_goals(double base_goal, double shoulder_goal, double elbow_goal, double eoat_goal);
void module_handlers_set_esp_now_mode(uint8_t mode);
void module_handlers_get_arm_positions(double *base, double *shoulder, double *elbow, double *eoat);
void module_handlers_get_gimbal_feedback(void *pan_fb, void *tilt_fb);

#ifdef __cplusplus
}
#endif

#endif // MODULE_HANDLERS_H
