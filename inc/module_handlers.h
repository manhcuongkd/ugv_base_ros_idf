#ifndef MODULE_HANDLERS_H
#define MODULE_HANDLERS_H

#include "ugv_config.h"

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

#ifdef __cplusplus
}
#endif

#endif // MODULE_HANDLERS_H
