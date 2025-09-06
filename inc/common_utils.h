#ifndef COMMON_UTILS_H
#define COMMON_UTILS_H

#include <esp_err.h>
#include <esp_log.h>

// Common validation macros to eliminate redundancy
#define VALIDATE_INITIALIZED(initialized_flag, tag) \
    do { \
        if (!(initialized_flag)) { \
            ESP_LOGE(tag, #initialized_flag " not initialized"); \
            return ESP_ERR_INVALID_STATE; \
        } \
    } while(0)

#define VALIDATE_NULL_PARAM(param, tag) \
    do { \
        if ((param) == NULL) { \
            ESP_LOGE(tag, "Parameter " #param " is NULL"); \
            return ESP_ERR_INVALID_ARG; \
        } \
    } while(0)

#define VALIDATE_NULL_PARAMS(param1, param2, tag) \
    do { \
        if ((param1) == NULL || (param2) == NULL) { \
            ESP_LOGE(tag, "Parameters " #param1 " or " #param2 " are NULL"); \
            return ESP_ERR_INVALID_ARG; \
        } \
    } while(0)

#define VALIDATE_NULL_PARAMS3(param1, param2, param3, tag) \
    do { \
        if ((param1) == NULL || (param2) == NULL || (param3) == NULL) { \
            ESP_LOGE(tag, "Parameters " #param1 ", " #param2 ", or " #param3 " are NULL"); \
            return ESP_ERR_INVALID_ARG; \
        } \
    } while(0)

// Common file operation validation
#define VALIDATE_FILE_OPERATION(initialized_flag, filename, tag) \
    do { \
        VALIDATE_INITIALIZED(initialized_flag, tag); \
        VALIDATE_NULL_PARAM(filename, tag); \
    } while(0)

// Common JSON operation validation  
#define VALIDATE_JSON_OPERATION(initialized_flag, json_param, tag) \
    do { \
        VALIDATE_INITIALIZED(initialized_flag, tag); \
        VALIDATE_NULL_PARAM(json_param, tag); \
    } while(0)

#endif // COMMON_UTILS_H
