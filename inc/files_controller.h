#ifndef FILES_CONTROLLER_H
#define FILES_CONTROLLER_H

#include <esp_err.h>
#include <esp_spiffs.h>
#include <freertos/FreeRTOS.h>
#include <freertos/task.h>
#include <cJSON.h>

// File System Configuration
#define MAX_FILE_NAME_LEN               64
#define MAX_FILE_CONTENT_LEN            2048
#define MAX_MISSION_NAME_LEN            32
#define MAX_MISSION_CMD_LEN             256
#define MAX_MISSIONS                    20
#define SPIFFS_MAX_PATH_LEN             64
#define SPIFFS_MAX_FILE_SIZE            4096

// Mission Structure
typedef struct {
    char name[MAX_MISSION_NAME_LEN];
    char commands[MAX_MISSION_CMD_LEN];
    bool active;
    uint32_t created_time;
} mission_t;

// File System Status
typedef struct {
    bool mounted;
    size_t total_bytes;
    size_t used_bytes;
    size_t free_bytes;
    uint32_t file_count;
} files_status_t;

// Function Prototypes
esp_err_t files_controller_init(void);
esp_err_t files_controller_deinit(void);
esp_err_t files_controller_mount(void);
esp_err_t files_controller_unmount(void);

// File Operations
esp_err_t files_controller_create_file(const char *filename, const char *content);
esp_err_t files_controller_read_file(const char *filename, char *content, size_t max_len);
esp_err_t files_controller_write_file(const char *filename, const char *content);
esp_err_t files_controller_delete_file(const char *filename);
esp_err_t files_controller_file_exists(const char *filename);
esp_err_t files_controller_list_files(char *file_list, size_t max_len);

// Line-based File Operations
esp_err_t files_controller_append_line(const char *filename, const char *content);
esp_err_t files_controller_insert_line(const char *filename, int line_num, const char *content);
esp_err_t files_controller_replace_line(const char *filename, int line_num, const char *content);
esp_err_t files_controller_read_line(const char *filename, int line_num, char *content, size_t max_len);
esp_err_t files_controller_delete_line(const char *filename, int line_num);

// Mission System
esp_err_t files_controller_create_mission(const char *name, const char *commands);
esp_err_t files_controller_delete_mission(const char *name);
esp_err_t files_controller_play_mission(const char *name, int repeat_count);
esp_err_t files_controller_list_missions(mission_t *missions, uint8_t *count);
esp_err_t files_controller_get_mission(const char *name, mission_t *mission);

// Status and Information
esp_err_t files_controller_get_status(files_status_t *status);
esp_err_t files_controller_get_free_space(uint32_t *free_bytes);
esp_err_t files_controller_scan_contents(void);

// Configuration Files
esp_err_t files_controller_save_config(const char *config_name, const cJSON *config);
esp_err_t files_controller_load_config(const char *config_name, cJSON **config);
esp_err_t files_controller_delete_config(const char *config_name);

// Utility Functions
esp_err_t files_controller_format(void);
esp_err_t files_controller_check_integrity(void);
const char* files_controller_get_error_string(esp_err_t error);

#endif // FILES_CONTROLLER_H
