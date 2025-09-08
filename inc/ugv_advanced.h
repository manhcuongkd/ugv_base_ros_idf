#ifndef UGV_ADVANCED_H
#define UGV_ADVANCED_H

#include <stdbool.h>
#include <stdint.h>
#include "ugv_config.h"

// Mission system constants
#define MISSION_FILE_EXTENSION ".mission"
#define UGV_MAX_MISSION_NAME_LEN 64
#define MAX_MISSION_INTRO_LEN 256
#define MAX_STEP_JSON_LEN 512
#define FEEDBACK_BASE_INFO 1001
#define CMD_SET_SPD_RATE 138
#define CMD_MM_TYPE_SET 900
#define CMD_ARM_POSITION 104
#define CMD_DELAY 111

#ifdef __cplusplus
extern "C" {
#endif

// External configuration variable
extern ugv_config_t ugv_config;

// ============================================================================
// ADVANCED UGV FUNCTIONS (Matching Arduino Implementation)
// ============================================================================

// Mission Management Functions
bool create_mission(const char* name, const char* intro);
int mission_content(const char* name);
bool append_step_json(const char* name, const char* step);
void append_step_fb(const char* name, float speed);
void append_delay_cmd(const char* name, int delay_time);
bool insert_step_json(const char* name, int step_num, const char* step);
void insert_step_fb(const char* name, int step_num, float speed);
void insert_delay_cmd(const char* name, int step_num, int delay_time);
bool replace_step_json(const char* name, int step_num, const char* step);
void replace_step_fb(const char* name, int step_num, float speed);
void replace_delay_cmd(const char* name, int step_num, int delay_time);
void delete_step(const char* name, int step_num);
bool move_to_step(const char* name, int step_num);
void mission_play(const char* name, int repeat_times);
bool serial_mission_abort(void);

// JSON Command Processing
void json_cmd_receive_handler(void);

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
void change_module_type(uint8_t cmd);

#ifdef __cplusplus
}
#endif

#endif // UGV_ADVANCED_H
