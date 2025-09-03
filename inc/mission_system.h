#ifndef MISSION_SYSTEM_H
#define MISSION_SYSTEM_H

#include <stdbool.h>
#include <stdint.h>

#ifdef __cplusplus
extern "C" {
#endif

// ============================================================================
// MISSION SYSTEM FUNCTIONS (Matching Arduino Implementation)
// ============================================================================

// Mission creation and management
bool create_mission(const char *name, const char *intro);
int mission_content(const char *name);

// Step operations
bool append_step_json(const char *name, const char *step);
void append_step_feedback(const char *name, float speed);
void append_delay_cmd(const char *name, int delay_time);

bool insert_step_json(const char *name, int step_num, const char *step);
void insert_step_feedback(const char *name, int step_num, float speed);
void insert_delay_cmd(const char *name, int step_num, int delay_time);

bool replace_step_json(const char *name, int step_num, const char *step);
void replace_step_feedback(const char *name, int step_num, float speed);
void replace_delay_cmd(const char *name, int step_num, int delay_time);

void delete_step(const char *name, int step_num);

// Mission execution
bool move_to_step(const char *name, int step_num);
void mission_play(const char *name, int repeat_times);
bool serial_mission_abort(void);

#ifdef __cplusplus
}
#endif

#endif // MISSION_SYSTEM_H
