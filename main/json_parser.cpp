#include "../inc/json_parser.h"
#include "../inc/motion_module.h"
#include "../inc/imu_controller.h"
#include <esp_log.h>
#include <cJSON.h>
#include <freertos/FreeRTOS.h>
#include <freertos/queue.h>
#include <freertos/task.h>
#include <string.h>

static const char *TAG = "JSONParser";

// Private variables
static bool json_parser_initialized = false;
static QueueHandle_t feedback_queue = NULL;

// Private function prototypes
static esp_err_t json_parser_parse_command_data(const cJSON *json, json_command_t *cmd);

esp_err_t json_parser_init(void) {
    if (json_parser_initialized) {
        return ESP_OK;
    }

    ESP_LOGI(TAG, "Initializing JSON parser...");

    // Create feedback queue
    feedback_queue = xQueueCreate(10, sizeof(json_feedback_t));
    if (feedback_queue == NULL) {
        ESP_LOGE(TAG, "Failed to create feedback queue");
        return ESP_FAIL;
    }

    json_parser_initialized = true;
    ESP_LOGI(TAG, "JSON parser initialized successfully");
    return ESP_OK;
}

esp_err_t json_parser_parse_command(const char *json_str, json_command_t *cmd) {
    if (!json_parser_initialized || json_str == NULL || cmd == NULL) {
        return ESP_ERR_INVALID_ARG;
    }

    // Parse JSON string
    cJSON *json = cJSON_Parse(json_str);
    if (json == NULL) {
        ESP_LOGE(TAG, "Failed to parse JSON: %s", json_str);
        return ESP_ERR_INVALID_ARG;
    }

    // Get command type (T field)
    cJSON *type_json = cJSON_GetObjectItem(json, "T");
    if (type_json == NULL || !cJSON_IsNumber(type_json)) {
        ESP_LOGE(TAG, "Missing or invalid 'T' field in JSON");
        cJSON_Delete(json);
        return ESP_ERR_INVALID_ARG;
    }

    cmd->type = (uint16_t)type_json->valueint;
    ESP_LOGI(TAG, "Parsing command type: %d", cmd->type);

    // Parse command data based on type
    esp_err_t result = json_parser_parse_command_data(json, cmd);
    
    cJSON_Delete(json);
    return result;
}

esp_err_t json_parser_process_command(json_command_t *cmd) {
    if (!json_parser_initialized || cmd == NULL) {
        return ESP_ERR_INVALID_STATE;
    }

    ESP_LOGI(TAG, "Processing JSON command type: %d", cmd->type);
    // Commands are now processed in uart_controller.cpp
    return ESP_OK;
}

static esp_err_t json_parser_parse_command_data(const cJSON *json, json_command_t *cmd) {
    switch (cmd->type) {
        case CMD_EMERGENCY_STOP: // {"T":0}
            // No additional data needed
            break;

        case CMD_SPEED_CTRL: // {"T":1,"L":0.5,"R":0.5}
            {
                cJSON *L = cJSON_GetObjectItem(json, "L");
                cJSON *R = cJSON_GetObjectItem(json, "R");
                if (L && cJSON_IsNumber(L)) cmd->data.speed_ctrl.L = L->valuedouble;
                if (R && cJSON_IsNumber(R)) cmd->data.speed_ctrl.R = R->valuedouble;
            }
            break;

        case CMD_SET_MOTOR_PID: // {"T":2,"P":200,"I":2500,"D":0,"L":255}
            {
                cJSON *P = cJSON_GetObjectItem(json, "P");
                cJSON *I = cJSON_GetObjectItem(json, "I");
                cJSON *D = cJSON_GetObjectItem(json, "D");
                cJSON *L = cJSON_GetObjectItem(json, "L");
                if (P && cJSON_IsNumber(P)) cmd->data.pid_params.P = P->valuedouble;
                if (I && cJSON_IsNumber(I)) cmd->data.pid_params.I = I->valuedouble;
                if (D && cJSON_IsNumber(D)) cmd->data.pid_params.D = D->valuedouble;
                if (L && cJSON_IsNumber(L)) cmd->data.pid_params.L = L->valuedouble;
            }
            break;

        case CMD_OLED_CTRL: // {"T":3,"lineNum":0,"Text":"Hello"}
            {
                cJSON *lineNum = cJSON_GetObjectItem(json, "lineNum");
                cJSON *Text = cJSON_GetObjectItem(json, "Text");
                if (lineNum && cJSON_IsNumber(lineNum)) cmd->data.oled_ctrl.lineNum = lineNum->valueint;
                if (Text && cJSON_IsString(Text)) {
                    strncpy(cmd->data.oled_ctrl.Text, Text->valuestring, sizeof(cmd->data.oled_ctrl.Text) - 1);
                    cmd->data.oled_ctrl.Text[sizeof(cmd->data.oled_ctrl.Text) - 1] = '\0';
                }
            }
            break;

        case CMD_MODULE_TYPE: // {"T":4,"cmd":0}
            {
                cJSON *cmd_item = cJSON_GetObjectItem(json, "cmd");
                if (cmd_item && cJSON_IsNumber(cmd_item)) cmd->data.module_type.cmd = cmd_item->valueint;
            }
            break;

        case CMD_PWM_INPUT: // {"T":11,"L":164,"R":164}
            {
                cJSON *L = cJSON_GetObjectItem(json, "L");
                cJSON *R = cJSON_GetObjectItem(json, "R");
                if (L && cJSON_IsNumber(L)) cmd->data.pwm_input.L = L->valueint;
                if (R && cJSON_IsNumber(R)) cmd->data.pwm_input.R = R->valueint;
            }
            break;

        case CMD_ROS_CTRL: // {"T":13,"X":0.1,"Z":0.3}
            {
                cJSON *X = cJSON_GetObjectItem(json, "X");
                cJSON *Z = cJSON_GetObjectItem(json, "Z");
                if (X && cJSON_IsNumber(X)) cmd->data.ros_ctrl.X = X->valuedouble;
                if (Z && cJSON_IsNumber(Z)) cmd->data.ros_ctrl.Z = Z->valuedouble;
            }
            break;

        case CMD_SET_SPD_RATE: // {"T":138,"L":1,"R":1}
            {
                cJSON *L = cJSON_GetObjectItem(json, "L");
                cJSON *R = cJSON_GetObjectItem(json, "R");
                if (L && cJSON_IsNumber(L)) cmd->data.set_spd_rate.L = L->valuedouble;
                if (R && cJSON_IsNumber(R)) cmd->data.set_spd_rate.R = R->valuedouble;
            }
            break;

        case CMD_MM_TYPE_SET: // {"T":900,"main":1,"module":0}
            {
                cJSON *main = cJSON_GetObjectItem(json, "main");
                cJSON *module = cJSON_GetObjectItem(json, "module");
                if (main && cJSON_IsNumber(main)) cmd->data.mm_type_set.main = main->valueint;
                if (module && cJSON_IsNumber(module)) cmd->data.mm_type_set.module = module->valueint;
            }
            break;

        case CMD_SET_IMU_OFFSET: // {"T":129,"gx":0,"gy":0,"gz":0,"ax":0,"ay":0,"az":0,"cx":0,"cy":0,"cz":0}
            {
                cJSON *gx = cJSON_GetObjectItem(json, "gx");
                cJSON *gy = cJSON_GetObjectItem(json, "gy");
                cJSON *gz = cJSON_GetObjectItem(json, "gz");
                cJSON *ax = cJSON_GetObjectItem(json, "ax");
                cJSON *ay = cJSON_GetObjectItem(json, "ay");
                cJSON *az = cJSON_GetObjectItem(json, "az");
                cJSON *cx = cJSON_GetObjectItem(json, "cx");
                cJSON *cy = cJSON_GetObjectItem(json, "cy");
                cJSON *cz = cJSON_GetObjectItem(json, "cz");
                if (gx && cJSON_IsNumber(gx)) cmd->data.set_imu_offset.gx = gx->valuedouble;
                if (gy && cJSON_IsNumber(gy)) cmd->data.set_imu_offset.gy = gy->valuedouble;
                if (gz && cJSON_IsNumber(gz)) cmd->data.set_imu_offset.gz = gz->valuedouble;
                if (ax && cJSON_IsNumber(ax)) cmd->data.set_imu_offset.ax = ax->valuedouble;
                if (ay && cJSON_IsNumber(ay)) cmd->data.set_imu_offset.ay = ay->valuedouble;
                if (az && cJSON_IsNumber(az)) cmd->data.set_imu_offset.az = az->valuedouble;
                if (cx && cJSON_IsNumber(cx)) cmd->data.set_imu_offset.cx = cx->valuedouble;
                if (cy && cJSON_IsNumber(cy)) cmd->data.set_imu_offset.cy = cy->valuedouble;
                if (cz && cJSON_IsNumber(cz)) cmd->data.set_imu_offset.cz = cz->valuedouble;
            }
            break;

        case CMD_BASE_FEEDBACK_FLOW: // {"T":131,"cmd":1}
            {
                cJSON *cmd_item = cJSON_GetObjectItem(json, "cmd");
                if (cmd_item && cJSON_IsNumber(cmd_item)) cmd->data.base_feedback_flow.cmd = cmd_item->valueint;
            }
            break;

        case CMD_FEEDBACK_FLOW_INTERVAL: // {"T":142,"cmd":0}
            {
                cJSON *cmd_item = cJSON_GetObjectItem(json, "cmd");
                if (cmd_item && cJSON_IsNumber(cmd_item)) cmd->data.feedback_flow_interval.cmd = cmd_item->valueint;
            }
            break;

        case CMD_UART_ECHO_MODE: // {"T":143,"cmd":0}
            {
                cJSON *cmd_item = cJSON_GetObjectItem(json, "cmd");
                if (cmd_item && cJSON_IsNumber(cmd_item)) cmd->data.uart_echo_mode.cmd = cmd_item->valueint;
            }
            break;

        case CMD_LED_CTRL: // {"T":132,"IO4":255,"IO5":255}
            {
                cJSON *IO4 = cJSON_GetObjectItem(json, "IO4");
                cJSON *IO5 = cJSON_GetObjectItem(json, "IO5");
                if (IO4 && cJSON_IsNumber(IO4)) cmd->data.led_ctrl.IO4 = IO4->valueint;
                if (IO5 && cJSON_IsNumber(IO5)) cmd->data.led_ctrl.IO5 = IO5->valueint;
            }
            break;

        case CMD_GIMBAL_CTRL_SIMPLE: // {"T":133,"X":45,"Y":45,"SPD":0,"ACC":0}
            {
                cJSON *X = cJSON_GetObjectItem(json, "X");
                cJSON *Y = cJSON_GetObjectItem(json, "Y");
                cJSON *SPD = cJSON_GetObjectItem(json, "SPD");
                cJSON *ACC = cJSON_GetObjectItem(json, "ACC");
                if (X && cJSON_IsNumber(X)) cmd->data.gimbal_ctrl_simple.X = X->valuedouble;
                if (Y && cJSON_IsNumber(Y)) cmd->data.gimbal_ctrl_simple.Y = Y->valuedouble;
                if (SPD && cJSON_IsNumber(SPD)) cmd->data.gimbal_ctrl_simple.SPD = SPD->valuedouble;
                if (ACC && cJSON_IsNumber(ACC)) cmd->data.gimbal_ctrl_simple.ACC = ACC->valuedouble;
            }
            break;

        case CMD_GIMBAL_CTRL_MOVE: // {"T":134,"X":45,"Y":45,"SX":300,"SY":300}
            {
                cJSON *X = cJSON_GetObjectItem(json, "X");
                cJSON *Y = cJSON_GetObjectItem(json, "Y");
                cJSON *SX = cJSON_GetObjectItem(json, "SX");
                cJSON *SY = cJSON_GetObjectItem(json, "SY");
                if (X && cJSON_IsNumber(X)) cmd->data.gimbal_ctrl_move.X = X->valuedouble;
                if (Y && cJSON_IsNumber(Y)) cmd->data.gimbal_ctrl_move.Y = Y->valuedouble;
                if (SX && cJSON_IsNumber(SX)) cmd->data.gimbal_ctrl_move.SX = SX->valuedouble;
                if (SY && cJSON_IsNumber(SY)) cmd->data.gimbal_ctrl_move.SY = SY->valuedouble;
            }
            break;

        case CMD_HEART_BEAT_SET: // {"T":136,"cmd":3000}
            {
                cJSON *cmd_item = cJSON_GetObjectItem(json, "cmd");
                if (cmd_item && cJSON_IsNumber(cmd_item)) cmd->data.heart_beat_set.cmd = cmd_item->valueint;
            }
            break;

        case CMD_GIMBAL_STEADY: // {"T":137,"s":1,"y":0}
            {
                cJSON *s = cJSON_GetObjectItem(json, "s");
                cJSON *y = cJSON_GetObjectItem(json, "y");
                if (s && cJSON_IsNumber(s)) cmd->data.gimbal_steady.s = s->valueint;
                if (y && cJSON_IsNumber(y)) cmd->data.gimbal_steady.y = y->valueint;
            }
            break;

        case CMD_GIMBAL_USER_CTRL: // {"T":141,"X":0,"Y":0,"SPD":300}
            {
                cJSON *X = cJSON_GetObjectItem(json, "X");
                cJSON *Y = cJSON_GetObjectItem(json, "Y");
                cJSON *SPD = cJSON_GetObjectItem(json, "SPD");
                if (X && cJSON_IsNumber(X)) cmd->data.gimbal_user_ctrl.X = X->valuedouble;
                if (Y && cJSON_IsNumber(Y)) cmd->data.gimbal_user_ctrl.Y = Y->valuedouble;
                if (SPD && cJSON_IsNumber(SPD)) cmd->data.gimbal_user_ctrl.SPD = SPD->valuedouble;
            }
            break;

        case CMD_ARM_CTRL_UI: // {"T":144,"E":100,"Z":0,"R":0}
            {
                cJSON *E = cJSON_GetObjectItem(json, "E");
                cJSON *Z = cJSON_GetObjectItem(json, "Z");
                cJSON *R = cJSON_GetObjectItem(json, "R");
                if (E && cJSON_IsNumber(E)) cmd->data.arm_ctrl_ui.E = E->valuedouble;
                if (Z && cJSON_IsNumber(Z)) cmd->data.arm_ctrl_ui.Z = Z->valuedouble;
                if (R && cJSON_IsNumber(R)) cmd->data.arm_ctrl_ui.R = R->valuedouble;
            }
            break;

        case CMD_SINGLE_JOINT_CTRL: // {"T":101,"joint":0,"rad":0,"spd":0,"acc":10}
            {
                cJSON *joint = cJSON_GetObjectItem(json, "joint");
                cJSON *rad = cJSON_GetObjectItem(json, "rad");
                cJSON *spd = cJSON_GetObjectItem(json, "spd");
                cJSON *acc = cJSON_GetObjectItem(json, "acc");
                if (joint && cJSON_IsNumber(joint)) cmd->data.single_joint_ctrl.joint = joint->valueint;
                if (rad && cJSON_IsNumber(rad)) cmd->data.single_joint_ctrl.rad = rad->valuedouble;
                if (spd && cJSON_IsNumber(spd)) cmd->data.single_joint_ctrl.spd = spd->valuedouble;
                if (acc && cJSON_IsNumber(acc)) cmd->data.single_joint_ctrl.acc = acc->valuedouble;
            }
            break;

        case CMD_JOINTS_RAD_CTRL: // {"T":102,"base":0,"shoulder":0,"elbow":1.57,"hand":1.57,"spd":0,"acc":10}
            {
                cJSON *base = cJSON_GetObjectItem(json, "base");
                cJSON *shoulder = cJSON_GetObjectItem(json, "shoulder");
                cJSON *elbow = cJSON_GetObjectItem(json, "elbow");
                cJSON *hand = cJSON_GetObjectItem(json, "hand");
                cJSON *spd = cJSON_GetObjectItem(json, "spd");
                cJSON *acc = cJSON_GetObjectItem(json, "acc");
                if (base && cJSON_IsNumber(base)) cmd->data.joints_rad_ctrl.base = base->valuedouble;
                if (shoulder && cJSON_IsNumber(shoulder)) cmd->data.joints_rad_ctrl.shoulder = shoulder->valuedouble;
                if (elbow && cJSON_IsNumber(elbow)) cmd->data.joints_rad_ctrl.elbow = elbow->valuedouble;
                if (hand && cJSON_IsNumber(hand)) cmd->data.joints_rad_ctrl.hand = hand->valuedouble;
                if (spd && cJSON_IsNumber(spd)) cmd->data.joints_rad_ctrl.spd = spd->valuedouble;
                if (acc && cJSON_IsNumber(acc)) cmd->data.joints_rad_ctrl.acc = acc->valuedouble;
            }
            break;

        case CMD_XYZT_GOAL_CTRL: // {"T":104,"x":235,"y":0,"z":234,"t":3.14,"spd":0.25}
            {
                cJSON *x = cJSON_GetObjectItem(json, "x");
                cJSON *y = cJSON_GetObjectItem(json, "y");
                cJSON *z = cJSON_GetObjectItem(json, "z");
                cJSON *t = cJSON_GetObjectItem(json, "t");
                cJSON *spd = cJSON_GetObjectItem(json, "spd");
                if (x && cJSON_IsNumber(x)) cmd->data.xyzt_goal_ctrl.x = x->valuedouble;
                if (y && cJSON_IsNumber(y)) cmd->data.xyzt_goal_ctrl.y = y->valuedouble;
                if (z && cJSON_IsNumber(z)) cmd->data.xyzt_goal_ctrl.z = z->valuedouble;
                if (t && cJSON_IsNumber(t)) cmd->data.xyzt_goal_ctrl.t = t->valuedouble;
                if (spd && cJSON_IsNumber(spd)) cmd->data.xyzt_goal_ctrl.spd = spd->valuedouble;
            }
            break;

        case CMD_EOAT_HAND_CTRL: // {"T":106,"cmd":1.57,"spd":0,"acc":0}
            {
                cJSON *cmd_item = cJSON_GetObjectItem(json, "cmd");
                cJSON *spd = cJSON_GetObjectItem(json, "spd");
                cJSON *acc = cJSON_GetObjectItem(json, "acc");
                if (cmd_item && cJSON_IsNumber(cmd_item)) cmd->data.eoat_hand_ctrl.cmd = cmd_item->valuedouble;
                if (spd && cJSON_IsNumber(spd)) cmd->data.eoat_hand_ctrl.spd = spd->valuedouble;
                if (acc && cJSON_IsNumber(acc)) cmd->data.eoat_hand_ctrl.acc = acc->valuedouble;
            }
            break;

        case CMD_EOAT_GRAB_TORQUE: // {"T":107,"tor":200}
            {
                cJSON *tor = cJSON_GetObjectItem(json, "tor");
                if (tor && cJSON_IsNumber(tor)) cmd->data.eoat_grab_torque.tor = tor->valuedouble;
            }
            break;

        case CMD_SET_JOINT_PID: // {"T":108,"joint":3,"p":16,"i":0}
            {
                cJSON *joint = cJSON_GetObjectItem(json, "joint");
                cJSON *p = cJSON_GetObjectItem(json, "p");
                cJSON *i = cJSON_GetObjectItem(json, "i");
                if (joint && cJSON_IsNumber(joint)) cmd->data.set_joint_pid.joint = joint->valueint;
                if (p && cJSON_IsNumber(p)) cmd->data.set_joint_pid.p = p->valuedouble;
                if (i && cJSON_IsNumber(i)) cmd->data.set_joint_pid.i = i->valuedouble;
            }
            break;

        case CMD_READ_FILE: // {"T":202,"name":"file.txt"}
            {
                cJSON *name = cJSON_GetObjectItem(json, "name");
                if (name && cJSON_IsString(name)) {
                    strncpy(cmd->data.read_file.name, name->valuestring, sizeof(cmd->data.read_file.name) - 1);
                    cmd->data.read_file.name[sizeof(cmd->data.read_file.name) - 1] = '\0';
                }
            }
            break;

        case CMD_READ_LINE: // {"T":207,"name":"file.txt","lineNum":3}
            {
                cJSON *name = cJSON_GetObjectItem(json, "name");
                cJSON *lineNum = cJSON_GetObjectItem(json, "lineNum");
                if (name && cJSON_IsString(name)) {
                    strncpy(cmd->data.read_line.name, name->valuestring, sizeof(cmd->data.read_line.name) - 1);
                    cmd->data.read_line.name[sizeof(cmd->data.read_line.name) - 1] = '\0';
                }
                if (lineNum && cJSON_IsNumber(lineNum)) cmd->data.read_line.lineNum = lineNum->valueint;
            }
            break;

        case CMD_TORQUE_CTRL: // {"T":210,"cmd":1}
            {
                cJSON *cmd_item = cJSON_GetObjectItem(json, "cmd");
                if (cmd_item && cJSON_IsNumber(cmd_item)) cmd->data.torque_ctrl.cmd = cmd_item->valueint;
            }
            break;

        case CMD_BROADCAST_FOLLOWER: // {"T":300,"mode":1,"mac":"FF:FF:FF:FF:FF:FF"}
            {
                cJSON *mode = cJSON_GetObjectItem(json, "mode");
                cJSON *mac = cJSON_GetObjectItem(json, "mac");
                if (mode && cJSON_IsNumber(mode)) cmd->data.broadcast_follower.mode = mode->valueint;
                if (mac && cJSON_IsString(mac)) {
                    strncpy(cmd->data.broadcast_follower.mac, mac->valuestring, sizeof(cmd->data.broadcast_follower.mac) - 1);
                    cmd->data.broadcast_follower.mac[sizeof(cmd->data.broadcast_follower.mac) - 1] = '\0';
                }
            }
            break;

        case CMD_ESP_NOW_CONFIG: // {"T":301,"mode":3}
            {
                cJSON *mode = cJSON_GetObjectItem(json, "mode");
                if (mode && cJSON_IsNumber(mode)) cmd->data.esp_now_config.mode = mode->valueint;
            }
            break;

        case CMD_ESP_NOW_ADD_FOLLOWER: // {"T":303,"mac":"FF:FF:FF:FF:FF:FF"}
            {
                cJSON *mac = cJSON_GetObjectItem(json, "mac");
                if (mac && cJSON_IsString(mac)) {
                    strncpy(cmd->data.esp_now_add_follower.mac, mac->valuestring, sizeof(cmd->data.esp_now_add_follower.mac) - 1);
                    cmd->data.esp_now_add_follower.mac[sizeof(cmd->data.esp_now_add_follower.mac) - 1] = '\0';
                }
            }
            break;

        case CMD_ESP_NOW_REMOVE_FOLLOWER: // {"T":304,"mac":"FF:FF:FF:FF:FF:FF"}
            {
                cJSON *mac = cJSON_GetObjectItem(json, "mac");
                if (mac && cJSON_IsString(mac)) {
                    strncpy(cmd->data.esp_now_remove_follower.mac, mac->valuestring, sizeof(cmd->data.esp_now_remove_follower.mac) - 1);
                    cmd->data.esp_now_remove_follower.mac[sizeof(cmd->data.esp_now_remove_follower.mac) - 1] = '\0';
                }
            }
            break;

        case CMD_ESP_NOW_GROUP_CTRL: // {"T":305,"dev":0,"b":0,"s":0,"e":1.57,"h":1.57,"cmd":0,"megs":"hello!"}
            {
                cJSON *dev = cJSON_GetObjectItem(json, "dev");
                cJSON *b = cJSON_GetObjectItem(json, "b");
                cJSON *s = cJSON_GetObjectItem(json, "s");
                cJSON *e = cJSON_GetObjectItem(json, "e");
                cJSON *h = cJSON_GetObjectItem(json, "h");
                cJSON *cmd_item = cJSON_GetObjectItem(json, "cmd");
                cJSON *megs = cJSON_GetObjectItem(json, "megs");
                if (dev && cJSON_IsNumber(dev)) cmd->data.esp_now_group_ctrl.dev = dev->valueint;
                if (b && cJSON_IsNumber(b)) cmd->data.esp_now_group_ctrl.b = b->valuedouble;
                if (s && cJSON_IsNumber(s)) cmd->data.esp_now_group_ctrl.s = s->valuedouble;
                if (e && cJSON_IsNumber(e)) cmd->data.esp_now_group_ctrl.e = e->valuedouble;
                if (h && cJSON_IsNumber(h)) cmd->data.esp_now_group_ctrl.h = h->valuedouble;
                if (cmd_item && cJSON_IsNumber(cmd_item)) cmd->data.esp_now_group_ctrl.cmd = cmd_item->valueint;
                if (megs && cJSON_IsString(megs)) {
                    strncpy(cmd->data.esp_now_group_ctrl.megs, megs->valuestring, sizeof(cmd->data.esp_now_group_ctrl.megs) - 1);
                    cmd->data.esp_now_group_ctrl.megs[sizeof(cmd->data.esp_now_group_ctrl.megs) - 1] = '\0';
                }
            }
            break;

        case CMD_ESP_NOW_SINGLE: // {"T":306,"mac":"FF:FF:FF:FF:FF:FF","dev":0,"b":0,"s":0,"e":1.57,"h":1.57,"cmd":0,"megs":"hello!"}
            {
                cJSON *mac = cJSON_GetObjectItem(json, "mac");
                cJSON *dev = cJSON_GetObjectItem(json, "dev");
                cJSON *b = cJSON_GetObjectItem(json, "b");
                cJSON *s = cJSON_GetObjectItem(json, "s");
                cJSON *e = cJSON_GetObjectItem(json, "e");
                cJSON *h = cJSON_GetObjectItem(json, "h");
                cJSON *cmd_item = cJSON_GetObjectItem(json, "cmd");
                cJSON *megs = cJSON_GetObjectItem(json, "megs");
                if (mac && cJSON_IsString(mac)) {
                    strncpy(cmd->data.esp_now_single.mac, mac->valuestring, sizeof(cmd->data.esp_now_single.mac) - 1);
                    cmd->data.esp_now_single.mac[sizeof(cmd->data.esp_now_single.mac) - 1] = '\0';
                }
                if (dev && cJSON_IsNumber(dev)) cmd->data.esp_now_single.dev = dev->valueint;
                if (b && cJSON_IsNumber(b)) cmd->data.esp_now_single.b = b->valuedouble;
                if (s && cJSON_IsNumber(s)) cmd->data.esp_now_single.s = s->valuedouble;
                if (e && cJSON_IsNumber(e)) cmd->data.esp_now_single.e = e->valuedouble;
                if (h && cJSON_IsNumber(h)) cmd->data.esp_now_single.h = h->valuedouble;
                if (cmd_item && cJSON_IsNumber(cmd_item)) cmd->data.esp_now_single.cmd = cmd_item->valueint;
                if (megs && cJSON_IsString(megs)) {
                    strncpy(cmd->data.esp_now_single.megs, megs->valuestring, sizeof(cmd->data.esp_now_single.megs) - 1);
                    cmd->data.esp_now_single.megs[sizeof(cmd->data.esp_now_single.megs) - 1] = '\0';
                }
            }
            break;

        case CMD_WIFI_ON_BOOT: // {"T":401,"cmd":3}
            {
                cJSON *cmd_item = cJSON_GetObjectItem(json, "cmd");
                if (cmd_item && cJSON_IsNumber(cmd_item)) cmd->data.wifi_on_boot.cmd = cmd_item->valueint;
            }
            break;

        case CMD_SET_AP: // {"T":402,"ssid":"RoArm-M2","password":"12345678"}
            {
                cJSON *ssid = cJSON_GetObjectItem(json, "ssid");
                cJSON *password = cJSON_GetObjectItem(json, "password");
                if (ssid && cJSON_IsString(ssid)) {
                    strncpy(cmd->data.set_ap.ssid, ssid->valuestring, sizeof(cmd->data.set_ap.ssid) - 1);
                    cmd->data.set_ap.ssid[sizeof(cmd->data.set_ap.ssid) - 1] = '\0';
                }
                if (password && cJSON_IsString(password)) {
                    strncpy(cmd->data.set_ap.password, password->valuestring, sizeof(cmd->data.set_ap.password) - 1);
                    cmd->data.set_ap.password[sizeof(cmd->data.set_ap.password) - 1] = '\0';
                }
            }
            break;

        case CMD_SET_STA: // {"T":403,"ssid":"JSBZY-2.4G","password":"waveshare0755"}
            {
                cJSON *ssid = cJSON_GetObjectItem(json, "ssid");
                cJSON *password = cJSON_GetObjectItem(json, "password");
                if (ssid && cJSON_IsString(ssid)) {
                    strncpy(cmd->data.set_sta.ssid, ssid->valuestring, sizeof(cmd->data.set_sta.ssid) - 1);
                    cmd->data.set_sta.ssid[sizeof(cmd->data.set_sta.ssid) - 1] = '\0';
                }
                if (password && cJSON_IsString(password)) {
                    strncpy(cmd->data.set_sta.password, password->valuestring, sizeof(cmd->data.set_sta.password) - 1);
                    cmd->data.set_sta.password[sizeof(cmd->data.set_sta.password) - 1] = '\0';
                }
            }
            break;

        case CMD_WIFI_APSTA: // {"T":404,"ap_ssid":"RoArm-M2","ap_password":"12345678","sta_ssid":"JSBZY-2.4G","sta_password":"waveshare0755"}
            {
                cJSON *ap_ssid = cJSON_GetObjectItem(json, "ap_ssid");
                cJSON *ap_password = cJSON_GetObjectItem(json, "ap_password");
                cJSON *sta_ssid = cJSON_GetObjectItem(json, "sta_ssid");
                cJSON *sta_password = cJSON_GetObjectItem(json, "sta_password");
                if (ap_ssid && cJSON_IsString(ap_ssid)) {
                    strncpy(cmd->data.wifi_apsta.ap_ssid, ap_ssid->valuestring, sizeof(cmd->data.wifi_apsta.ap_ssid) - 1);
                    cmd->data.wifi_apsta.ap_ssid[sizeof(cmd->data.wifi_apsta.ap_ssid) - 1] = '\0';
                }
                if (ap_password && cJSON_IsString(ap_password)) {
                    strncpy(cmd->data.wifi_apsta.ap_password, ap_password->valuestring, sizeof(cmd->data.wifi_apsta.ap_password) - 1);
                    cmd->data.wifi_apsta.ap_password[sizeof(cmd->data.wifi_apsta.ap_password) - 1] = '\0';
                }
                if (sta_ssid && cJSON_IsString(sta_ssid)) {
                    strncpy(cmd->data.wifi_apsta.sta_ssid, sta_ssid->valuestring, sizeof(cmd->data.wifi_apsta.sta_ssid) - 1);
                    cmd->data.wifi_apsta.sta_ssid[sizeof(cmd->data.wifi_apsta.sta_ssid) - 1] = '\0';
                }
                if (sta_password && cJSON_IsString(sta_password)) {
                    strncpy(cmd->data.wifi_apsta.sta_password, sta_password->valuestring, sizeof(cmd->data.wifi_apsta.sta_password) - 1);
                    cmd->data.wifi_apsta.sta_password[sizeof(cmd->data.wifi_apsta.sta_password) - 1] = '\0';
                }
            }
            break;

        case CMD_WIFI_CONFIG_CREATE_BY_INPUT: // {"T":407,"mode":3,"ap_ssid":"RoArm-M2","ap_password":"12345678","sta_ssid":"JSBZY-2.4G","sta_password":"waveshare0755"}
            {
                cJSON *mode = cJSON_GetObjectItem(json, "mode");
                cJSON *ap_ssid = cJSON_GetObjectItem(json, "ap_ssid");
                cJSON *ap_password = cJSON_GetObjectItem(json, "ap_password");
                cJSON *sta_ssid = cJSON_GetObjectItem(json, "sta_ssid");
                cJSON *sta_password = cJSON_GetObjectItem(json, "sta_password");
                if (mode && cJSON_IsNumber(mode)) cmd->data.wifi_config_create_by_input.mode = mode->valueint;
                if (ap_ssid && cJSON_IsString(ap_ssid)) {
                    strncpy(cmd->data.wifi_config_create_by_input.ap_ssid, ap_ssid->valuestring, sizeof(cmd->data.wifi_config_create_by_input.ap_ssid) - 1);
                    cmd->data.wifi_config_create_by_input.ap_ssid[sizeof(cmd->data.wifi_config_create_by_input.ap_ssid) - 1] = '\0';
                }
                if (ap_password && cJSON_IsString(ap_password)) {
                    strncpy(cmd->data.wifi_config_create_by_input.ap_password, ap_password->valuestring, sizeof(cmd->data.wifi_config_create_by_input.ap_password) - 1);
                    cmd->data.wifi_config_create_by_input.ap_password[sizeof(cmd->data.wifi_config_create_by_input.ap_password) - 1] = '\0';
                }
                if (sta_ssid && cJSON_IsString(sta_ssid)) {
                    strncpy(cmd->data.wifi_config_create_by_input.sta_ssid, sta_ssid->valuestring, sizeof(cmd->data.wifi_config_create_by_input.sta_ssid) - 1);
                    cmd->data.wifi_config_create_by_input.sta_ssid[sizeof(cmd->data.wifi_config_create_by_input.sta_ssid) - 1] = '\0';
                }
                if (sta_password && cJSON_IsString(sta_password)) {
                    strncpy(cmd->data.wifi_config_create_by_input.sta_password, sta_password->valuestring, sizeof(cmd->data.wifi_config_create_by_input.sta_password) - 1);
                    cmd->data.wifi_config_create_by_input.sta_password[sizeof(cmd->data.wifi_config_create_by_input.sta_password) - 1] = '\0';
                }
            }
            break;

        case CMD_SET_SERVO_ID: // {"T":501,"raw":1,"new":11}
            {
                cJSON *raw = cJSON_GetObjectItem(json, "raw");
                cJSON *new_id = cJSON_GetObjectItem(json, "new");
                if (raw && cJSON_IsNumber(raw)) cmd->data.set_servo_id.raw = raw->valueint;
                if (new_id && cJSON_IsNumber(new_id)) cmd->data.set_servo_id.new_id = new_id->valueint;
            }
            break;

        case CMD_SET_MIDDLE: // {"T":502,"id":11}
            {
                cJSON *id = cJSON_GetObjectItem(json, "id");
                if (id && cJSON_IsNumber(id)) cmd->data.set_middle.id = id->valueint;
            }
            break;

        case CMD_SET_SERVO_PID: // {"T":503,"id":14,"p":16}
            {
                cJSON *id = cJSON_GetObjectItem(json, "id");
                cJSON *p = cJSON_GetObjectItem(json, "p");
                if (id && cJSON_IsNumber(id)) cmd->data.set_servo_pid.id = id->valueint;
                if (p && cJSON_IsNumber(p)) cmd->data.set_servo_pid.p = p->valuedouble;
            }
            break;

        case CMD_INFO_PRINT: // {"T":605,"cmd":1}
            {
                cJSON *cmd_item = cJSON_GetObjectItem(json, "cmd");
                if (cmd_item && cJSON_IsNumber(cmd_item)) cmd->data.info_print.cmd = cmd_item->valueint;
            }
            break;

        default:
            ESP_LOGW(TAG, "Unknown command type: %d", cmd->type);
            break;
    }

    return ESP_OK;
}

esp_err_t json_parser_send_base_feedback(void) {
    if (!json_parser_initialized) {
        return ESP_ERR_INVALID_STATE;
    }

    ESP_LOGI(TAG, "Sending base feedback");
    
    // Create base feedback JSON
    cJSON *json = cJSON_CreateObject();
    cJSON_AddNumberToObject(json, "T", FEEDBACK_BASE_INFO);
    
    // Get motion status
    motion_control_t motion_status;
    if (motion_module_get_status(&motion_status) == ESP_OK) {
        cJSON_AddNumberToObject(json, "L", motion_status.left_speed);
        cJSON_AddNumberToObject(json, "R", motion_status.right_speed);
    } else {
        cJSON_AddNumberToObject(json, "L", 0.0);
        cJSON_AddNumberToObject(json, "R", 0.0);
    }
    
    // Get IMU data
    const imu_data_t *imu_data = imu_controller_get_data();
    if (imu_data) {
        cJSON_AddNumberToObject(json, "gx", imu_data->gyro_x);
        cJSON_AddNumberToObject(json, "gy", imu_data->gyro_y);
        cJSON_AddNumberToObject(json, "gz", imu_data->gyro_z);
        cJSON_AddNumberToObject(json, "ax", imu_data->accel_x);
        cJSON_AddNumberToObject(json, "ay", imu_data->accel_y);
        cJSON_AddNumberToObject(json, "az", imu_data->accel_z);
        cJSON_AddNumberToObject(json, "mx", imu_data->mag_x);
        cJSON_AddNumberToObject(json, "my", imu_data->mag_y);
        cJSON_AddNumberToObject(json, "mz", imu_data->mag_z);
    } else {
        // Default values if IMU data not available
        cJSON_AddNumberToObject(json, "gx", 0.0);
        cJSON_AddNumberToObject(json, "gy", 0.0);
        cJSON_AddNumberToObject(json, "gz", 0.0);
        cJSON_AddNumberToObject(json, "ax", 0.0);
        cJSON_AddNumberToObject(json, "ay", 0.0);
        cJSON_AddNumberToObject(json, "az", 0.0);
        cJSON_AddNumberToObject(json, "mx", 0.0);
        cJSON_AddNumberToObject(json, "my", 0.0);
        cJSON_AddNumberToObject(json, "mz", 0.0);
    }
    
    // Get encoder data (odometry)
    encoder_data_t encoder_data;
    if (motion_module_get_encoder_data(&encoder_data) == ESP_OK) {
        cJSON_AddNumberToObject(json, "odl", encoder_data.left_count);
        cJSON_AddNumberToObject(json, "odr", encoder_data.right_count);
    } else {
        cJSON_AddNumberToObject(json, "odl", 0);
        cJSON_AddNumberToObject(json, "odr", 0);
    }
    
    // Get battery voltage (placeholder - would need battery controller)
    cJSON_AddNumberToObject(json, "v", 11.0); // Default voltage
    
    // Send via UART
    char *json_string = cJSON_Print(json);
    if (json_string) {
        printf("%s\n", json_string);
        free(json_string);
    }
    
    cJSON_Delete(json);
    ESP_LOGI(TAG, "Base feedback sent successfully");
    return ESP_OK;
}

esp_err_t json_parser_deinit(void) {
    if (!json_parser_initialized) {
        return ESP_OK;
    }

    ESP_LOGI(TAG, "Deinitializing JSON parser...");

    if (feedback_queue != NULL) {
        vQueueDelete(feedback_queue);
        feedback_queue = NULL;
    }

    json_parser_initialized = false;
    ESP_LOGI(TAG, "JSON parser deinitialized");
    return ESP_OK;
}
