#include "../inc/oled_controller.h"
#include "../inc/battery_controller.h"
#include "../inc/ugv_config.h"
#include <esp_log.h>
#include <inttypes.h>
#include <esp_timer.h>
#include <driver/i2c.h>
#include <freertos/FreeRTOS.h>
#include <freertos/task.h>
#include <string.h>
#include <math.h>

static const char *TAG = "OLEDController";

// I2C address for SSD1306 OLED display
// OLED_I2C_ADDR defined in ugv_config.h

// Global variable definition
oled_state_t oled_state;

// Default display lines (Arduino-style)
oled_line_t screen_line_0;
oled_line_t screen_line_1;
oled_line_t screen_line_2;
oled_line_t screen_line_3;

// Private variables
static bool oled_initialized = false;
static uint8_t oled_buffer[OLED_WIDTH * OLED_HEIGHT / 8];
static bool screen_default_mode = true;  // Arduino-style default mode flag
static uint32_t last_info_update_time = 0;  // For periodic updates

// Private function prototypes
static esp_err_t oled_write_command(uint8_t cmd);
static esp_err_t oled_write_data(uint8_t data);
static esp_err_t oled_set_position(uint8_t page, uint8_t col);
static void oled_clear_buffer(void);
static void oled_draw_char(uint8_t x, uint8_t y, char c);
static void oled_draw_string(uint8_t x, uint8_t y, const char *str);

// Font data (6x8 font) - Basic ASCII characters
static const uint8_t font_6x8[][6] = {
    {0x00, 0x00, 0x00, 0x00, 0x00, 0x00}, // Space (32)
    {0x00, 0x00, 0x5F, 0x00, 0x00, 0x00}, // ! (33)
    {0x00, 0x07, 0x00, 0x07, 0x00, 0x00}, // " (34)
    {0x14, 0x7F, 0x14, 0x7F, 0x14, 0x00}, // # (35)
    {0x24, 0x2A, 0x7F, 0x2A, 0x12, 0x00}, // $ (36)
    {0x23, 0x13, 0x08, 0x64, 0x62, 0x00}, // % (37)
    {0x36, 0x49, 0x55, 0x22, 0x50, 0x00}, // & (38)
    {0x00, 0x05, 0x03, 0x00, 0x00, 0x00}, // ' (39)
    {0x00, 0x1C, 0x22, 0x41, 0x00, 0x00}, // ( (40)
    {0x00, 0x41, 0x22, 0x1C, 0x00, 0x00}, // ) (41)
    {0x08, 0x2A, 0x1C, 0x2A, 0x08, 0x00}, // * (42)
    {0x08, 0x08, 0x3E, 0x08, 0x08, 0x00}, // + (43)
    {0x00, 0x50, 0x30, 0x00, 0x00, 0x00}, // , (44)
    {0x08, 0x08, 0x08, 0x08, 0x08, 0x00}, // - (45)
    {0x00, 0x60, 0x60, 0x00, 0x00, 0x00}, // . (46)
    {0x20, 0x10, 0x08, 0x04, 0x02, 0x00}, // / (47)
    {0x3E, 0x51, 0x49, 0x45, 0x3E, 0x00}, // 0 (48)
    {0x00, 0x42, 0x7F, 0x40, 0x00, 0x00}, // 1 (49)
    {0x42, 0x61, 0x51, 0x49, 0x46, 0x00}, // 2 (50)
    {0x21, 0x41, 0x45, 0x4B, 0x31, 0x00}, // 3 (51)
    {0x18, 0x14, 0x12, 0x7F, 0x10, 0x00}, // 4 (52)
    {0x27, 0x45, 0x45, 0x45, 0x39, 0x00}, // 5 (53)
    {0x3C, 0x4A, 0x49, 0x49, 0x30, 0x00}, // 6 (54)
    {0x01, 0x71, 0x09, 0x05, 0x03, 0x00}, // 7 (55)
    {0x36, 0x49, 0x49, 0x49, 0x36, 0x00}, // 8 (56)
    {0x06, 0x49, 0x49, 0x29, 0x1E, 0x00}, // 9 (57)
    {0x00, 0x36, 0x36, 0x00, 0x00, 0x00}, // : (58)
    {0x00, 0x56, 0x36, 0x00, 0x00, 0x00}, // ; (59)
    {0x00, 0x08, 0x14, 0x22, 0x41, 0x00}, // < (60)
    {0x14, 0x14, 0x14, 0x14, 0x14, 0x00}, // = (61)
    {0x41, 0x22, 0x14, 0x08, 0x00, 0x00}, // > (62)
    {0x02, 0x01, 0x51, 0x09, 0x06, 0x00}, // ? (63)
    {0x32, 0x49, 0x79, 0x41, 0x3E, 0x00}, // @ (64)
    {0x7E, 0x11, 0x11, 0x11, 0x7E, 0x00}, // A (65)
    {0x7F, 0x49, 0x49, 0x49, 0x36, 0x00}, // B (66)
    {0x3E, 0x41, 0x41, 0x41, 0x22, 0x00}, // C (67)
    {0x7F, 0x41, 0x41, 0x22, 0x1C, 0x00}, // D (68)
    {0x7F, 0x49, 0x49, 0x49, 0x41, 0x00}, // E (69)
    {0x7F, 0x09, 0x09, 0x01, 0x01, 0x00}, // F (70)
    {0x3E, 0x41, 0x41, 0x51, 0x32, 0x00}, // G (71)
    {0x7F, 0x08, 0x08, 0x08, 0x7F, 0x00}, // H (72)
    {0x00, 0x41, 0x7F, 0x41, 0x00, 0x00}, // I (73)
    {0x20, 0x40, 0x41, 0x3F, 0x01, 0x00}, // J (74)
    {0x7F, 0x08, 0x14, 0x22, 0x41, 0x00}, // K (75)
    {0x7F, 0x40, 0x40, 0x40, 0x40, 0x00}, // L (76)
    {0x7F, 0x02, 0x04, 0x02, 0x7F, 0x00}, // M (77)
    {0x7F, 0x04, 0x08, 0x10, 0x7F, 0x00}, // N (78)
    {0x3E, 0x41, 0x41, 0x41, 0x3E, 0x00}, // O (79)
    {0x7F, 0x09, 0x09, 0x09, 0x06, 0x00}, // P (80)
    {0x3E, 0x41, 0x51, 0x21, 0x5E, 0x00}, // Q (81)
    {0x7F, 0x09, 0x19, 0x29, 0x46, 0x00}, // R (82)
    {0x46, 0x49, 0x49, 0x49, 0x31, 0x00}, // S (83)
    {0x01, 0x01, 0x7F, 0x01, 0x01, 0x00}, // T (84)
    {0x3F, 0x40, 0x40, 0x40, 0x3F, 0x00}, // U (85)
    {0x1F, 0x20, 0x40, 0x20, 0x1F, 0x00}, // V (86)
    {0x7F, 0x20, 0x18, 0x20, 0x7F, 0x00}, // W (87)
    {0x63, 0x14, 0x08, 0x14, 0x63, 0x00}, // X (88)
    {0x03, 0x04, 0x78, 0x04, 0x03, 0x00}, // Y (89)
    {0x61, 0x51, 0x49, 0x45, 0x43, 0x00}, // Z (90)
    {0x00, 0x00, 0x7F, 0x41, 0x41, 0x00}, // [ (91)
    {0x02, 0x04, 0x08, 0x10, 0x20, 0x00}, // \ (92)
    {0x41, 0x41, 0x7F, 0x00, 0x00, 0x00}, // ] (93)
    {0x04, 0x02, 0x01, 0x02, 0x04, 0x00}, // ^ (94)
    {0x40, 0x40, 0x40, 0x40, 0x40, 0x00}, // _ (95)
    {0x00, 0x00, 0x00, 0x00, 0x00, 0x00}, // ` (96) - unused
    {0x20, 0x54, 0x54, 0x54, 0x78, 0x00}, // a (97)
    {0x7F, 0x48, 0x44, 0x44, 0x38, 0x00}, // b (98)
    {0x38, 0x44, 0x44, 0x44, 0x20, 0x00}, // c (99)
    {0x38, 0x44, 0x44, 0x48, 0x7F, 0x00}, // d (100)
    {0x38, 0x54, 0x54, 0x54, 0x18, 0x00}, // e (101)
    {0x08, 0x7E, 0x09, 0x01, 0x02, 0x00}, // f (102)
    {0x0C, 0x52, 0x52, 0x52, 0x3E, 0x00}, // g (103)
    {0x7F, 0x08, 0x04, 0x04, 0x78, 0x00}, // h (104)
    {0x00, 0x44, 0x7D, 0x40, 0x00, 0x00}, // i (105)
    {0x20, 0x40, 0x44, 0x3D, 0x00, 0x00}, // j (106)
    {0x7F, 0x10, 0x28, 0x44, 0x00, 0x00}, // k (107)
    {0x00, 0x41, 0x7F, 0x40, 0x00, 0x00}, // l (108)
    {0x7C, 0x04, 0x18, 0x04, 0x78, 0x00}, // m (109)
    {0x7C, 0x08, 0x04, 0x04, 0x78, 0x00}, // n (110)
    {0x38, 0x44, 0x44, 0x44, 0x38, 0x00}, // o (111)
    {0x7C, 0x14, 0x14, 0x14, 0x08, 0x00}, // p (112)
    {0x08, 0x14, 0x14, 0x18, 0x7C, 0x00}, // q (113)
    {0x7C, 0x08, 0x04, 0x04, 0x08, 0x00}, // r (114)
    {0x48, 0x54, 0x54, 0x54, 0x20, 0x00}, // s (115)
    {0x04, 0x3F, 0x44, 0x40, 0x20, 0x00}, // t (116)
    {0x3C, 0x40, 0x40, 0x20, 0x7C, 0x00}, // u (117)
    {0x1C, 0x20, 0x40, 0x20, 0x1C, 0x00}, // v (118)
    {0x3C, 0x40, 0x30, 0x40, 0x3C, 0x00}, // w (119)
    {0x44, 0x28, 0x10, 0x28, 0x44, 0x00}, // x (120)
    {0x0C, 0x50, 0x50, 0x50, 0x3C, 0x00}, // y (121)
    {0x44, 0x64, 0x54, 0x4C, 0x44, 0x00}, // z (122)
};

esp_err_t oled_controller_init(void) {
    if (oled_initialized) {
        return ESP_OK;
    }

    ESP_LOGI(TAG, "Initializing OLED controller...");

    // Wait a bit for I2C driver to be fully ready
    vTaskDelay(pdMS_TO_TICKS(100));

    // Test I2C communication with OLED
    i2c_cmd_handle_t cmd = i2c_cmd_link_create();
    i2c_master_start(cmd);
    i2c_master_write_byte(cmd, (OLED_I2C_ADDR << 1) | I2C_MASTER_WRITE, true);
    i2c_master_stop(cmd);
    esp_err_t ret = i2c_master_cmd_begin(I2C_NUM_0, cmd, pdMS_TO_TICKS(1000));
    i2c_cmd_link_delete(cmd);
    
    if (ret != ESP_OK) {
        ESP_LOGW(TAG, "OLED not found on I2C bus (%s), continuing without OLED", esp_err_to_name(ret));
        return ESP_FAIL;
    }

    // Reset sequence (Arduino-style: no reset pin, just wait)
    vTaskDelay(pdMS_TO_TICKS(10));

    // Initialize OLED display (Arduino Adafruit_SSD1306 sequence for 128x32)
    oled_write_command(0xAE); // Display off
    oled_write_command(0xD5); // Set display clock div
    oled_write_command(0x80); // Suggested ratio 0x80
    oled_write_command(0xA8); // Set multiplex ratio
    oled_write_command(0x1F); // 31 for 128x32 display (HEIGHT - 1)
    oled_write_command(0xD3); // Set display offset
    oled_write_command(0x00); // No offset
    oled_write_command(0x40); // Set start line (0x40 | 0x0)
    oled_write_command(0x8D); // Charge pump
    oled_write_command(0x14); // Enable charge pump (0x14 for internal VCC)
    oled_write_command(0x20); // Memory mode
    oled_write_command(0x00); // 0x0 act like ks0108
    oled_write_command(0xA1); // Segment remap (0xA1 | 0x1)
    oled_write_command(0xC8); // COM scan direction (COMSCANDEC)
    oled_write_command(0xDA); // COM pins
    oled_write_command(0x02); // 0x02 for 128x32 display
    oled_write_command(0x81); // Contrast
    oled_write_command(0x8F); // 0x8F for 128x32 display
    oled_write_command(0xD9); // Pre-charge
    oled_write_command(0xF1); // 0xF1 for internal VCC
    oled_write_command(0xDB); // VCOM detect
    oled_write_command(0x40);
    oled_write_command(0xA4); // Display all on resume
    oled_write_command(0xA6); // Normal display
    oled_write_command(0x2E); // Deactivate scroll
    oled_write_command(0xAF); // Display on

    // Clear display
    oled_clear_buffer();
    oled_controller_clear_display();
    
    // Initialize global oled_state
    oled_state.initialized = true;
    oled_state.display_on = true;
    oled_state.brightness = 255;
    oled_state.last_update = xTaskGetTickCount() * portTICK_PERIOD_MS;
    
    // Initialize default screen lines (Arduino-style)
    strncpy(screen_line_0.text, "RaspRover IDF", OLED_MAX_CHARS_PER_LINE);
    screen_line_0.text[OLED_MAX_CHARS_PER_LINE] = '\0';
    screen_line_0.updated = true;
    screen_line_0.timestamp = esp_timer_get_time() / 1000;
    
    strncpy(screen_line_1.text, "Initializing...", OLED_MAX_CHARS_PER_LINE);
    screen_line_1.text[OLED_MAX_CHARS_PER_LINE] = '\0';
    screen_line_1.updated = true;
    screen_line_1.timestamp = esp_timer_get_time() / 1000;
    
    strncpy(screen_line_2.text, "Please wait...", OLED_MAX_CHARS_PER_LINE);
    screen_line_2.text[OLED_MAX_CHARS_PER_LINE] = '\0';
    screen_line_2.updated = true;
    screen_line_2.timestamp = esp_timer_get_time() / 1000;
    
    strncpy(screen_line_3.text, "V:12.0V Ready", OLED_MAX_CHARS_PER_LINE);
    screen_line_3.text[OLED_MAX_CHARS_PER_LINE] = '\0';
    screen_line_3.updated = true;
    screen_line_3.timestamp = esp_timer_get_time() / 1000;
    
    // Initialize oled_state lines
    for (int i = 0; i < OLED_MAX_LINES; i++) {
        oled_state.lines[i].text[0] = '\0';
        oled_state.lines[i].updated = false;
        oled_state.lines[i].timestamp = 0;
    }
    
    oled_initialized = true;
    ESP_LOGI(TAG, "OLED controller initialized successfully");
    return ESP_OK;
}

esp_err_t oled_controller_display_text(uint8_t line_num, const char *text) {
    if (!oled_initialized || line_num >= OLED_MAX_LINES) {
        return ESP_ERR_INVALID_STATE;
    }

    if (text == NULL) {
        text = "";
    }

    // Store text in line buffer (Arduino-style)
    strncpy(oled_state.lines[line_num].text, text, OLED_MAX_CHARS_PER_LINE);
    oled_state.lines[line_num].text[OLED_MAX_CHARS_PER_LINE] = '\0';

    // Clear the line area in buffer (8 pixels per line) - page-based indexing
    uint8_t page = line_num;
    if (page < OLED_HEIGHT / 8) {
        for (int x = 0; x < OLED_WIDTH; x++) {
            oled_buffer[page * OLED_WIDTH + x] = 0x00;
        }
    }

    // Draw the text at correct y position (line_num * 8)
    oled_draw_string(0, line_num * 8, text);
    
    // Update the display
    return oled_controller_update();
}

esp_err_t oled_controller_clear_display(void) {
    if (!oled_initialized) {
        return ESP_ERR_INVALID_STATE;
    }

    oled_clear_buffer();
    return oled_controller_update();
}

esp_err_t oled_controller_update(void) {
    if (!oled_initialized) {
        return ESP_ERR_INVALID_STATE;
    }

    // Update display with buffer contents
    for (uint8_t page = 0; page < OLED_HEIGHT / 8; page++) {
        oled_set_position(page, 0);
        for (uint8_t col = 0; col < OLED_WIDTH; col++) {
            oled_write_data(oled_buffer[page * OLED_WIDTH + col]);
        }
    }
    
    return ESP_OK;
}

esp_err_t oled_controller_update_system_info(void) {
    if (!oled_initialized) {
        return ESP_ERR_INVALID_STATE;
    }

    // Update system information display
    char line[21];
    
    // Line 0: System status
    snprintf(line, sizeof(line), "RaspRover IDF v1.0");
    oled_controller_display_text(0, line);
    
    // Line 1: Free heap
    snprintf(line, sizeof(line), "Heap: %" PRIu32 " KB", (uint32_t)(esp_get_free_heap_size() / 1024));
    oled_controller_display_text(1, line);
    
    // Line 2: Uptime
    uint32_t uptime = xTaskGetTickCount() * portTICK_PERIOD_MS / 1000;
    snprintf(line, sizeof(line), "Uptime: %02" PRIu32 ":%02" PRIu32, uptime / 60, uptime % 60);
    oled_controller_display_text(2, line);
    
    // Line 3: System status
    snprintf(line, sizeof(line), "Status: Ready");
    oled_controller_display_text(3, line);
    
    return oled_controller_update();
}

// Private functions
static esp_err_t oled_write_command(uint8_t cmd) {
    i2c_cmd_handle_t cmd_handle = i2c_cmd_link_create();
    i2c_master_start(cmd_handle);
    i2c_master_write_byte(cmd_handle, (OLED_I2C_ADDR << 1) | I2C_MASTER_WRITE, true);
    i2c_master_write_byte(cmd_handle, 0x00, true); // Command mode
    i2c_master_write_byte(cmd_handle, cmd, true);
    i2c_master_stop(cmd_handle);
    esp_err_t ret = i2c_master_cmd_begin(I2C_NUM_0, cmd_handle, pdMS_TO_TICKS(100));
    i2c_cmd_link_delete(cmd_handle);
    return ret;
}

static esp_err_t oled_write_data(uint8_t data) {
    i2c_cmd_handle_t cmd_handle = i2c_cmd_link_create();
    i2c_master_start(cmd_handle);
    i2c_master_write_byte(cmd_handle, (OLED_I2C_ADDR << 1) | I2C_MASTER_WRITE, true);
    i2c_master_write_byte(cmd_handle, 0x40, true); // Data mode
    i2c_master_write_byte(cmd_handle, data, true);
    i2c_master_stop(cmd_handle);
    esp_err_t ret = i2c_master_cmd_begin(I2C_NUM_0, cmd_handle, pdMS_TO_TICKS(100));
    i2c_cmd_link_delete(cmd_handle);
    return ret;
}

static esp_err_t oled_set_position(uint8_t page, uint8_t col) {
    oled_write_command(0xB0 + page);        // Set page address
    oled_write_command(0x02 + (col >> 4)); // Set column address high
    oled_write_command(0x10 + (col & 0x0F)); // Set column address low
    return ESP_OK;
}

static void oled_clear_buffer(void) {
    memset(oled_buffer, 0, sizeof(oled_buffer));
}

static void oled_draw_char(uint8_t x, uint8_t y, char c) {
    if (c < 32 || c > 122) c = 32; // Use space for invalid characters (font covers 32-122)
    
    uint8_t char_index = c - 32;
    if (char_index < sizeof(font_6x8) / sizeof(font_6x8[0])) {
        // Calculate page and row within page
        uint8_t page = y / 8;
        uint8_t row_in_page = y % 8;
        
        for (uint8_t col = 0; col < 6; col++) {
            if (x + col < OLED_WIDTH && page < OLED_HEIGHT / 8) {
                // Use page-based indexing: page * OLED_WIDTH + column
                uint16_t buffer_index = page * OLED_WIDTH + x + col;
                if (buffer_index < sizeof(oled_buffer)) {
                    oled_buffer[buffer_index] |= (font_6x8[char_index][col] << row_in_page);
                }
            }
        }
    }
}

static void oled_draw_string(uint8_t x, uint8_t y, const char *str) {
    uint8_t current_x = x;
    while (*str && current_x < OLED_WIDTH - 6) {
        oled_draw_char(current_x, y, *str);
        current_x += 6;
        str++;
    }
}

esp_err_t oled_controller_set_text(uint8_t line_num, const char *text) {
    if (!oled_initialized) {
        return ESP_ERR_INVALID_STATE;
    }
    
    if (line_num >= OLED_MAX_LINES || !text) {
        return ESP_ERR_INVALID_ARG;
    }
    
    ESP_LOGI(TAG, "Setting OLED line %d text: %s", line_num, text);
    
    // Copy text to line buffer (truncate if too long)
    strncpy(oled_state.lines[line_num].text, text, OLED_MAX_CHARS_PER_LINE);
    oled_state.lines[line_num].text[OLED_MAX_CHARS_PER_LINE] = '\0';
    oled_state.lines[line_num].updated = true;
    oled_state.lines[line_num].timestamp = esp_timer_get_time() / 1000; // Convert to ms
    
    // Clear buffer and redraw all lines
    oled_clear_buffer();
    
    for (uint8_t i = 0; i < OLED_MAX_LINES; i++) {
        if (oled_state.lines[i].text[0] != '\0') {
            oled_draw_string(0, i * 8, oled_state.lines[i].text);
        }
    }
    
    // Update display
    oled_set_position(0, 0);
    for (uint16_t i = 0; i < sizeof(oled_buffer); i++) {
        oled_write_data(oled_buffer[i]);
    }
    
    ESP_LOGI(TAG, "OLED text updated successfully");
    return ESP_OK;
}

esp_err_t oled_controller_reset_to_default(void) {
    if (!oled_initialized) {
        return ESP_ERR_INVALID_STATE;
    }
    
    ESP_LOGI(TAG, "Resetting OLED to default mode");
    
    // Switch to default mode (Arduino-style)
    screen_default_mode = true;
    
    // Update voltage info (Arduino-style)
    strncpy(screen_line_3.text, "V:12.0V Ready", OLED_MAX_CHARS_PER_LINE);
    screen_line_3.text[OLED_MAX_CHARS_PER_LINE] = '\0';
    screen_line_3.updated = true;
    screen_line_3.timestamp = esp_timer_get_time() / 1000;
    
    // Update display with default lines
    oled_clear_buffer();
    
    // Display all default lines
    if (screen_line_0.text[0] != '\0') {
        oled_draw_string(0, 0, screen_line_0.text);
    }
    if (screen_line_1.text[0] != '\0') {
        oled_draw_string(0, 8, screen_line_1.text);
    }
    if (screen_line_2.text[0] != '\0') {
        oled_draw_string(0, 16, screen_line_2.text);
    }
    if (screen_line_3.text[0] != '\0') {
        oled_draw_string(0, 24, screen_line_3.text);
    }
    
    // Update display
    oled_set_position(0, 0);
    for (uint16_t i = 0; i < sizeof(oled_buffer); i++) {
        oled_write_data(oled_buffer[i]);
    }
    
    // Update timestamp for periodic updates
    last_info_update_time = esp_timer_get_time() / 1000;
    
    ESP_LOGI(TAG, "OLED reset to default mode successfully");
    return ESP_OK;
}

// Arduino-style OLED control function
esp_err_t oled_controller_control(uint8_t line_num, const char *text) {
    if (!oled_initialized) {
        return ESP_ERR_INVALID_STATE;
    }
    
    if (line_num >= OLED_MAX_LINES || !text) {
        return ESP_ERR_INVALID_ARG;
    }
    
    ESP_LOGI(TAG, "OLED control: line=%d, text=%s", line_num, text);
    
    // Switch to custom mode (Arduino-style)
    screen_default_mode = false;
    
    // Set custom line text
    strncpy(oled_state.lines[line_num].text, text, OLED_MAX_CHARS_PER_LINE);
    oled_state.lines[line_num].text[OLED_MAX_CHARS_PER_LINE] = '\0';
    oled_state.lines[line_num].updated = true;
    oled_state.lines[line_num].timestamp = esp_timer_get_time() / 1000;
    
    // Clear display and show custom lines
    oled_clear_buffer();
    
    for (uint8_t i = 0; i < OLED_MAX_LINES; i++) {
        if (oled_state.lines[i].text[0] != '\0') {
            oled_draw_string(0, i * 8, oled_state.lines[i].text);
        }
    }
    
    // Update display
    oled_set_position(0, 0);
    for (uint16_t i = 0; i < sizeof(oled_buffer); i++) {
        oled_write_data(oled_buffer[i]);
    }
    
    ESP_LOGI(TAG, "OLED custom display updated");
    return ESP_OK;
}

// Arduino-style periodic info update
esp_err_t oled_controller_info_update(void) {
    if (!oled_initialized || !screen_default_mode) {
        return ESP_OK;  // Only update in default mode
    }
    
    uint32_t current_time = esp_timer_get_time() / 1000;
    
    // Update every 10 seconds (Arduino-style)
    if (current_time - last_info_update_time < 10000) {
        return ESP_OK;
    }
    
    // Update voltage info (Arduino-style: "V:"+String(loadVoltage_V) + " s " +String(mainType) + String(moduleType))
    char voltage_str[16];
    float voltage = 0.0f;
    esp_err_t voltage_result = battery_controller_read_voltage(&voltage);
    
    if (voltage_result == ESP_OK) {
        snprintf(voltage_str, sizeof(voltage_str), "V:%.1fV", voltage);
    } else {
        snprintf(voltage_str, sizeof(voltage_str), "V:--.-V");
    }
    
    // Update system status info (Arduino-style)
    // Line 1: Uptime
    uint32_t uptime_seconds = current_time / 1000;
    uint32_t minutes = uptime_seconds / 60;
    uint32_t seconds = uptime_seconds % 60;
    char uptime_str[16];
    snprintf(uptime_str, sizeof(uptime_str), "Up: %02" PRIu32 ":%02" PRIu32, minutes, seconds);
    strncpy(screen_line_0.text, uptime_str, OLED_MAX_CHARS_PER_LINE - 1);
    screen_line_0.text[OLED_MAX_CHARS_PER_LINE - 1] = '\0';
    screen_line_0.updated = true;
    screen_line_0.timestamp = current_time;
    
    // Line 2: System status
    strncpy(screen_line_1.text, "System Ready", OLED_MAX_CHARS_PER_LINE - 1);
    screen_line_1.text[OLED_MAX_CHARS_PER_LINE - 1] = '\0';
    screen_line_1.updated = true;
    screen_line_1.timestamp = current_time;
    
    // Line 3: Free heap memory
    uint32_t free_heap = esp_get_free_heap_size() / 1024;
    char heap_str[16];
    snprintf(heap_str, sizeof(heap_str), "Heap: %" PRIu32 "KB", free_heap);
    strncpy(screen_line_2.text, heap_str, OLED_MAX_CHARS_PER_LINE - 1);
    screen_line_2.text[OLED_MAX_CHARS_PER_LINE - 1] = '\0';
    screen_line_2.updated = true;
    screen_line_2.timestamp = current_time;
    
    // Line 4: Voltage and module type info (Arduino-style: mainType + moduleType)
    extern ugv_config_t ugv_config;  // Access global config
    char temp_buffer[32];
    snprintf(temp_buffer, sizeof(temp_buffer), "%s s %d%d", 
             voltage_str, ugv_config.main_type, ugv_config.module_type);
    strncpy(screen_line_3.text, temp_buffer, OLED_MAX_CHARS_PER_LINE - 1);
    screen_line_3.text[OLED_MAX_CHARS_PER_LINE - 1] = '\0';  // Ensure null termination
    screen_line_3.updated = true;
    screen_line_3.timestamp = current_time;
    
    // Update display
    oled_clear_buffer();
    
    if (screen_line_0.text[0] != '\0') {
        oled_draw_string(0, 0, screen_line_0.text);
    }
    if (screen_line_1.text[0] != '\0') {
        oled_draw_string(0, 8, screen_line_1.text);
    }
    if (screen_line_2.text[0] != '\0') {
        oled_draw_string(0, 16, screen_line_2.text);
    }
    if (screen_line_3.text[0] != '\0') {
        oled_draw_string(0, 24, screen_line_3.text);
    }
    
    oled_set_position(0, 0);
    for (uint16_t i = 0; i < sizeof(oled_buffer); i++) {
        oled_write_data(oled_buffer[i]);
    }
    
    last_info_update_time = current_time;
    return ESP_OK;
}
