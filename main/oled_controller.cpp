#include "../inc/oled_controller.h"
#include <esp_log.h>
#include <driver/i2c.h>
#include <freertos/FreeRTOS.h>
#include <freertos/task.h>
#include <string.h>
#include <math.h>

static const char *TAG = "OLEDController";

// I2C address for SSD1306 OLED display
#define OLED_I2C_ADDR 0x3C

// Global variable definition
oled_state_t oled_state;

// Private variables
static bool oled_initialized = false;
static uint8_t oled_buffer[OLED_WIDTH * OLED_HEIGHT / 8];

// Private function prototypes
static esp_err_t oled_write_command(uint8_t cmd);
static esp_err_t oled_write_data(uint8_t data);
static esp_err_t oled_set_position(uint8_t page, uint8_t col);
static void oled_clear_buffer(void);
static void oled_draw_char(uint8_t x, uint8_t y, char c);
static void oled_draw_string(uint8_t x, uint8_t y, const char *str);

// Font data (6x8 font)
static const uint8_t font_6x8[][6] = {
    {0x00, 0x00, 0x00, 0x00, 0x00, 0x00}, // Space
    {0x00, 0x00, 0x5F, 0x00, 0x00, 0x00}, // !
    {0x00, 0x07, 0x00, 0x07, 0x00, 0x00}, // "
    // Add more characters as needed...
};

esp_err_t oled_controller_init(void) {
    if (oled_initialized) {
        return ESP_OK;
    }

    ESP_LOGI(TAG, "Initializing OLED controller...");

    // Initialize I2C if not already done
    i2c_cmd_handle_t cmd = i2c_cmd_link_create();
    i2c_master_start(cmd);
    i2c_master_write_byte(cmd, (OLED_I2C_ADDR << 1) | I2C_MASTER_WRITE, true);
    i2c_master_stop(cmd);
    esp_err_t ret = i2c_master_cmd_begin(I2C_NUM_0, cmd, pdMS_TO_TICKS(100));
    i2c_cmd_link_delete(cmd);
    
    if (ret != ESP_OK) {
        ESP_LOGE(TAG, "OLED not found on I2C bus");
        return ESP_FAIL;
    }

    // Initialize OLED display
    oled_write_command(0xAE); // Display off
    oled_write_command(0xD5); // Set display clock
    oled_write_command(0x80);
    oled_write_command(0xA8); // Set multiplex ratio
    oled_write_command(0x3F);
    oled_write_command(0xD3); // Set display offset
    oled_write_command(0x00);
    oled_write_command(0x40); // Set start line
    oled_write_command(0x8D); // Charge pump
    oled_write_command(0x14);
    oled_write_command(0x20); // Memory mode
    oled_write_command(0x00);
    oled_write_command(0xA1); // Segment remap
    oled_write_command(0xC8); // COM scan direction
    oled_write_command(0xDA); // COM pins
    oled_write_command(0x12);
    oled_write_command(0x81); // Contrast
    oled_write_command(0xCF);
    oled_write_command(0xD9); // Pre-charge
    oled_write_command(0xF1);
    oled_write_command(0xDB); // VCOM detect
    oled_write_command(0x40);
    oled_write_command(0xA4); // Display all on resume
    oled_write_command(0xA6); // Normal display
    oled_write_command(0xAF); // Display on

    // Clear display
    oled_clear_buffer();
    oled_controller_clear_display();
    
    // Initialize global oled_state
    oled_state.initialized = true;
    oled_state.display_on = true;
    oled_state.brightness = 255;
    oled_state.last_update = xTaskGetTickCount() * portTICK_PERIOD_MS;
    
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

    // Clear the line first
    for (int x = 0; x < OLED_WIDTH; x++) {
        oled_buffer[line_num * OLED_WIDTH + x] = 0x00;
    }

    // Draw the text
    oled_draw_string(0, line_num, text);
    
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
    snprintf(line, sizeof(line), "Heap: %d KB", esp_get_free_heap_size() / 1024);
    oled_controller_display_text(1, line);
    
    // Line 2: Uptime
    uint32_t uptime = xTaskGetTickCount() * portTICK_PERIOD_MS / 1000;
    snprintf(line, sizeof(line), "Uptime: %02d:%02d", uptime / 60, uptime % 60);
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
    if (c < 32 || c > 126) c = 32; // Use space for invalid characters
    
    uint8_t char_index = c - 32;
    if (char_index < sizeof(font_6x8) / sizeof(font_6x8[0])) {
        for (uint8_t col = 0; col < 6; col++) {
            if (x + col < OLED_WIDTH) {
                oled_buffer[y * OLED_WIDTH + x + col] = font_6x8[char_index][col];
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
