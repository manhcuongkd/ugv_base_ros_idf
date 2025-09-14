#include "../inc/battery_controller.h"
#include "../inc/ugv_config.h"
#include <esp_log.h>
#include <driver/i2c.h>
#include <freertos/FreeRTOS.h>
#include <freertos/task.h>
#include <math.h>

static const char *TAG = "BatteryController";

// Private variables
static bool battery_initialized = false;
static uint16_t ina219_calibration;
static float current_lsb;
static float power_lsb;

// Private function prototypes
static esp_err_t ina219_write_register(uint8_t reg, uint16_t value);
static esp_err_t ina219_read_register(uint8_t reg, uint16_t *value);
static esp_err_t ina219_calibrate(void);
static float ina219_read_shunt_voltage(void);
static float ina219_read_bus_voltage(void);

esp_err_t battery_controller_init(void) {
    if (battery_initialized) {
        return ESP_OK;
    }

    ESP_LOGI(TAG, "Initializing battery controller...");

    // Check if INA219 is present
    uint16_t config;
    esp_err_t ret = ina219_read_register(INA219_REG_CONFIG, &config);
    if (ret != ESP_OK) {
        ESP_LOGE(TAG, "INA219 not found on I2C bus");
        return ESP_FAIL;
    }

    // Configure INA219
    uint16_t config_value = INA219_CONFIG_BUS_VOLTAGE_RANGE_16V |
                           INA219_CONFIG_GAIN_1_40MV |
                           INA219_CONFIG_BUS_ADC_RES_12BIT |
                           INA219_CONFIG_SHUNT_ADC_RES_12BIT |
                           INA219_CONFIG_MODE_SANDBVOLT_CONTINUOUS;

    ret = ina219_write_register(INA219_REG_CONFIG, config_value);
    if (ret != ESP_OK) {
        ESP_LOGE(TAG, "Failed to configure INA219");
        return ret;
    }

    // Calibrate the sensor
    ret = ina219_calibrate();
    if (ret != ESP_OK) {
        ESP_LOGE(TAG, "Failed to calibrate INA219");
        return ret;
    }

    battery_initialized = true;
    ESP_LOGI(TAG, "Battery controller initialized successfully");
    return ESP_OK;
}

esp_err_t battery_controller_read_voltage(float *voltage) {
    if (!battery_initialized || voltage == NULL) {
        return ESP_ERR_INVALID_STATE;
    }

    *voltage = ina219_read_bus_voltage();
    return ESP_OK;
}

esp_err_t battery_controller_read_current(float *current) {
    if (!battery_initialized || current == NULL) {
        return ESP_ERR_INVALID_STATE;
    }

    uint16_t raw_current;
    esp_err_t ret = ina219_read_register(INA219_REG_CURRENT, &raw_current);
    if (ret != ESP_OK) {
        return ret;
    }

    // Convert to signed value
    int16_t signed_current = (int16_t)raw_current;
    *current = signed_current * current_lsb;
    return ESP_OK;
}

esp_err_t battery_controller_read_power(float *power) {
    if (!battery_initialized || power == NULL) {
        return ESP_ERR_INVALID_STATE;
    }

    uint16_t raw_power;
    esp_err_t ret = ina219_read_register(INA219_REG_POWER, &raw_power);
    if (ret != ESP_OK) {
        return ret;
    }

    *power = raw_power * power_lsb;
    return ESP_OK;
}

battery_status_t battery_controller_get_status(void) {
    if (!battery_initialized) {
        return BATTERY_STATUS_UNKNOWN;
    }

    float voltage;
    if (battery_controller_read_voltage(&voltage) != ESP_OK) {
        return BATTERY_STATUS_UNKNOWN;
    }

    if (voltage < 10.0f) {
        return BATTERY_STATUS_CRITICAL;
    } else if (voltage < 11.0f) {
        return BATTERY_STATUS_LOW;
    } else if (voltage > 14.0f) {
        return BATTERY_STATUS_FULL;
    } else {
        return BATTERY_STATUS_NORMAL;
    }
}

uint8_t battery_controller_get_percentage(void) {
    if (!battery_initialized) {
        return 0;
    }

    float voltage;
    if (battery_controller_read_voltage(&voltage) != ESP_OK) {
        return 0;
    }

    // Simple linear mapping: 10V = 0%, 12.6V = 100%
    float percentage = ((voltage - 10.0f) / 2.6f) * 100.0f;
    
    if (percentage < 0) percentage = 0;
    if (percentage > 100) percentage = 100;
    
    return (uint8_t)percentage;
}

// Private functions
static esp_err_t ina219_write_register(uint8_t reg, uint16_t value) {
    i2c_cmd_handle_t cmd = i2c_cmd_link_create();
    i2c_master_start(cmd);
    i2c_master_write_byte(cmd, (INA219_I2C_ADDR << 1) | I2C_MASTER_WRITE, true);
    i2c_master_write_byte(cmd, reg, true);
    i2c_master_write_byte(cmd, (value >> 8) & 0xFF, true); // MSB first
    i2c_master_write_byte(cmd, value & 0xFF, true);        // LSB
    i2c_master_stop(cmd);
    esp_err_t ret = i2c_master_cmd_begin(I2C_NUM_0, cmd, pdMS_TO_TICKS(100));
    i2c_cmd_link_delete(cmd);
    return ret;
}

static esp_err_t ina219_read_register(uint8_t reg, uint16_t *value) {
    i2c_cmd_handle_t cmd = i2c_cmd_link_create();
    
    // Write register address
    i2c_master_start(cmd);
    i2c_master_write_byte(cmd, (INA219_I2C_ADDR << 1) | I2C_MASTER_WRITE, true);
    i2c_master_write_byte(cmd, reg, true);
    i2c_master_stop(cmd);
    esp_err_t ret = i2c_master_cmd_begin(I2C_NUM_0, cmd, pdMS_TO_TICKS(100));
    i2c_cmd_link_delete(cmd);
    
    if (ret != ESP_OK) {
        return ret;
    }
    
    // Read register value
    cmd = i2c_cmd_link_create();
    i2c_master_start(cmd);
    i2c_master_write_byte(cmd, (INA219_I2C_ADDR << 1) | I2C_MASTER_READ, true);
    i2c_master_read_byte(cmd, (uint8_t*)&value[0], I2C_MASTER_ACK);
    i2c_master_read_byte(cmd, (uint8_t*)&value[1], I2C_MASTER_NACK);
    i2c_master_stop(cmd);
    ret = i2c_master_cmd_begin(I2C_NUM_0, cmd, pdMS_TO_TICKS(100));
    i2c_cmd_link_delete(cmd);
    
    if (ret == ESP_OK) {
        // Swap bytes (INA219 sends MSB first)
        *value = ((*value & 0xFF) << 8) | ((*value >> 8) & 0xFF);
    }
    
    return ret;
}

static esp_err_t ina219_calibrate(void) {
    // Calculate calibration value
    // For 1.1A max current with 0.1 ohm shunt resistor
    float max_current = 1.1f;
    float shunt_resistor = 0.1f;
    
    current_lsb = max_current / 32768.0f;
    power_lsb = 20.0f * current_lsb;
    
    ina219_calibration = (uint16_t)(0.04096f / (current_lsb * shunt_resistor));
    
    return ina219_write_register(INA219_REG_CALIB, ina219_calibration);
}

static float __attribute__((unused)) ina219_read_shunt_voltage(void) {
    uint16_t raw_shunt;
    if (ina219_read_register(INA219_REG_SHUNTVOLT, &raw_shunt) != ESP_OK) {
        return 0.0f;
    }
    
    // Convert to voltage (LSB = 10uV)
    int16_t signed_shunt = (int16_t)raw_shunt;
    return signed_shunt * 0.00001f;
}

static float ina219_read_bus_voltage(void) {
    uint16_t raw_bus;
    if (ina219_read_register(INA219_REG_BUSVOLT, &raw_bus) != ESP_OK) {
        return 0.0f;
    }
    
    // Convert to voltage (LSB = 4mV)
    return (raw_bus >> 3) * 0.004f;
}
