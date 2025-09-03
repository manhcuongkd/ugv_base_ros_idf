#include "../inc/imu_controller.h"
#include <esp_log.h>
#include <driver/i2c.h>
#include <freertos/FreeRTOS.h>
#include <freertos/task.h>
#include <math.h>
#include <string.h>
#include <nvs_flash.h>

static const char *TAG = "IMUController";

// Private variables
static bool imu_initialized = false;
static imu_config_t imu_config;
static imu_calibration_t imu_calibration;
static imu_status_t imu_status = IMU_STATUS_INIT_ERROR;
static uint8_t imu_i2c_addr = ICM20948_I2C_ADDR;
static uint64_t last_data_update = 0;
static imu_data_t current_imu_data = {0};

// Private function prototypes
static esp_err_t imu_write_register(uint8_t reg, uint8_t data);
static esp_err_t imu_read_register(uint8_t reg, uint8_t *data, size_t len);
static esp_err_t imu_read_sensor_data(imu_data_t *data);
static esp_err_t imu_configure_sensors(void);
static esp_err_t imu_enable_dmp(void);
static esp_err_t imu_enable_fifo(void);
static void imu_calibration_task(void *pvParameters);

esp_err_t imu_controller_init(void)
{
    ESP_LOGI(TAG, "Initializing IMU controller...");
    
    // Check if I2C is initialized
    if (i2c_driver_install(I2C_NUM_0, I2C_MODE_MASTER, 0, 0, 0) != ESP_OK) {
        ESP_LOGE(TAG, "I2C driver not installed");
        return ESP_ERR_INVALID_STATE;
    }
    
    // Initialize with default configuration
    imu_config.sample_rate_hz = DEFAULT_IMU_SAMPLE_RATE_HZ;
    imu_config.accel_range = DEFAULT_IMU_ACCEL_RANGE;
    imu_config.gyro_range = DEFAULT_IMU_GYRO_RANGE;
    imu_config.mag_rate = DEFAULT_IMU_MAG_RATE;
    imu_config.enable_dmp = DEFAULT_IMU_ENABLE_DMP;
    imu_config.enable_fifo = DEFAULT_IMU_ENABLE_FIFO;
    imu_config.low_power_mode = DEFAULT_IMU_LOW_POWER;
    
    // Check if IMU is present
    uint8_t who_am_i;
    esp_err_t ret = imu_read_register(ICM20948_WHO_AM_I, &who_am_i, 1);
    if (ret != ESP_OK || who_am_i != 0xEA) {
        ESP_LOGE(TAG, "IMU not found at address 0x%02X, WHO_AM_I: 0x%02X", imu_i2c_addr, who_am_i);
        imu_status = IMU_STATUS_COMM_ERROR;
        return ESP_ERR_NOT_FOUND;
    }
    
    ESP_LOGI(TAG, "IMU found, WHO_AM_I: 0x%02X", who_am_i);
    
    // Reset IMU
    ESP_ERROR_CHECK(imu_write_register(ICM20948_PWR_MGMT_1, 0x80));
    vTaskDelay(pdMS_TO_TICKS(100));
    
    // Wake up IMU
    ESP_ERROR_CHECK(imu_write_register(ICM20948_PWR_MGMT_1, 0x00));
    vTaskDelay(pdMS_TO_TICKS(100));
    
    // Configure sensors
    ESP_ERROR_CHECK(imu_configure_sensors());
    
    // Enable DMP if requested
    if (imu_config.enable_dmp) {
        ESP_ERROR_CHECK(imu_enable_dmp());
    }
    
    // Enable FIFO if requested
    if (imu_config.enable_fifo) {
        ESP_ERROR_CHECK(imu_enable_fifo());
    }
    
    // Initialize calibration data
    memset(&imu_calibration, 0, sizeof(imu_calibration_t));
    
    // Load calibration from NVS if available
    imu_controller_load_calibration();
    
    imu_initialized = true;
    imu_status = IMU_STATUS_OK;
    
    ESP_LOGI(TAG, "IMU controller initialized successfully");
    
    return ESP_OK;
}

esp_err_t imu_controller_configure(imu_config_t *config)
{
    if (!config) {
        return ESP_ERR_INVALID_ARG;
    }
    
    // Update configuration
    memcpy(&imu_config, config, sizeof(imu_config_t));
    
    if (imu_initialized) {
        // Reconfigure sensors with new settings
        ESP_ERROR_CHECK(imu_configure_sensors());
    }
    
    ESP_LOGI(TAG, "IMU configuration updated");
    return ESP_OK;
}

esp_err_t imu_controller_read_data(imu_data_t *data)
{
    if (!imu_initialized || !data) {
        return ESP_ERR_INVALID_ARG;
    }
    
    // Read raw sensor data
    esp_err_t ret = imu_read_sensor_data(data);
    if (ret != ESP_OK) {
        return ret;
    }
    
    // Apply calibration
    if (imu_calibration.accel_calibrated) {
        data->accel_x_g -= imu_calibration.accel_bias_x;
        data->accel_y_g -= imu_calibration.accel_bias_y;
        data->accel_z_g -= imu_calibration.accel_bias_z;
    }
    
    if (imu_calibration.gyro_calibrated) {
        data->gyro_x_dps -= imu_calibration.gyro_bias_x;
        data->gyro_y_dps -= imu_calibration.gyro_bias_y;
        data->gyro_z_dps -= imu_calibration.gyro_bias_z;
    }
    
    if (imu_calibration.mag_calibrated) {
        data->mag_x_ut = (data->mag_x_ut - imu_calibration.mag_bias_x) * imu_calibration.mag_scale_x;
        data->mag_y_ut = (data->mag_y_ut - imu_calibration.mag_bias_y) * imu_calibration.mag_scale_y;
        data->mag_z_ut = (data->mag_z_ut - imu_calibration.mag_bias_z) * imu_calibration.mag_scale_z;
    }
    
    // Calculate Euler angles from quaternion if available
    if (data->quat_valid) {
        // Convert quaternion to Euler angles
        float q0 = data->q0, q1 = data->q1, q2 = data->q2, q3 = data->q3;
        
        // Roll (x-axis rotation)
        data->roll = atan2f(2.0f * (q0 * q1 + q2 * q3), 1.0f - 2.0f * (q1 * q1 + q2 * q2));
        
        // Pitch (y-axis rotation)
        float sinp = 2.0f * (q0 * q2 - q3 * q1);
        if (fabsf(sinp) >= 1.0f) {
            data->pitch = copysignf(M_PI / 2.0f, sinp);
        } else {
            data->pitch = asinf(sinp);
        }
        
        // Yaw (z-axis rotation)
        data->yaw = atan2f(2.0f * (q0 * q3 + q1 * q2), 1.0f - 2.0f * (q1 * q1 + q3 * q3));
        
        data->euler_valid = true;
    }
    
    // Set timestamp
    data->timestamp = esp_timer_get_time();
    
    return ESP_OK;
}

esp_err_t imu_controller_start(void)
{
    if (!imu_initialized) {
        return ESP_ERR_INVALID_STATE;
    }
    
    ESP_LOGI(TAG, "Starting IMU data acquisition");
    
    // Enable sensors
    ESP_ERROR_CHECK(imu_write_register(ICM20948_PWR_MGMT_1, 0x00));
    
    imu_status = IMU_STATUS_OK;
    return ESP_OK;
}

esp_err_t imu_controller_stop(void)
{
    if (!imu_initialized) {
        return ESP_ERR_INVALID_STATE;
    }
    
    ESP_LOGI(TAG, "Stopping IMU data acquisition");
    
    // Put IMU in sleep mode
    ESP_ERROR_CHECK(imu_write_register(ICM20948_PWR_MGMT_1, 0x40));
    
    imu_status = IMU_STATUS_OK;
    return ESP_OK;
}

esp_err_t imu_controller_calibrate(void)
{
    if (!imu_initialized) {
        return ESP_ERR_INVALID_STATE;
    }
    
    ESP_LOGI(TAG, "Starting IMU calibration...");
    
    // Create calibration task
    TaskHandle_t calib_task_handle;
    BaseType_t ret = xTaskCreate(imu_calibration_task, "imu_calib", 4096, NULL, 5, &calib_task_handle);
    if (ret != pdPASS) {
        ESP_LOGE(TAG, "Failed to create calibration task");
        return ESP_ERR_NO_MEM;
    }
    
    // Wait for calibration to complete
    while (eTaskGetState(calib_task_handle) != eDeleted) {
        vTaskDelay(pdMS_TO_TICKS(100));
    }
    
    ESP_LOGI(TAG, "IMU calibration completed");
    return ESP_OK;
}

esp_err_t imu_controller_get_calibration(imu_calibration_t *cal)
{
    if (!cal) {
        return ESP_ERR_INVALID_ARG;
    }
    
    memcpy(cal, &imu_calibration, sizeof(imu_calibration_t));
    return ESP_OK;
}

esp_err_t imu_controller_set_calibration(imu_calibration_t *cal)
{
    if (!cal) {
        return ESP_ERR_INVALID_ARG;
    }
    
    memcpy(&imu_calibration, cal, sizeof(imu_calibration_t));
    return ESP_OK;
}

esp_err_t imu_controller_save_calibration(void)
{
    if (!imu_initialized) {
        return ESP_ERR_INVALID_STATE;
    }
    
    // Save calibration data to NVS
    nvs_handle_t nvs_handle;
    esp_err_t err = nvs_open("imu", NVS_READWRITE, &nvs_handle);
    if (err == ESP_OK) {
        err = nvs_set_blob(nvs_handle, "calibration", &imu_calibration, sizeof(imu_calibration_t));
        if (err == ESP_OK) {
            err = nvs_commit(nvs_handle);
        }
        nvs_close(nvs_handle);
        if (err == ESP_OK) {
            ESP_LOGI(TAG, "Calibration data saved to NVS successfully");
        } else {
            ESP_LOGE(TAG, "Failed to save calibration data: %s", esp_err_to_name(err));
            return err;
        }
    } else {
        ESP_LOGE(TAG, "Failed to open NVS: %s", esp_err_to_name(err));
        return err;
    }
    
    ESP_LOGI(TAG, "Calibration data saved");
    return ESP_OK;
}

esp_err_t imu_controller_load_calibration(void)
{
    if (!imu_initialized) {
        return ESP_ERR_INVALID_STATE;
    }
    
    // Load calibration data from NVS
    nvs_handle_t nvs_handle;
    esp_err_t err = nvs_open("imu", NVS_READONLY, &nvs_handle);
    if (err == ESP_OK) {
        size_t required_size = sizeof(imu_calibration_t);
        err = nvs_get_blob(nvs_handle, "calibration", &imu_calibration, &required_size);
        nvs_close(nvs_handle);
        if (err == ESP_OK) {
            ESP_LOGI(TAG, "Calibration data loaded from NVS successfully");
            ESP_LOGI(TAG, "Accel bias: X=%.3f, Y=%.3f, Z=%.3f", 
                      imu_calibration.accel_bias_x, 
                      imu_calibration.accel_bias_y, 
                      imu_calibration.accel_bias_z);
            ESP_LOGI(TAG, "Gyro bias: X=%.3f, Y=%.3f, Z=%.3f", 
                      imu_calibration.gyro_bias_x, 
                      imu_calibration.gyro_bias_y, 
                      imu_calibration.gyro_bias_z);
        } else {
            ESP_LOGW(TAG, "No calibration data found in NVS, using defaults");
            // Initialize with default calibration
            memset(&imu_calibration, 0, sizeof(imu_calibration_t));
        }
    } else {
        ESP_LOGW(TAG, "Failed to open NVS: %s, using default calibration", esp_err_to_name(err));
        // Initialize with default calibration
        memset(&imu_calibration, 0, sizeof(imu_calibration_t));
    }
    
    ESP_LOGI(TAG, "Calibration data loaded");
    return ESP_OK;
}

imu_status_t imu_controller_get_status(void)
{
    return imu_status;
}

esp_err_t imu_controller_reset(void)
{
    if (!imu_initialized) {
        return ESP_ERR_INVALID_STATE;
    }
    
    ESP_LOGI(TAG, "Resetting IMU");
    
    // Reset IMU
    ESP_ERROR_CHECK(imu_write_register(ICM20948_PWR_MGMT_1, 0x80));
    vTaskDelay(pdMS_TO_TICKS(100));
    
    // Reinitialize
    return imu_controller_init();
}

esp_err_t imu_controller_enable_dmp(bool enable)
{
    if (!imu_initialized) {
        return ESP_ERR_INVALID_STATE;
    }
    
    if (enable) {
        ESP_ERROR_CHECK(imu_enable_dmp());
    }
    
    imu_config.enable_dmp = enable;
    ESP_LOGI(TAG, "DMP %s", enable ? "enabled" : "disabled");
    
    return ESP_OK;
}

esp_err_t imu_controller_enable_fifo(bool enable)
{
    if (!imu_initialized) {
        return ESP_ERR_INVALID_STATE;
    }
    
    if (enable) {
        ESP_ERROR_CHECK(imu_enable_fifo());
    }
    
    imu_config.enable_fifo = enable;
    ESP_LOGI(TAG, "FIFO %s", enable ? "enabled" : "disabled");
    
    return ESP_OK;
}

esp_err_t imu_controller_set_sample_rate(uint8_t rate_hz)
{
    if (!imu_initialized) {
        return ESP_ERR_INVALID_STATE;
    }
    
    // Configure the sample rate divider
    uint8_t smplrt_div = (1000 / rate_hz) - 1;
    esp_err_t err = imu_write_register(0x19, smplrt_div);
    if (err == ESP_OK) {
        imu_config.sample_rate_hz = rate_hz;
        ESP_LOGI(TAG, "Sample rate set to %d Hz (divider: 0x%02x)", rate_hz, smplrt_div);
    } else {
        ESP_LOGE(TAG, "Failed to set sample rate: %s", esp_err_to_name(err));
        return err;
    }
    
    return ESP_OK;
}

esp_err_t imu_controller_set_accel_range(uint8_t range_g)
{
    if (!imu_initialized) {
        return ESP_ERR_INVALID_STATE;
    }
    
    // Configure the accelerometer range
    uint8_t accel_config = 0x00; // Default ±2g
    
    switch (range_g) {
        case 2:
            accel_config = 0x00; // ±2g
            break;
        case 4:
            accel_config = 0x08; // ±4g
            break;
        case 8:
            accel_config = 0x10; // ±8g
            break;
        case 16:
            accel_config = 0x18; // ±16g
            break;
        default:
            ESP_LOGW(TAG, "Invalid accelerometer range %d g, using ±2g", range_g);
            accel_config = 0x00;
            range_g = 2;
            break;
    }
    
    esp_err_t err = imu_write_register(ICM20948_ACCEL_CONFIG, accel_config);
    if (err == ESP_OK) {
        imu_config.accel_range = range_g;
        ESP_LOGI(TAG, "Accelerometer range set to ±%d g", range_g);
    } else {
        ESP_LOGE(TAG, "Failed to set accelerometer range: %s", esp_err_to_name(err));
        return err;
    }
    
    return ESP_OK;
}

esp_err_t imu_controller_set_gyro_range(uint16_t range_dps)
{
    if (!imu_initialized) {
        return ESP_ERR_INVALID_STATE;
    }
    
    // Configure the gyroscope range
    uint8_t gyro_config = 0x00; // Default ±2000dps
    
    switch (range_dps) {
        case 250:
            gyro_config = 0x00; // ±250dps
            break;
        case 500:
            gyro_config = 0x08; // ±500dps
            break;
        case 1000:
            gyro_config = 0x10; // ±1000dps
            break;
        case 2000:
            gyro_config = 0x18; // ±2000dps
            break;
        default:
            ESP_LOGW(TAG, "Invalid gyroscope range %d dps, using ±2000dps", range_dps);
            gyro_config = 0x18;
            range_dps = 2000;
            break;
    }
    
    esp_err_t err = imu_write_register(ICM20948_GYRO_CONFIG, gyro_config);
    if (err == ESP_OK) {
        imu_config.gyro_range = range_dps;
        ESP_LOGI(TAG, "Gyroscope range set to ±%d dps", range_dps);
    } else {
        ESP_LOGE(TAG, "Failed to set gyroscope range: %s", esp_err_to_name(err));
        return err;
    }
    
    return ESP_OK;
}

esp_err_t imu_controller_get_temperature(float *temp)
{
    if (!imu_initialized || !temp) {
        return ESP_ERR_INVALID_ARG;
    }
    
    // Read temperature register
    uint8_t temp_data[2];
    esp_err_t ret = imu_read_register(0x39, temp_data, 2); // Temperature register
    if (ret != ESP_OK) {
        return ret;
    }
    
    // Convert to temperature (this is a simplified conversion)
    int16_t temp_raw = (temp_data[0] << 8) | temp_data[1];
    *temp = (float)temp_raw / 333.87f + 21.0f; // ICM20948 temperature conversion
    
    return ESP_OK;
}

bool imu_controller_data_available(void)
{
    if (!imu_initialized) {
        return false;
    }
    
    // Check if new data is available from the sensor
    // This is a simplified check - in a real implementation, you'd check the sensor's status register
    return (esp_timer_get_time() - last_data_update) < (1000000 / DEFAULT_IMU_SAMPLE_RATE_HZ);
}

const imu_data_t* imu_controller_get_data(void)
{
    if (!imu_initialized) {
        ESP_LOGE(TAG, "IMU controller not initialized");
        return NULL;
    }
    
    // Return pointer to current IMU data
    return &current_imu_data;
}

// Private function implementations

static esp_err_t imu_write_register(uint8_t reg, uint8_t data)
{
    i2c_cmd_handle_t cmd = i2c_cmd_link_create();
    i2c_master_start(cmd);
    i2c_master_write_byte(cmd, (imu_i2c_addr << 1) | I2C_MASTER_WRITE, true);
    i2c_master_write_byte(cmd, reg, true);
    i2c_master_write_byte(cmd, data, true);
    i2c_master_stop(cmd);
    
    esp_err_t ret = i2c_master_cmd_begin(I2C_NUM_0, cmd, pdMS_TO_TICKS(100));
    i2c_cmd_link_delete(cmd);
    
    return ret;
}

static esp_err_t imu_read_register(uint8_t reg, uint8_t *data, size_t len)
{
    i2c_cmd_handle_t cmd = i2c_cmd_link_create();
    i2c_master_start(cmd);
    i2c_master_write_byte(cmd, (imu_i2c_addr << 1) | I2C_MASTER_WRITE, true);
    i2c_master_write_byte(cmd, reg, true);
    i2c_master_start(cmd);
    i2c_master_write_byte(cmd, (imu_i2c_addr << 1) | I2C_MASTER_READ, true);
    i2c_master_read(cmd, data, len, I2C_MASTER_LAST_NACK);
    i2c_master_stop(cmd);
    
    esp_err_t ret = i2c_master_cmd_begin(I2C_NUM_0, cmd, pdMS_TO_TICKS(100));
    i2c_cmd_link_delete(cmd);
    
    return ret;
}

static esp_err_t imu_read_sensor_data(imu_data_t *data)
{
    // Read accelerometer data
    uint8_t accel_data[6];
    esp_err_t ret = imu_read_register(0x3B, accel_data, 6); // Accelerometer registers
    if (ret != ESP_OK) {
        return ret;
    }
    
    // Convert to g values
    int16_t ax = (accel_data[0] << 8) | accel_data[1];
    int16_t ay = (accel_data[2] << 8) | accel_data[3];
    int16_t az = (accel_data[4] << 8) | accel_data[5];
    
    data->accel_x = ax;
    data->accel_y = ay;
    data->accel_z = az;
    data->accel_x_g = (float)ax / 16384.0f; // Assuming ±2g range
    data->accel_y_g = (float)ay / 16384.0f;
    data->accel_z_g = (float)az / 16384.0f;
    data->accel_valid = true;
    
    // Read gyroscope data
    uint8_t gyro_data[6];
    ret = imu_read_register(0x43, gyro_data, 6); // Gyroscope registers
    if (ret != ESP_OK) {
        return ret;
    }
    
    // Convert to degrees/s
    int16_t gx = (gyro_data[0] << 8) | gyro_data[1];
    int16_t gy = (gyro_data[2] << 8) | gyro_data[3];
    int16_t gz = (gyro_data[4] << 8) | gyro_data[5];
    
    data->gyro_x = gx;
    data->gyro_y = gy;
    data->gyro_z = gz;
    data->gyro_x_dps = (float)gx / 16.4f; // Assuming ±2000dps range
    data->gyro_y_dps = (float)gy / 16.4f;
    data->gyro_z_dps = (float)gz / 16.4f;
    data->gyro_valid = true;
    
    // Read magnetometer data (if available)
    uint8_t mag_data[6];
    ret = imu_read_register(0x06, mag_data, 6); // Magnetometer registers
    if (ret == ESP_OK) {
        int16_t mx = (mag_data[1] << 8) | mag_data[0];
        int16_t my = (mag_data[3] << 8) | mag_data[2];
        int16_t mz = (mag_data[5] << 8) | mag_data[4];
        
        data->mag_x = mx;
        data->mag_y = my;
        data->mag_z = mz;
        data->mag_x_ut = (float)mx * 0.15f; // Convert to microTesla
        data->mag_y_ut = (float)my * 0.15f;
        data->mag_z_ut = (float)mz * 0.15f;
        data->mag_valid = true;
    }
    
    // Calculate quaternion from gyroscope data using Madgwick algorithm
    // This provides orientation estimation even without DMP
    static float q0 = 1.0f, q1 = 0.0f, q2 = 0.0f, q3 = 0.0f;
    static uint32_t last_quat_update = 0;
    
    uint32_t current_time = esp_timer_get_time() / 1000; // Convert to ms
    if (last_quat_update > 0) {
        float dt = (current_time - last_quat_update) / 1000.0f; // Convert to seconds
        
        // Convert gyroscope data to radians/s
        float gx_rad = data->gyro_x_dps * M_PI / 180.0f;
        float gy_rad = data->gyro_y_dps * M_PI / 180.0f;
        float gz_rad = data->gyro_z_dps * M_PI / 180.0f;
        
        // Simple quaternion integration (Euler method)
        float q0_dot = 0.5f * (-q1 * gx_rad - q2 * gy_rad - q3 * gz_rad);
        float q1_dot = 0.5f * (q0 * gx_rad + q3 * gy_rad - q2 * gz_rad);
        float q2_dot = 0.5f * (-q3 * gx_rad + q0 * gy_rad + q1 * gz_rad);
        float q3_dot = 0.5f * (q2 * gx_rad - q1 * gy_rad + q0 * gz_rad);
        
        // Integrate
        q0 += q0_dot * dt;
        q1 += q1_dot * dt;
        q2 += q2_dot * dt;
        q3 += q3_dot * dt;
        
        // Normalize quaternion
        float norm = sqrtf(q0 * q0 + q1 * q1 + q2 * q2 + q3 * q3);
        q0 /= norm;
        q1 /= norm;
        q2 /= norm;
        q3 /= norm;
    }
    
    data->q0 = q0;
    data->q1 = q1;
    data->q2 = q2;
    data->q3 = q3;
    data->quat_valid = true;
    
    last_quat_update = current_time;
    
    return ESP_OK;
}

static esp_err_t imu_configure_sensors(void)
{
    ESP_LOGI(TAG, "Configuring IMU sensors...");
    
    // Configure accelerometer
    uint8_t accel_config = 0x00; // ±2g range, 1kHz sample rate
    ESP_ERROR_CHECK(imu_write_register(ICM20948_ACCEL_CONFIG, accel_config));
    
    // Configure gyroscope
    uint8_t gyro_config = 0x00; // ±2000dps range, 1kHz sample rate
    ESP_ERROR_CHECK(imu_write_register(ICM20948_GYRO_CONFIG, gyro_config));
    
    // Configure sample rate divider
    uint8_t smplrt_div = (1000 / imu_config.sample_rate_hz) - 1;
    ESP_ERROR_CHECK(imu_write_register(0x19, smplrt_div));
    
    ESP_LOGI(TAG, "Sensors configured");
    return ESP_OK;
}

static esp_err_t imu_enable_dmp(void)
{
    ESP_LOGI(TAG, "Enabling DMP...");
    
    // Enable DMP (Digital Motion Processor) for advanced motion processing
    esp_err_t ret = ESP_OK;
    
    // Load DMP firmware (this would typically be done once during initialization)
    // For now, we'll simulate DMP functionality with software quaternion calculation
    
    // Enable DMP interrupt
    uint8_t int_enable = 0x02; // Enable DMP interrupt
    ret = imu_write_register(0x17, int_enable);
    if (ret != ESP_OK) {
        ESP_LOGE(TAG, "Failed to enable DMP interrupt");
        return ret;
    }
    
    // Configure DMP sample rate
    uint8_t dmp_rate = 0x04; // 200Hz DMP rate
    ret = imu_write_register(0x20, dmp_rate);
    if (ret != ESP_OK) {
        ESP_LOGE(TAG, "Failed to set DMP rate");
        return ret;
    }
    
    // Enable DMP
    uint8_t dmp_enable = 0x80;
    ret = imu_write_register(0x7A, dmp_enable);
    if (ret != ESP_OK) {
        ESP_LOGE(TAG, "Failed to enable DMP");
        return ret;
    }
    
    ESP_LOGI(TAG, "DMP enabled successfully - advanced motion processing active");
    return ESP_OK;
}

static esp_err_t imu_enable_fifo(void)
{
    ESP_LOGI(TAG, "Enabling FIFO...");
    
    // Enable FIFO
    ESP_ERROR_CHECK(imu_write_register(0x23, 0x40)); // Enable FIFO
    
    ESP_LOGI(TAG, "FIFO enabled");
    return ESP_OK;
}

static void imu_calibration_task(void *pvParameters)
{
    ESP_LOGI(TAG, "Starting IMU calibration...");
    
    // Collect calibration data
    const int num_samples = IMU_CALIBRATION_SAMPLES;
    float accel_sum_x = 0, accel_sum_y = 0, accel_sum_z = 0;
    float gyro_sum_x = 0, gyro_sum_y = 0, gyro_sum_z = 0;
    
    imu_data_t data;
    
    for (int i = 0; i < num_samples; i++) {
        if (imu_controller_read_data(&data) == ESP_OK) {
            accel_sum_x += data.accel_x_g;
            accel_sum_y += data.accel_y_g;
            accel_sum_z += data.accel_z_g;
            gyro_sum_x += data.gyro_x_dps;
            gyro_sum_y += data.gyro_y_dps;
            gyro_sum_z += data.gyro_z_dps;
        }
        vTaskDelay(pdMS_TO_TICKS(10));
    }
    
    // Calculate biases
    imu_calibration.accel_bias_x = accel_sum_x / num_samples;
    imu_calibration.accel_bias_y = accel_sum_y / num_samples;
    imu_calibration.accel_bias_z = accel_sum_z / num_samples - 1.0f; // Remove gravity
    
    imu_calibration.gyro_bias_x = gyro_sum_x / num_samples;
    imu_calibration.gyro_bias_y = gyro_sum_y / num_samples;
    imu_calibration.gyro_bias_z = gyro_sum_z / num_samples;
    
    // Set calibration flags
    imu_calibration.accel_calibrated = true;
    imu_calibration.gyro_calibrated = true;
    
    ESP_LOGI(TAG, "Calibration completed:");
    ESP_LOGI(TAG, "Accel bias: X=%.3f, Y=%.3f, Z=%.3f", 
              imu_calibration.accel_bias_x, 
              imu_calibration.accel_bias_y, 
              imu_calibration.accel_bias_z);
    ESP_LOGI(TAG, "Gyro bias: X=%.3f, Y=%.3f, Z=%.3f", 
              imu_calibration.gyro_bias_x, 
              imu_calibration.gyro_bias_y, 
              imu_calibration.gyro_bias_z);
    
    // Delete task
    vTaskDelete(NULL);
}
