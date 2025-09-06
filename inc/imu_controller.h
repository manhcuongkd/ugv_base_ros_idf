#ifndef IMU_CONTROLLER_H
#define IMU_CONTROLLER_H

#include <stdint.h>
#include <stdbool.h>
#include <esp_err.h>
#include "ugv_config.h"

#ifdef __cplusplus
extern "C" {
#endif

// IMU data structure
typedef struct {
    // Raw sensor data
    int16_t accel_x;       // Raw accelerometer X
    int16_t accel_y;       // Raw accelerometer Y
    int16_t accel_z;       // Raw accelerometer Z
    int16_t gyro_x;        // Raw gyroscope X
    int16_t gyro_y;        // Raw gyroscope Y
    int16_t gyro_z;        // Raw gyroscope Z
    int16_t mag_x;         // Raw magnetometer X
    int16_t mag_y;         // Raw magnetometer Y
    int16_t mag_z;         // Raw magnetometer Z
    
    // Processed data
    float accel_x_g;       // Accelerometer X in g
    float accel_y_g;       // Accelerometer Y in g
    float accel_z_g;       // Accelerometer Z in g
    float gyro_x_dps;      // Gyroscope X in degrees/s
    float gyro_y_dps;      // Gyroscope Y in degrees/s
    float gyro_z_dps;      // Gyroscope Z in degrees/s
    float mag_x_ut;        // Magnetometer X in microTesla
    float mag_y_ut;        // Magnetometer Y in microTesla
    float mag_z_ut;        // Magnetometer Z in microTesla
    
    // Euler angles (radians)
    float roll;            // Roll angle (X-axis rotation)
    float pitch;           // Pitch angle (Y-axis rotation)
    float yaw;             // Yaw angle (Z-axis rotation)
    
    // Quaternion
    float q0;              // Quaternion w component
    float q1;              // Quaternion x component
    float q2;              // Quaternion y component
    float q3;              // Quaternion z component
    
    // Temperature
    float temperature;     // Temperature in Celsius
    
    // Timestamp
    uint64_t timestamp;    // Timestamp in microseconds
    
    // Data validity flags
    bool accel_valid;      // Accelerometer data valid
    bool gyro_valid;       // Gyroscope data valid
    bool mag_valid;        // Magnetometer data valid
    bool quat_valid;       // Quaternion data valid
    bool euler_valid;      // Euler angles valid
} imu_data_t;

// IMU calibration data
typedef struct {
    // Accelerometer bias
    float accel_bias_x;    // Accelerometer X bias
    float accel_bias_y;    // Accelerometer Y bias
    float accel_bias_z;    // Accelerometer Z bias
    
    // Gyroscope bias
    float gyro_bias_x;     // Gyroscope X bias
    float gyro_bias_y;     // Gyroscope Y bias
    float gyro_bias_z;     // Gyroscope Z bias
    
    // Magnetometer bias and scale
    float mag_bias_x;      // Magnetometer X bias
    float mag_bias_y;      // Magnetometer Y bias
    float mag_bias_z;      // Magnetometer Z bias
    float mag_scale_x;     // Magnetometer X scale
    float mag_scale_y;     // Magnetometer Y scale
    float mag_scale_z;     // Magnetometer Z scale
    
    // Calibration status
    bool accel_calibrated; // Accelerometer calibrated
    bool gyro_calibrated;  // Gyroscope calibrated
    bool mag_calibrated;   // Magnetometer calibrated
} imu_calibration_t;

// IMU configuration
typedef struct {
    uint8_t sample_rate_hz;        // Sample rate in Hz
    uint8_t accel_range;           // Accelerometer range (2, 4, 8, 16 g)
    uint16_t gyro_range;           // Gyroscope range (250, 500, 1000, 2000 dps)
    uint8_t mag_rate;              // Magnetometer rate
    bool enable_dmp;               // Enable DMP
    bool enable_fifo;              // Enable FIFO
    bool low_power_mode;           // Low power mode
} imu_config_t;

// IMU status
typedef enum {
    IMU_STATUS_OK = 0,             // IMU working normally
    IMU_STATUS_INIT_ERROR,         // Initialization error
    IMU_STATUS_COMM_ERROR,         // Communication error
    IMU_STATUS_CALIBRATION_ERROR,  // Calibration error
    IMU_STATUS_DATA_ERROR,         // Data error
    IMU_STATUS_TIMEOUT_ERROR       // Timeout error
} imu_status_t;

// Function prototypes

/**
 * @brief Initialize the IMU controller
 * @return ESP_OK on success, error code otherwise
 */
esp_err_t imu_controller_init(void);

/**
 * @brief Configure IMU settings
 * @param config Pointer to IMU configuration
 * @return ESP_OK on success, error code otherwise
 */
esp_err_t imu_controller_configure(imu_config_t *config);

/**
 * @brief Read IMU data
 * @param data Pointer to IMU data structure
 * @return ESP_OK on success, error code otherwise
 */
esp_err_t imu_controller_read_data(imu_data_t *data);

/**
 * @brief Start IMU data acquisition
 * @return ESP_OK on success, error code otherwise
 */
esp_err_t imu_controller_start(void);

/**
 * @brief Stop IMU data acquisition
 * @return ESP_OK on success, error code otherwise
 */
esp_err_t imu_controller_stop(void);

/**
 * @brief Calibrate IMU sensors
 * @return ESP_OK on success, error code otherwise
 */
esp_err_t imu_controller_calibrate(void);

/**
 * @brief Get calibration data
 * @param cal Pointer to calibration data structure
 * @return ESP_OK on success, error code otherwise
 */
esp_err_t imu_controller_get_calibration(imu_calibration_t *cal);

/**
 * @brief Set calibration data
 * @param cal Pointer to calibration data structure
 * @return ESP_OK on success, error code otherwise
 */
esp_err_t imu_controller_set_calibration(imu_calibration_t *cal);

/**
 * @brief Save calibration data to NVS
 * @return ESP_OK on success, error code otherwise
 */
esp_err_t imu_controller_save_calibration(void);

/**
 * @brief Load calibration data from NVS
 * @return ESP_OK on success, error code otherwise
 */
esp_err_t imu_controller_load_calibration(void);

/**
 * @brief Get IMU status
 * @return IMU status
 */
imu_status_t imu_controller_get_status(void);

/**
 * @brief Reset IMU
 * @return ESP_OK on success, error code otherwise
 */
esp_err_t imu_controller_reset(void);

/**
 * @brief Enable/disable DMP
 * @param enable True to enable, false to disable
 * @return ESP_OK on success, error code otherwise
 */
esp_err_t imu_controller_enable_dmp(bool enable);

/**
 * @brief Enable/disable FIFO
 * @param enable True to enable, false to disable
 * @return ESP_OK on success, error code otherwise
 */
esp_err_t imu_controller_enable_fifo(bool enable);

/**
 * @brief Set sample rate
 * @param rate_hz Sample rate in Hz
 * @return ESP_OK on success, error code otherwise
 */
esp_err_t imu_controller_set_sample_rate(uint8_t rate_hz);

/**
 * @brief Set accelerometer range
 * @param range_g Range in g (2, 4, 8, 16)
 * @return ESP_OK on success, error code otherwise
 */
esp_err_t imu_controller_set_accel_range(uint8_t range_g);

/**
 * @brief Set gyroscope range
 * @param range_dps Range in degrees/s (250, 500, 1000, 2000)
 * @return ESP_OK on success, error code otherwise
 */
esp_err_t imu_controller_set_gyro_range(uint8_t range_dps);

/**
 * @brief Get temperature
 * @param temp Pointer to temperature value
 * @return ESP_OK on success, error code otherwise
 */
esp_err_t imu_controller_get_temperature(float *temp);

/**
 * @brief Check if new data is available
 * @return True if new data available, false otherwise
 */
bool imu_controller_data_available(void);

/**
 * @brief Get current IMU data
 * @return Pointer to current IMU data structure
 */
const imu_data_t* imu_controller_get_data(void);

/**
 * @brief Set IMU offset values
 * @param gx Gyroscope X offset
 * @param gy Gyroscope Y offset
 * @param gz Gyroscope Z offset
 * @param ax Accelerometer X offset
 * @param ay Accelerometer Y offset
 * @param az Accelerometer Z offset
 * @param cx Magnetometer X offset
 * @param cy Magnetometer Y offset
 * @param cz Magnetometer Z offset
 * @return ESP_OK on success, error code otherwise
 */
esp_err_t imu_controller_set_offset(float gx, float gy, float gz, 
                                   float ax, float ay, float az,
                                   float cx, float cy, float cz);

/**
 * @brief Get IMU offset values
 * @param gx Pointer to gyroscope X offset
 * @param gy Pointer to gyroscope Y offset
 * @param gz Pointer to gyroscope Z offset
 * @param ax Pointer to accelerometer X offset
 * @param ay Pointer to accelerometer Y offset
 * @param az Pointer to accelerometer Z offset
 * @param cx Pointer to magnetometer X offset
 * @param cy Pointer to magnetometer Y offset
 * @param cz Pointer to magnetometer Z offset
 * @return ESP_OK on success, error code otherwise
 */
esp_err_t imu_controller_get_offset(float *gx, float *gy, float *gz,
                                   float *ax, float *ay, float *az,
                                   float *cx, float *cy, float *cz);

/**
 * @brief Get current IMU data and log it
 * @return ESP_OK on success, error code otherwise
 */
esp_err_t imu_controller_get_data_log(void);

// Default IMU Configuration
#define DEFAULT_IMU_SAMPLE_RATE_HZ  100
#define DEFAULT_IMU_ACCEL_RANGE     2      // ±2g
#define DEFAULT_IMU_GYRO_RANGE      2000U  // ±2000dps
#define DEFAULT_IMU_MAG_RATE        10     // 10Hz
#define DEFAULT_IMU_ENABLE_DMP      true
#define DEFAULT_IMU_ENABLE_FIFO     true
#define DEFAULT_IMU_LOW_POWER       false

// IMU I2C address
#define ICM20948_I2C_ADDR          0x68

// Register addresses (simplified)
#define ICM20948_WHO_AM_I          0x00
#define ICM20948_PWR_MGMT_1        0x06
#define ICM20948_CONFIG            0x1A
#define ICM20948_GYRO_CONFIG       0x1B
#define ICM20948_ACCEL_CONFIG      0x1C
#define ICM20948_ACCEL_CONFIG2     0x1D

#ifdef __cplusplus
}
#endif

#endif // IMU_CONTROLLER_H
