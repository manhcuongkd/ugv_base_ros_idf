#include <stdio.h>
#include <string.h>
#include <freertos/FreeRTOS.h>
#include <freertos/task.h>
#include <freertos/queue.h>
#include <esp_system.h>
#include <nvs_flash.h>
#include <esp_log.h>
#include <driver/uart.h>
#include <driver/gpio.h>
#include <esp_wifi.h>
#include <esp_event.h>
#include <esp_http_server.h>
#include <esp_spiffs.h>
#include <esp_timer.h>
#include <cJSON.h>

// Include our custom headers
#include "../inc/ugv_config.h"
#include "../inc/json_parser.h"
#include "../inc/uart_controller.h"
#include "../inc/oled_controller.h"
#include "../inc/imu_controller.h"
#include "../inc/gimbal_controller.h"
#include "../inc/motion_module.h"
#include "../inc/mission_system.h"
#include "../inc/ugv_advanced.h"
#include "../inc/module_handlers.h"
#include "../inc/system_manager.h"

static const char *TAG = "Main";

// Helper function to wait for system initialization (eliminates redundancy)
static void wait_for_system_initialization(void)
{
    while (!system_manager_is_initialized()) {
        vTaskDelay(pdMS_TO_TICKS(100));
    }
}

// Global variables
static QueueHandle_t json_cmd_queue;
static TaskHandle_t main_task_handle;
static TaskHandle_t imu_task_handle;
static TaskHandle_t motion_task_handle;
static TaskHandle_t uart_task_handle;

// Configuration
ugv_config_t ugv_config = {
    .main_type = 1,        // RaspRover
    .module_type = 2,      // Gimbal
    .info_print = 1,       // Debug info
    .esp_now_mode = 3,     // Follower mode
    .ctrl_by_broadcast = true,
    .steady_mode = false,
    .base_feedback_flow = true,
    .eem_mode = 0          // Default EEM mode
};

// Task function declarations
static void main_task(void *pvParameters);
static void imu_task(void *pvParameters);
static void motion_task(void *pvParameters);
static void uart_task(void *pvParameters);

// Gimbal testing function declarations
static void test_gimbal_initialization(void);
static void test_gimbal_basic_movement(void);
static void test_gimbal_center_position(void);
static void test_gimbal_limits(void);
static void test_gimbal_smooth_movement(void);
static void test_gimbal_feedback(void);
static void test_gimbal_stabilization(void);
static void run_gimbal_tests(void);

extern "C" void app_main(void)
{
    ESP_LOGI(TAG, "Starting RaspRover IDF Application");
    
    // Initialize NVS
    esp_err_t ret = nvs_flash_init();
    if (ret == ESP_ERR_NVS_NO_FREE_PAGES || ret == ESP_ERR_NVS_NEW_VERSION_FOUND) {
        ESP_ERROR_CHECK(nvs_flash_erase());
        ret = nvs_flash_init();
    }
    ESP_ERROR_CHECK(ret);
    
    // Initialize system
    ESP_ERROR_CHECK(system_manager_init());
    
    // Create queues
    json_cmd_queue = xQueueCreate(10, sizeof(json_command_t));
    if (json_cmd_queue == NULL) {
        ESP_LOGE(TAG, "Failed to create JSON command queue");
        return;
    }
    
    // Create tasks
    xTaskCreatePinnedToCore(main_task, "main_task", 32768, NULL, 5, &main_task_handle, 0);
    xTaskCreatePinnedToCore(imu_task, "imu_task", 6144, NULL, 4, &imu_task_handle, 1);
    // xTaskCreatePinnedToCore(motion_task, "motion_task", 6144, NULL, 3, &motion_task_handle, 0); // Commented out for cleaner gimbal testing logs
    xTaskCreatePinnedToCore(uart_task, "uart_task", 8192, NULL, 4, &uart_task_handle, 0);
    
    ESP_LOGI(TAG, "RaspRover application started successfully");
}

// Main application task
static void main_task(void *pvParameters)
{
    ESP_LOGI(TAG, "Main task started");
    
    // Wait for system initialization
    wait_for_system_initialization();
    
    // Check stack usage after initialization
    UBaseType_t stack_remaining = uxTaskGetStackHighWaterMark(NULL);
    ESP_LOGI(TAG, "Main task stack remaining after init: %u bytes", stack_remaining * sizeof(StackType_t));
    
    // Display startup message
    oled_controller_display_text(0, "RaspRover IDF");
    oled_controller_display_text(1, "Version: 1.0.0");
    oled_controller_display_text(2, "Starting...");
    oled_controller_display_text(3, "");
    oled_controller_update();
    
    vTaskDelay(pdMS_TO_TICKS(1200));
    
    // Switch back to default mode for periodic updates
    oled_controller_reset_to_default();
    
//     // Force gimbal mode for testing
//     ESP_LOGI(TAG, "🔧 FORCING GIMBAL MODE FOR TESTING");
//     ugv_config.module_type = 2;
    
//     // Modern gimbal API-based diagnostic
//     ESP_LOGI(TAG, "🔧 GIMBAL API DIAGNOSTIC");
//     ESP_LOGI(TAG, "========================================");
//     ESP_LOGI(TAG, "📋 Configuration:");
//     ESP_LOGI(TAG, "   Communication: SCServo protocol via system manager");
//     ESP_LOGI(TAG, "   Expected Servos: ID 1 (Tilt), ID 2 (Pan)");
//     ESP_LOGI(TAG, "   Protocol: SMS_STS with corrected pan direction");
//     ESP_LOGI(TAG, "   Testing Method: Gimbal controller API");
//     ESP_LOGI(TAG, "");
//     ESP_LOGI(TAG, "");
//     ESP_LOGI(TAG, "🎮 GIMBAL API TESTING:");
//     ESP_LOGI(TAG, "   Using gimbal_controller API instead of bare UART...");
//     ESP_LOGI(TAG, "");
    
//     // Test gimbal API with corrected pan direction
//     ESP_LOGI(TAG, "⚡ Waiting for system initialization...");
//     vTaskDelay(pdMS_TO_TICKS(1000)); // Allow system manager to complete initialization
    
//     // Test servo feedback using gimbal API
//     ESP_LOGI(TAG, "🔍 GIMBAL SERVO DETECTION using API...");
    
//     gimbal_feedback_t pan_feedback, tilt_feedback;
//     esp_err_t ret = gimbal_controller_get_feedback(&pan_feedback, &tilt_feedback);
    
//     bool servos_detected = false;
//     if (ret == ESP_OK) {
//         ESP_LOGI(TAG, "   ✅ Gimbal feedback API successful!");
//         ESP_LOGI(TAG, "   📊 Pan servo (ID 2): status=%s, pos=%d, voltage=%.1fV, temp=%.1f°C", 
//                  pan_feedback.status ? "OK" : "ERROR", pan_feedback.pos, 
//                  pan_feedback.voltage, pan_feedback.temper);
//         ESP_LOGI(TAG, "   📊 Tilt servo (ID 1): status=%s, pos=%d, voltage=%.1fV, temp=%.1f°C", 
//                  tilt_feedback.status ? "OK" : "ERROR", tilt_feedback.pos, 
//                  tilt_feedback.voltage, tilt_feedback.temper);
        
//         if (pan_feedback.status && tilt_feedback.status) {
//             servos_detected = true;
//             ESP_LOGI(TAG, "   🎉 BOTH GIMBAL SERVOS DETECTED via API!");
//         } else if (pan_feedback.status || tilt_feedback.status) {
//             ESP_LOGI(TAG, "   ⚠️ Partial servo detection - one servo may have issues");
//             servos_detected = true; // Still attempt movement
//         } else {
//             ESP_LOGI(TAG, "   ❌ No servo status detected - may be communication issue");
//         }
//     } else {
//         ESP_LOGI(TAG, "   ❌ Gimbal feedback API failed: %s", esp_err_to_name(ret));
//         ESP_LOGI(TAG, "   🔄 Continuing with movement test anyway...");
//         servos_detected = true; // Still attempt movement for testing
//     }
    
//     ESP_LOGI(TAG, "");
//     ESP_LOGI(TAG, "🎮 GIMBAL API MOVEMENT TEST - CORRECTED PAN DIRECTION:");
//     ESP_LOGI(TAG, "   Testing with gimbal_controller API instead of bare UART...");
//     ESP_LOGI(TAG, "");
    
//     if (servos_detected) {
//         // Test 1: Center the gimbal
//         ESP_LOGI(TAG, "   🎯 Step 1: Center gimbal using API...");
//         ret = gimbal_controller_center();
//         if (ret == ESP_OK) {
//             ESP_LOGI(TAG, "   ✅ Gimbal centered successfully!");
//         } else {
//             ESP_LOGE(TAG, "   ❌ Failed to center gimbal: %s", esp_err_to_name(ret));
//         }
//         vTaskDelay(pdMS_TO_TICKS(2000));
        
//         // Test 2: Set speed and acceleration
//         ESP_LOGI(TAG, "   ⚡ Step 2: Set movement speed and acceleration...");
//         ret = gimbal_controller_set_speed(3.0f, 3.0f);  // Pan speed, Tilt speed
//         if (ret == ESP_OK) {
//             ESP_LOGI(TAG, "   ✅ Speed set to 3.0 for both axes");
//         }
        
//         ret = gimbal_controller_set_acceleration(2.0f, 2.0f);  // Pan acc, Tilt acc
//         if (ret == ESP_OK) {
//             ESP_LOGI(TAG, "   ✅ Acceleration set to 2.0 for both axes");
//         }
//         vTaskDelay(pdMS_TO_TICKS(500));
        
//         // Test 3: Movement sequence using angle-based API (with corrected pan direction)
//         ESP_LOGI(TAG, "");
//         ESP_LOGI(TAG, "   🎬 CORRECTED PAN DIRECTION MOVEMENT SEQUENCE:");
//         ESP_LOGI(TAG, "   Movement: center -> up -> left -> center -> down -> right");
//         ESP_LOGI(TAG, "");
        
//         struct {
//             float pan_angle;
//             float tilt_angle;
//             const char* description;
//         } movements[] = {
//             {0.0f, 0.0f, "CENTER position (start)"},
//             {0.0f, -20.0f, "UP position (tilt up)"},
//             {-30.0f, 0.0f, "LEFT position (pan left) - CORRECTED"},
//             {0.0f, 0.0f, "CENTER position (return)"},
//             {0.0f, 20.0f, "DOWN position (tilt down)"},
//             {30.0f, 0.0f, "RIGHT position (pan right) - CORRECTED"}
//         };
        
//         for (int i = 0; i < 6; i++) {
//             ESP_LOGI(TAG, "   🎯 %s: Pan=%.1f°, Tilt=%.1f°", 
//                      movements[i].description, movements[i].pan_angle, movements[i].tilt_angle);
            
//             ret = gimbal_controller_set_pan_tilt(movements[i].pan_angle, movements[i].tilt_angle);
//             if (ret == ESP_OK) {
//                 ESP_LOGI(TAG, "         ✅ Movement command sent successfully");
//             } else {
//                 ESP_LOGE(TAG, "         ❌ Movement command failed: %s", esp_err_to_name(ret));
//             }
            
//             // Wait for movement to complete
//             vTaskDelay(pdMS_TO_TICKS(2000));
            
//             // Optional: Read current position
//             uint16_t current_pan, current_tilt;
//             if (gimbal_controller_get_position(&current_pan, &current_tilt) == ESP_OK) {
//                 ESP_LOGI(TAG, "         📍 Current position: Pan=%d, Tilt=%d", current_pan, current_tilt);
//             }
//         }
        
//         ESP_LOGI(TAG, "");
//         ESP_LOGI(TAG, "   ✅ GIMBAL API MOVEMENT TEST COMPLETE!");
//         ESP_LOGI(TAG, "   🎉 Pan direction correction verified!");
        
//         // Test 4: Final status check
//         ESP_LOGI(TAG, "");
//         ESP_LOGI(TAG, "   🔍 FINAL GIMBAL STATUS CHECK:");
        
//         gimbal_control_t gimbal_status;
//         ret = gimbal_controller_get_status(&gimbal_status);
//         if (ret == ESP_OK) {
//             ESP_LOGI(TAG, "   Mode: %d, Pan: %d, Tilt: %d", 
//                      gimbal_status.mode, gimbal_status.pan_position, gimbal_status.tilt_position);
//             ESP_LOGI(TAG, "   Speed: Pan=%.1f, Tilt=%.1f", 
//                      gimbal_status.pan_speed, gimbal_status.tilt_speed);
//             ESP_LOGI(TAG, "   Calibrated: %s", gimbal_status.calibrated ? "Yes" : "No");
//         }
        
//         ESP_LOGI(TAG, "");
//         ESP_LOGI(TAG, "✅ GIMBAL API TEST SEQUENCE COMPLETED!");
//         ESP_LOGI(TAG, "");
//         ESP_LOGI(TAG, "📊 Summary:");
//         ESP_LOGI(TAG, "   - Servo detection: %s", servos_detected ? "SUCCESS" : "PARTIAL/FAILED");
//         ESP_LOGI(TAG, "   - Movement test: COMPLETED with corrected pan direction");
//         ESP_LOGI(TAG, "   - API functionality: VERIFIED");
        
//     } else {
//         ESP_LOGI(TAG, "   ⚠️ Skipping movement test - servo detection issues");
//     }
    
//     // Main application loop - FreeRTOS tasks should never return
//     ESP_LOGI(TAG, "");
//     ESP_LOGI(TAG, "🔄 Entering main application loop...");
    
//     while (1) {
//         // Main application loop - keep the task running
//         vTaskDelay(pdMS_TO_TICKS(5000)); // 5 second loop
        
//         // Optionally log status periodically
//         ESP_LOGI(TAG, "📡 RaspRover running - gimbal test completed");
        
//         // You can add periodic tasks here:
//         // - Check system status
//         // - Handle commands
//         // - Update OLED display
//         // etc.
//     }
// }
// // Additional gimbal test functions

// static void gimbal_test_task(void *pvParameters)
// {
//     ESP_LOGI(TAG, "Gimbal test task starting...");
    
//     // Let the system settle
//     vTaskDelay(pdMS_TO_TICKS(3000));
    
//     gimbal_feedback_t pan_fb, tilt_fb;
//     esp_err_t ret = gimbal_controller_get_feedback(&pan_fb, &tilt_fb);
//     if (ret == ESP_OK) {
//         ESP_LOGI(TAG, "✓ Gimbal feedback obtained successfully");
//         ESP_LOGI(TAG, "Pan servo - Status: %s, Position: %d, Voltage: %.1fV", 
//                  pan_fb.status ? "OK" : "ERROR", pan_fb.pos, pan_fb.voltage);
//         ESP_LOGI(TAG, "Tilt servo - Status: %s, Position: %d, Voltage: %.1fV", 
//                  tilt_fb.status ? "OK" : "ERROR", tilt_fb.pos, tilt_fb.voltage);
//     } else {
//         ESP_LOGW(TAG, "⚠ Failed to read servo feedback (may not be available): %s", esp_err_to_name(ret));
//     }
    
//     // Test torque control
//     ESP_LOGI(TAG, "Testing torque control...");
//     ret = gimbal_controller_set_torque(GIMBAL_PAN_ID, false);
//     if (ret == ESP_OK) {
//         ESP_LOGI(TAG, "✓ Pan servo torque disabled");
//         vTaskDelay(pdMS_TO_TICKS(1000));
        
//         ret = gimbal_controller_set_torque(GIMBAL_PAN_ID, true);
//         if (ret == ESP_OK) {
//             ESP_LOGI(TAG, "✓ Pan servo torque enabled");
//         }
//     } else {
//         ESP_LOGW(TAG, "⚠ Torque control may not be available: %s", esp_err_to_name(ret));
// }
    
//     // Main application loop
//     int gimbal_test_counter = 0;
//     while (1) {
//         // JSON commands are now processed directly in uart_controller
        
//         // Update system status
//         if (ugv_config.base_feedback_flow) {
//             // Send base feedback
//             base_info_feedback();
//         }
        
//         // Heartbeat control
//         heart_beat_ctrl();
        
//         // Update OLED display (Arduino-style periodic update)
//         oled_controller_info_update();
        
//         // Periodic gimbal test (every 5 seconds) if in gimbal mode
//         if (ugv_config.module_type == 2) {
//             gimbal_test_counter++;
//             if (gimbal_test_counter >= 5) { // Every 5 seconds
//                 ESP_LOGI(TAG, "🔧 MANUAL GIMBAL TEST - Attempting to center gimbal...");
//                 esp_err_t ret = gimbal_controller_center();
//                 ESP_LOGI(TAG, "🔧 Gimbal center result: %s", esp_err_to_name(ret));
                
//                 // Try direct position control
//                 vTaskDelay(pdMS_TO_TICKS(1000));
//                 ESP_LOGI(TAG, "🔧 MANUAL GIMBAL TEST - Setting position 2047,2047...");
//                 ret = gimbal_controller_set_position(2047, 2047);
//                 ESP_LOGI(TAG, "🔧 Gimbal position result: %s", esp_err_to_name(ret));
                
//                 gimbal_test_counter = 0;
//             }
//         }
        
//         vTaskDelay(pdMS_TO_TICKS(1000)); // Standard main task frequency
//     }
// }

// IMU task
static void imu_task(void *pvParameters)
{
    ESP_LOGI(TAG, "IMU task started");
    
    wait_for_system_initialization();
    
    // IMU calibration
    ESP_LOGI(TAG, "Starting IMU calibration...");
    oled_controller_display_text(3, "IMU Calibrating...");
    oled_controller_update();
    
    imu_controller_calibrate();
    
    oled_controller_display_text(3, "IMU Ready");
    oled_controller_update();
    
    // IMU data processing loop
    while (1) {
        imu_data_t imu_data;
        if (imu_controller_read_data(&imu_data) == ESP_OK) {
            // Process IMU data
            if (ugv_config.module_type == 2) { // Gimbal mode
                // Apply gimbal stabilization (only if gimbal is initialized)
                // Note: Temporarily disabled to prevent log spam affecting motion control
                // gimbal_controller_stabilize(&imu_data);
            }
        }
        
        vTaskDelay(pdMS_TO_TICKS(10)); // 100Hz IMU update rate
    }
}

// Motion task
static void motion_task(void *pvParameters)
{
    ESP_LOGI(TAG, "Motion task started");
    
    wait_for_system_initialization();
    
    // Initialize encoders
    motion_module_init_encoders();
    
    // Initialize PID controllers
    motion_module_init_pid();
    
    // Motion control loop (matching Arduino loop sequence)
    while (1) {
        if (!system_manager_is_emergency_stop()) {
            // Update encoder readings
            motion_module_update_encoders();
            
            // Get wheel speeds (matching Arduino getLeftSpeed/getRightSpeed)
            motion_module_get_left_speed();
            motion_module_get_right_speed();
            
            // Compute PID control (matching Arduino LeftPidControllerCompute/RightPidControllerCompute)
            motion_module_left_pid_compute();
            motion_module_right_pid_compute();
        }
        
        vTaskDelay(pdMS_TO_TICKS(20)); // 50Hz motion control rate
    }
}

// UART task
static void uart_task(void *pvParameters)
{
    ESP_LOGI(TAG, "UART task started");
    
    // Wait for system initialization (UART already configured by system manager)
    wait_for_system_initialization();
    
    ESP_LOGI(TAG, "UART initialized, starting command processing...");
    
    // UART buffer for accumulating JSON commands (like Arduino implementation)
    static char uart_buffer[2048];
    static size_t buffer_pos = 0;
    static uint32_t last_char_time = 0;
    const uint32_t JSON_TIMEOUT_MS = 100; // 100ms timeout for JSON completion
    
    char* temp_data = (char*) malloc(1024);
    
    // UART command processing loop
    while (1) {
        // Read data from UART
        int len = uart_read_bytes(UART_NUM_0, temp_data, 1023, pdMS_TO_TICKS(10));
        if (len > 0) {
            temp_data[len] = '\0';
            last_char_time = esp_timer_get_time() / 1000; // Convert to milliseconds
            
            // Append to buffer
            for (int i = 0; i < len && buffer_pos < sizeof(uart_buffer) - 1; i++) {
                char c = temp_data[i];
                uart_buffer[buffer_pos++] = c;
                
                // Check for end of JSON command (newline character)
                if (c == '\n') {
                    uart_buffer[buffer_pos] = '\0';
                    ESP_LOGI(TAG, "Complete JSON command received (%d bytes): %s", buffer_pos, uart_buffer);
                    
                    // Process the command using uart_controller
                    esp_err_t result = uart_controller_parse_command(uart_buffer, buffer_pos);
                    if (result != ESP_OK) {
                        ESP_LOGE(TAG, "Failed to process command: %s", esp_err_to_name(result));
                    }
                    
                    // Reset buffer for next command
                    buffer_pos = 0;
                }
            }
        }
        
        // Handle timeout for incomplete JSON (like Arduino implementation)
        uint32_t current_time = esp_timer_get_time() / 1000;
        if (buffer_pos > 0 && (current_time - last_char_time > JSON_TIMEOUT_MS)) {
            ESP_LOGW(TAG, "JSON timeout - clearing buffer (%d bytes): %.*s", buffer_pos, buffer_pos, uart_buffer);
            buffer_pos = 0;
        }
        
        vTaskDelay(pdMS_TO_TICKS(10)); // 100Hz UART processing rate
    }
    
    free(temp_data);
}

// Gimbal Testing Functions
static void test_gimbal_initialization(void)
{
    ESP_LOGI(TAG, "=== Testing Gimbal Initialization ===");
    
    esp_err_t ret = gimbal_controller_init();
    if (ret == ESP_OK) {
        ESP_LOGI(TAG, "✓ Gimbal controller initialized successfully");
    } else {
        ESP_LOGE(TAG, "✗ Gimbal controller initialization failed: %s", esp_err_to_name(ret));
        return;
    }
    
    gimbal_control_t status;
    ret = gimbal_controller_get_status(&status);
    if (ret == ESP_OK) {
        ESP_LOGI(TAG, "✓ Gimbal status read successfully");
        ESP_LOGI(TAG, "  Mode: %d, Pan: %d, Tilt: %d", status.mode, status.pan_position, status.tilt_position);
    } else {
        ESP_LOGE(TAG, "✗ Failed to read gimbal status: %s", esp_err_to_name(ret));
    }
}

static void test_gimbal_center_position(void)
{
    ESP_LOGI(TAG, "=== Testing Gimbal Center Position ===");
    
    esp_err_t ret = gimbal_controller_center();
    if (ret == ESP_OK) {
        ESP_LOGI(TAG, "✓ Gimbal centered successfully");
    } else {
        ESP_LOGE(TAG, "✗ Failed to center gimbal: %s", esp_err_to_name(ret));
        return;
    }
    
    vTaskDelay(pdMS_TO_TICKS(2000)); // Wait for movement to complete
    
    uint16_t pan, tilt;
    ret = gimbal_controller_get_position(&pan, &tilt);
    if (ret == ESP_OK) {
        ESP_LOGI(TAG, "✓ Current position - Pan: %d, Tilt: %d", pan, tilt);
        // Check if close to center (allowing some tolerance)
        if (abs((int)pan - GIMBAL_PAN_CENTER) < 50 && abs((int)tilt - GIMBAL_TILT_CENTER) < 50) {
            ESP_LOGI(TAG, "✓ Gimbal is properly centered");
        } else {
            ESP_LOGW(TAG, "⚠ Gimbal position may not be exactly centered");
        }
    } else {
        ESP_LOGE(TAG, "✗ Failed to read gimbal position: %s", esp_err_to_name(ret));
    }
}

static void test_gimbal_basic_movement(void)
{
    ESP_LOGI(TAG, "=== Testing Gimbal Basic Movement ===");
    
    // Test pan movement
    ESP_LOGI(TAG, "Testing pan movement...");
    esp_err_t ret = gimbal_controller_set_position(GIMBAL_PAN_CENTER + 300, GIMBAL_TILT_CENTER);
    if (ret == ESP_OK) {
        ESP_LOGI(TAG, "✓ Pan right command sent");
        vTaskDelay(pdMS_TO_TICKS(1500));
        
        ret = gimbal_controller_set_position(GIMBAL_PAN_CENTER - 300, GIMBAL_TILT_CENTER);
        if (ret == ESP_OK) {
            ESP_LOGI(TAG, "✓ Pan left command sent");
            vTaskDelay(pdMS_TO_TICKS(1500));
        }
    }
    
    // Test tilt movement
    ESP_LOGI(TAG, "Testing tilt movement...");
    ret = gimbal_controller_set_position(GIMBAL_PAN_CENTER, GIMBAL_TILT_CENTER + 200);
    if (ret == ESP_OK) {
        ESP_LOGI(TAG, "✓ Tilt up command sent");
        vTaskDelay(pdMS_TO_TICKS(1500));
        
        ret = gimbal_controller_set_position(GIMBAL_PAN_CENTER, GIMBAL_TILT_CENTER - 200);
        if (ret == ESP_OK) {
            ESP_LOGI(TAG, "✓ Tilt down command sent");
            vTaskDelay(pdMS_TO_TICKS(1500));
        }
    }
    
    // Return to center
    ESP_LOGI(TAG, "Returning to center...");
    gimbal_controller_center();
    vTaskDelay(pdMS_TO_TICKS(1500));
}

static void test_gimbal_limits(void)
{
    ESP_LOGI(TAG, "=== Testing Gimbal Limits ===");
    
    // Test pan limits
    ESP_LOGI(TAG, "Testing pan limits...");
    esp_err_t ret = gimbal_controller_set_position(GIMBAL_PAN_MAX, GIMBAL_TILT_CENTER);
    if (ret == ESP_OK) {
        ESP_LOGI(TAG, "✓ Pan max limit command sent");
        vTaskDelay(pdMS_TO_TICKS(2000));
        
        ret = gimbal_controller_set_position(GIMBAL_PAN_MIN, GIMBAL_TILT_CENTER);
        if (ret == ESP_OK) {
            ESP_LOGI(TAG, "✓ Pan min limit command sent");
            vTaskDelay(pdMS_TO_TICKS(2000));
        }
    }
    
    // Test tilt limits
    ESP_LOGI(TAG, "Testing tilt limits...");
    ret = gimbal_controller_set_position(GIMBAL_PAN_CENTER, GIMBAL_TILT_MAX);
    if (ret == ESP_OK) {
        ESP_LOGI(TAG, "✓ Tilt max limit command sent");
        vTaskDelay(pdMS_TO_TICKS(2000));
        
        ret = gimbal_controller_set_position(GIMBAL_PAN_CENTER, GIMBAL_TILT_MIN);
        if (ret == ESP_OK) {
            ESP_LOGI(TAG, "✓ Tilt min limit command sent");
            vTaskDelay(pdMS_TO_TICKS(2000));
        }
    }
    
    // Return to center
    ESP_LOGI(TAG, "Returning to center...");
    gimbal_controller_center();
    vTaskDelay(pdMS_TO_TICKS(1500));
}

static void test_gimbal_smooth_movement(void)
{
    ESP_LOGI(TAG, "=== Testing Gimbal Smooth Movement ===");
    
    // Test different speeds
    ESP_LOGI(TAG, "Testing slow movement...");
    esp_err_t ret = gimbal_controller_smooth_move(GIMBAL_PAN_CENTER + 400, GIMBAL_TILT_CENTER + 300, 0.3f, 0.2f);
    if (ret == ESP_OK) {
        ESP_LOGI(TAG, "✓ Slow movement command sent");
        vTaskDelay(pdMS_TO_TICKS(3000));
    }
    
    ESP_LOGI(TAG, "Testing fast movement...");
    ret = gimbal_controller_smooth_move(GIMBAL_PAN_CENTER - 400, GIMBAL_TILT_CENTER - 300, 0.8f, 0.5f);
    if (ret == ESP_OK) {
        ESP_LOGI(TAG, "✓ Fast movement command sent");
        vTaskDelay(pdMS_TO_TICKS(2000));
    }
    
    // Test velocity control
    ESP_LOGI(TAG, "Testing velocity control...");
    ret = gimbal_controller_set_velocity(30.0f, 20.0f); // degrees per second
    if (ret == ESP_OK) {
        ESP_LOGI(TAG, "✓ Velocity control started");
        vTaskDelay(pdMS_TO_TICKS(2000));
        
        // Stop velocity movement
        gimbal_controller_stop();
        ESP_LOGI(TAG, "✓ Velocity control stopped");
    }
    
    // Return to center
    ESP_LOGI(TAG, "Returning to center...");
    gimbal_controller_center();
    vTaskDelay(pdMS_TO_TICKS(1500));
}

static void test_gimbal_feedback(void)
{
    ESP_LOGI(TAG, "=== Testing Gimbal Feedback ===");
    
    gimbal_feedback_t pan_fb, tilt_fb;
    esp_err_t ret = gimbal_controller_get_feedback(&pan_fb, &tilt_fb);
    if (ret == ESP_OK) {
        ESP_LOGI(TAG, "✓ Servo feedback read successfully");
        ESP_LOGI(TAG, "Pan Servo - Status: %d, Pos: %d, Speed: %d, Load: %d, Voltage: %.2f, Current: %.2f, Temp: %.1f°C", 
                 pan_fb.status, pan_fb.pos, pan_fb.speed, pan_fb.load, pan_fb.voltage, pan_fb.current, pan_fb.temper);
        ESP_LOGI(TAG, "Tilt Servo - Status: %d, Pos: %d, Speed: %d, Load: %d, Voltage: %.2f, Current: %.2f, Temp: %.1f°C", 
                 tilt_fb.status, tilt_fb.pos, tilt_fb.speed, tilt_fb.load, tilt_fb.voltage, tilt_fb.current, tilt_fb.temper);
    } else {
        ESP_LOGW(TAG, "⚠ Failed to read servo feedback (may not be available): %s", esp_err_to_name(ret));
    }
    
    // Test torque control
    ESP_LOGI(TAG, "Testing torque control...");
    ret = gimbal_controller_set_torque(GIMBAL_PAN_ID, false);
    if (ret == ESP_OK) {
        ESP_LOGI(TAG, "✓ Pan servo torque disabled");
        vTaskDelay(pdMS_TO_TICKS(1000));
        
        ret = gimbal_controller_set_torque(GIMBAL_PAN_ID, true);
        if (ret == ESP_OK) {
            ESP_LOGI(TAG, "✓ Pan servo torque enabled");
        }
    } else {
        ESP_LOGW(TAG, "⚠ Torque control may not be available: %s", esp_err_to_name(ret));
    }
}

static void test_gimbal_stabilization(void)
{
    ESP_LOGI(TAG, "=== Testing Gimbal Stabilization ===");
    
    // Enable steady mode
    esp_err_t ret = gimbal_controller_enable_steady_mode(true);
    if (ret == ESP_OK) {
        ESP_LOGI(TAG, "✓ Steady mode enabled");
        
        // Set a steady goal
        ret = gimbal_controller_set_steady_goal(0.0f, 0.0f); // Level horizon
        if (ret == ESP_OK) {
            ESP_LOGI(TAG, "✓ Steady goal set to level horizon");
            
            // Test with IMU data (simulate some movement)
            imu_data_t test_imu_data = {};
            test_imu_data.roll = 5.0f * M_PI / 180.0f;   // 5 degree roll in radians
            test_imu_data.pitch = -3.0f * M_PI / 180.0f; // 3 degree pitch in radians
            test_imu_data.yaw = 0.0f;                    // No yaw
            test_imu_data.euler_valid = true;
            
            ret = gimbal_controller_stabilize(&test_imu_data);
            if (ret == ESP_OK) {
                ESP_LOGI(TAG, "✓ Stabilization applied with test IMU data");
            } else {
                ESP_LOGW(TAG, "⚠ Stabilization failed: %s", esp_err_to_name(ret));
            }
            
            vTaskDelay(pdMS_TO_TICKS(3000)); // Let stabilization work
        }
        
        // Disable steady mode
        gimbal_controller_enable_steady_mode(false);
        ESP_LOGI(TAG, "✓ Steady mode disabled");
    } else {
        ESP_LOGW(TAG, "⚠ Steady mode may not be available: %s", esp_err_to_name(ret));
    }
    
    // Return to center
    ESP_LOGI(TAG, "Returning to center...");
    gimbal_controller_center();
    vTaskDelay(pdMS_TO_TICKS(1500));
}

static void run_gimbal_tests(void)
{
    ESP_LOGI(TAG, "========================================");
    ESP_LOGI(TAG, "      STARTING GIMBAL TEST SUITE      ");
    ESP_LOGI(TAG, "========================================");
    
    // Test 1: Initialization
    test_gimbal_initialization();
    vTaskDelay(pdMS_TO_TICKS(1000));
    
    // Test 2: Center Position
    test_gimbal_center_position();
    vTaskDelay(pdMS_TO_TICKS(1000));
    
    // Test 3: Basic Movement
    test_gimbal_basic_movement();
    vTaskDelay(pdMS_TO_TICKS(1000));
    
    // Test 4: Limits
    test_gimbal_limits();
    vTaskDelay(pdMS_TO_TICKS(1000));
    
    // Test 5: Smooth Movement
    test_gimbal_smooth_movement();
    vTaskDelay(pdMS_TO_TICKS(1000));
    
    // Test 6: Feedback
    test_gimbal_feedback();
    vTaskDelay(pdMS_TO_TICKS(1000));
    
    // Test 7: Stabilization
    test_gimbal_stabilization();
    vTaskDelay(pdMS_TO_TICKS(1000));
    
    ESP_LOGI(TAG, "========================================");
    ESP_LOGI(TAG, "       GIMBAL TEST SUITE COMPLETE     ");
    ESP_LOGI(TAG, "========================================");
}
