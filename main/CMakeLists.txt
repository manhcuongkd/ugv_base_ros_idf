idf_component_register(SRCS "main.cpp"
                           "mission_system.cpp"
                           "ugv_advanced.cpp"
                           "module_handlers.cpp"
                           "system_manager.cpp"
                           "motion_module.cpp"
                           "imu_controller.cpp"
                           "oled_controller.cpp"
                           "battery_controller.cpp"
                           "led_controller.cpp"
                           "servo_controller.cpp"
                           "json_parser.cpp"
                           "uart_controller.cpp"
                           "wifi_controller.cpp"
                           "http_server.cpp"
                           "esp_now_controller.cpp"
                           "files_controller.cpp"
                           "gimbal_controller.cpp"
                    INCLUDE_DIRS "." "../inc")

# Set C++ standard
target_compile_features(${COMPONENT_LIB} PRIVATE cxx_std_17)

# Component dependencies are handled automatically by ESP-IDF
