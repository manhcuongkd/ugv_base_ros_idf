#ifndef UART_CONTROLLER_H
#define UART_CONTROLLER_H

#include <esp_err.h>
#include <freertos/FreeRTOS.h>
#include <freertos/task.h>

#ifdef __cplusplus
extern "C" {
#endif

// UART Configuration
#define UART_NUM                        UART_NUM_0
#define UART_BAUD_RATE                  115200
#define UART_BUFFER_SIZE                1024

// Function declarations
esp_err_t uart_controller_init(void);
esp_err_t uart_controller_deinit(void);
void uart_controller_task(void *pvParameters);
esp_err_t uart_controller_parse_command(const char *data, size_t len);

#ifdef __cplusplus
}
#endif

#endif // UART_CONTROLLER_H
