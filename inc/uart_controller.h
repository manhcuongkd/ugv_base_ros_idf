#ifndef UART_CONTROLLER_H
#define UART_CONTROLLER_H

#include <esp_err.h>
#include <freertos/FreeRTOS.h>
#include <freertos/task.h>

#ifdef __cplusplus
extern "C" {
#endif

// Function declarations
esp_err_t uart_controller_init(void);
esp_err_t uart_controller_deinit(void);
void uart_controller_task(void *pvParameters);

#ifdef __cplusplus
}
#endif

#endif // UART_CONTROLLER_H
