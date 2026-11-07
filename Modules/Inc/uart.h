#ifndef APP_UART_H_
#define APP_UART_H_

#include <stdint.h>

typedef enum {
    UART_SEND_NOT_ATTEMPTED = 0,
    UART_SEND_OK,
    UART_SEND_TIMEOUT,
    UART_SEND_BUSY,
    UART_SEND_ERROR,
    UART_SEND_FORMAT_ERROR
} uart_send_status_t;

void uart_report_encoder(int32_t tim3_count, int32_t tim4_count);
/* Call from the main loop; reports PID values once per second on USART1. */
void uart_report_pidmotor(void);
/* Call from the main loop; sends once per second on USART1. */
void uart_send_hello(void);
/* Status of the latest send attempt; suitable for debugger inspection. */
uart_send_status_t uart_get_last_status(void);

#endif
