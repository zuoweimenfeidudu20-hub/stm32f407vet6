#include "uart.h"
#include "usart.h"
#include "pidmotor.h"
#include <stdio.h>

static volatile uart_send_status_t uart_last_status = UART_SEND_NOT_ATTEMPTED;

static void uart_send_bytes(const uint8_t *data, uint16_t length)
{
    HAL_StatusTypeDef status = HAL_UART_Transmit(&huart1, data, length, 100U);
    switch (status) {
    case HAL_OK:      uart_last_status = UART_SEND_OK; break;
    case HAL_TIMEOUT: uart_last_status = UART_SEND_TIMEOUT; break;
    case HAL_BUSY:    uart_last_status = UART_SEND_BUSY; break;
    default:          uart_last_status = UART_SEND_ERROR; break;
    }
}

uart_send_status_t uart_get_last_status(void)
{
    return uart_last_status;
}

void uart_report_encoder(int32_t tim3_count, int32_t tim4_count)
{
    static uint32_t last_tick;
    char message[80];
    uint32_t now = HAL_GetTick();
    int length;

    if ((now - last_tick) < 1000U) return;
    last_tick = now;
    length = snprintf(message, sizeof(message), "TIM3=%ld, TIM4=%ld\r\n",
                      (long)tim3_count, (long)tim4_count);
    if ((length <= 0) || ((uint32_t)length >= sizeof(message))) {
        uart_last_status = UART_SEND_FORMAT_ERROR;
        return;
    }
    uart_send_bytes((const uint8_t *)message, (uint16_t)length);
}

void uart_report_pidmotor(void)
{
    static uint32_t last_tick;
    char message[160];
    uint32_t now = HAL_GetTick();
    int length;

    if ((now - last_tick) < 20U) return;
    last_tick = now;
    length = snprintf(message, sizeof(message),
                      "%ld,%ld,%ld,%ld,%ld\r\n",
                      (long)PercentToCounts(init_rightpwm),(long)right_count_10ms,(long)left_count_10ms,(long)left_pid.integral,(long)right_pid.integral);
    if ((length <= 0) || ((uint32_t)length >= sizeof(message))) {
        uart_last_status = UART_SEND_FORMAT_ERROR;
        return;
    }
    uart_send_bytes((const uint8_t *)message, (uint16_t)length);
}
