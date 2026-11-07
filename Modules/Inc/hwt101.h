#ifndef HWT101_H
#define HWT101_H

#include "stm32f4xx_hal.h"
#include <stdint.h>

HAL_StatusTypeDef hwt101_init(UART_HandleTypeDef *uart);
uint8_t hwt101_get_yaw(float *yaw, uint32_t *age_ms);
float hwt101_angle_error(float target, float current);
int hwt101_turn_right_35(uint8_t start);

#endif
