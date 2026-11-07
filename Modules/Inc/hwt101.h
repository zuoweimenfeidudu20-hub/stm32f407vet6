#ifndef HWT101_H
#define HWT101_H

#include "stm32f4xx_hal.h"
#include <stdint.h>

typedef enum {
    HWT101_TURN_IDLE = -1,
    HWT101_TURN_INVALID = -2,
    HWT101_TURN_NO_YAW = -3,
    HWT101_TURN_TIMEOUT = -4,
    HWT101_TURN_WRONG_DIRECTION = -5,
    HWT101_TURN_ACTIVE = 0,
    HWT101_TURN_REACHED = 1
} hwt101_turn_state;

HAL_StatusTypeDef hwt101_init(UART_HandleTypeDef *uart);
uint8_t hwt101_get_yaw(float *yaw, uint32_t *age_ms);
float hwt101_angle_error(float target, float current);
/* Positive angle follows increasing yaw; negative angle follows decreasing
 * yaw. Start once from the main loop, then sample in the 10 ms timer tick. */
HAL_StatusTypeDef hwt101_turn_begin(float delta_deg, uint32_t timeout_ms);
void hwt101_turn_tick10ms(void);
/* -1: invalid angle/timeout/not started; 0: turning; 1: angle reached. */
int hwt101_turn_result(void);
hwt101_turn_state hwt101_turn_get_state(void);
uint8_t hwt101_turn_get_angles(float *start, float *end, uint8_t *end_valid);
/* Compatibility with the existing mission state machine. */

#endif
