#include "uart.h"
#include "usart.h"
#include "pidmotor.h"
#include "mission.h"
#include "hwt101.h"
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

static const char *state1_name(trackstate1 state)
{
    switch (state)
    {
    case state1_line: return "state1_line";
    case state1_c_lockstraight1: return "state1_c_lockstraight1";
    case state1_c_lockstraight2: return "state1_c_lockstraight2";
    case state1_tofinish: return "state1_tofinish";
    case state1_stop: return "state1_stop";
    case state1_fault: return "state1_fault";
    default: return "state1_unknown";
    }
}

static const char *state2_name(trackstate2 state)
{
    switch (state)
    {
    case state2_line: return "state2_line";
    case state2_c_lockright: return "state2_c_lockright";
    case state2_stair1: return "state2_stair1";
    case state2_teshu_lockleft1: return "state2_teshu_lockleft1";
    case state2_stair2: return "state2_stair2";
    case state2_branch_lockleft2: return "state2_branch_lockleft2";
    case state2_stair3: return "state2_stair3";
    case state2_teshu_lockright2: return "state2_teshu_lockright2";
    case state2_stair4: return "state2_stair4";
    case state2_stop: return "state2_stop";
    case state2_fault: return "state2_fault";
    default: return "state2_unknown";
    }
}

static const char *state3_name(trackstate3 state)
{
    switch (state)
    {
    case state3_line: return "state3_line";
    case state3_c_lockstraight1: return "state3_c_lockstraight1";
    case state3_c_lockstraight2: return "state3_c_lockstraight2";
    case state3_line2: return "state3_line2";
    case state3_baidong: return "state3_baidong";
    case state3_tofinish: return "state3_tofinish";
    case state3_stop: return "state3_stop";
    case state3_fault: return "state3_fault";
    default: return "state3_unknown";
    }
}

static const char *turn_name(hwt101_turn_state state)
{
    switch (state)
    {
    case HWT101_TURN_IDLE: return "idle";
    case HWT101_TURN_INVALID: return "invalid";
    case HWT101_TURN_NO_YAW: return "no_yaw";
    case HWT101_TURN_TIMEOUT: return "timeout";
    case HWT101_TURN_WRONG_DIRECTION: return "wrong_direction";
    case HWT101_TURN_ACTIVE: return "turning";
    case HWT101_TURN_REACHED: return "reached";
    default: return "unknown";
    }
}

void track_report(void)
{
    static uint32_t last_tick;
    static keymode last_mode = key4_idle;
    static int last_state = -1;
    static int last_turn = -100;
    keymode mode;
    trackpattern pattern;
    trackstate1 state1;
    trackstate2 state2;
    trackstate3 state3;
    uint32_t now = HAL_GetTick();
    int state;
    int show_turn;
    hwt101_turn_state turn;
    const char *name;
    static char message[64];
    int length;

    if (!mission_report_snapshot(&mode, &pattern, &state1, &state2, &state3)) return;
    state = mode == key1_putong ? (int)state1 : mode == key2_teshu ? (int)state2 : (int)state3;
    name = mode == key1_putong ? state1_name(state1) : mode == key2_teshu ? state2_name(state2) : state3_name(state3);
    show_turn = 0;
    turn = hwt101_turn_get_state();
    if (mode == last_mode && state == last_state &&
        (!show_turn || (int)turn == last_turn) &&
        (now - last_tick) < 200U) return;
    if (huart1.gState != HAL_UART_STATE_READY) return;

    if (show_turn)
        length = snprintf(message, sizeof(message), "%s gyro=%s\r\n", name, turn_name(turn));
    else
        length = snprintf(message, sizeof(message), "%s\r\n", name);
    if (length <= 0 || (size_t)length >= sizeof(message)) return;
    if (HAL_UART_Transmit_DMA(&huart1, (uint8_t *)message, (uint16_t)length) != HAL_OK) return;

    last_mode = mode;
    last_state = state;
    last_turn = (int)turn;
    last_tick = now;
}


