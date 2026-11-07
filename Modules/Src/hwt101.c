#include "hwt101.h"

#define HWT101_FRAME_SIZE 11U

static UART_HandleTypeDef *sensor_uart;
static uint8_t rx_byte;
static uint8_t frame[HWT101_FRAME_SIZE];
static uint8_t frame_len;
static volatile int16_t yaw_raw;
static volatile uint32_t yaw_tick;
static volatile uint8_t yaw_valid;
static float right_turn_target_yaw;
static uint32_t right_turn_start_tick;
static uint8_t right_turn_active;

/* Sliding window permits recovery after a lost or corrupted byte. */
static void hwt101_parse_byte(uint8_t byte)
{
    uint8_t i;
    uint8_t sum;
    uint16_t raw;

    if ((frame_len == 0U) && (byte != 0x55U)) return;
    frame[frame_len++] = byte;
    if (frame_len < HWT101_FRAME_SIZE) return;

    sum = 0U;
    for (i = 0U; i < HWT101_FRAME_SIZE - 1U; ++i) sum += frame[i];
    if ((frame[1] == 0x53U) && (sum == frame[10])) {
        raw = (uint16_t)frame[6] | ((uint16_t)frame[7] << 8);
        yaw_raw = (int16_t)raw;
        yaw_tick = HAL_GetTick();
        yaw_valid = 1U;
        frame_len = 0U;
        return;
    }

    /* Valid non-angle frames can be discarded whole; otherwise resync. */
    if ((frame[0] == 0x55U) && (sum == frame[10])) {
        frame_len = 0U;
        return;
    }
    for (i = 1U; i < HWT101_FRAME_SIZE; ++i) {
        if (frame[i] == 0x55U) break;
    }
    frame_len = HWT101_FRAME_SIZE - i;
    for (i = 0U; i < frame_len; ++i) frame[i] = frame[HWT101_FRAME_SIZE - frame_len + i];
}

HAL_StatusTypeDef hwt101_init(UART_HandleTypeDef *uart)
{
    HAL_StatusTypeDef status;
    if (uart == NULL) return HAL_ERROR;
    sensor_uart = uart;
    frame_len = 0U;
    yaw_valid = 0U;
    right_turn_active = 0U;
    status = HAL_UART_Receive_IT(sensor_uart, &rx_byte, 1U);
    if (status != HAL_OK) sensor_uart = NULL;
    return status;
}

void HAL_UART_RxCpltCallback(UART_HandleTypeDef *uart)
{
    if (uart == sensor_uart) {
        hwt101_parse_byte(rx_byte);
        (void)HAL_UART_Receive_IT(sensor_uart, &rx_byte, 1U);
    }
}

void HAL_UART_ErrorCallback(UART_HandleTypeDef *uart)
{
    if (uart == sensor_uart) {
        frame_len = 0U;
        (void)HAL_UART_Receive_IT(sensor_uart, &rx_byte, 1U);
    }
}

uint8_t hwt101_get_yaw(float *yaw, uint32_t *age_ms)
{
    uint32_t primask = __get_PRIMASK();
    uint32_t tick;
    int16_t raw;
    uint8_t valid;
    __disable_irq();
    raw = yaw_raw;
    tick = yaw_tick;
    valid = yaw_valid;
    if (primask == 0U) __enable_irq();
    if (valid == 0U) return 0U;
    if (yaw != NULL) *yaw = (float)raw * (180.0f / 32768.0f);
    if (age_ms != NULL) *age_ms = HAL_GetTick() - tick;
    return 1U;
}

float hwt101_angle_error(float target, float current)
{
    float error = target - current;
    while (error >= 180.0f) error -= 360.0f;
    while (error < -180.0f) error += 360.0f;
    return error;
}

int hwt101_turn_right_35(uint8_t start)
{
    float yaw;
    uint32_t age_ms;

    if (!hwt101_get_yaw(&yaw, &age_ms) || age_ms > 100U) {
        right_turn_active = 0U;
        return -1;
    }

    if (start != 0U) {
        /* Right turn is assumed to decrease yaw; verify on the actual car. */
        right_turn_target_yaw = yaw - 35.0f;
        right_turn_start_tick = HAL_GetTick();
        right_turn_active = 1U;
        return 0;
    }

    if (right_turn_active == 0U) return -1;
    if ((HAL_GetTick() - right_turn_start_tick) > 2500U) {
        right_turn_active = 0U;
        return -1;
    }
    if (hwt101_angle_error(right_turn_target_yaw, yaw) >= -2.0f) {
        right_turn_active = 0U;
        return 1;
    }
    return 0;
}
