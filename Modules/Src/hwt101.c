#include "hwt101.h"

#define HWT101_FRAME_SIZE 11U
#define HWT101_MAX_AGE_MS 100U
#define HWT101_TURN_TOLERANCE_DEG 2.0f
#define HWT101_WRONG_DIRECTION_DEG 10.0f
#define HWT101_RIGHT_35_TIMEOUT_MS 2500U

static UART_HandleTypeDef *sensor_uart;
static uint8_t rx_byte;
static uint8_t frame[HWT101_FRAME_SIZE];
static uint8_t frame_len;
static volatile int16_t yaw_raw;
static volatile uint32_t yaw_tick;
static volatile uint8_t yaw_valid;
static float turn_delta_deg;
static float turn_previous_yaw;
static float turn_progress_deg;
static uint32_t turn_start_tick;
static uint32_t turn_timeout_ms;
static volatile int8_t turn_status = HWT101_TURN_IDLE;
static float turn_start_yaw;
static float turn_end_yaw;
static volatile uint8_t turn_angles_valid;
static volatile uint8_t turn_end_valid;

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
    turn_status = HWT101_TURN_IDLE;
    turn_angles_valid = 0U;
    turn_end_valid = 0U;
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

HAL_StatusTypeDef hwt101_turn_begin(float delta_deg, uint32_t timeout_ms)
{
    float yaw;
    uint32_t age_ms;
    uint32_t primask;

    turn_angles_valid = 0U;
    turn_end_valid = 0U;
    /* The shortest-angle comparison is unambiguous only below 180 degrees. */
    if (delta_deg == 0.0f || delta_deg <= -180.0f ||
        delta_deg >= 180.0f || timeout_ms == 0U) {
        turn_status = HWT101_TURN_INVALID;
        return HAL_ERROR;
    }
    if (!hwt101_get_yaw(&yaw, &age_ms) || age_ms > HWT101_MAX_AGE_MS) {
        turn_status = HWT101_TURN_NO_YAW;
        return HAL_ERROR;
    }

    primask = __get_PRIMASK();
    __disable_irq();
    turn_start_yaw = yaw;
    turn_previous_yaw = yaw;
    turn_progress_deg = 0.0f;
    turn_angles_valid = 1U;
    turn_delta_deg = delta_deg;
    turn_start_tick = HAL_GetTick();
    turn_timeout_ms = timeout_ms;
    turn_status = 0;
    if (primask == 0U) __enable_irq();
    return HAL_OK;
}

void hwt101_turn_tick1ms(void)
{
    float yaw;
    float directed_progress;
    uint32_t age_ms;

    if (turn_status != 0) return;
    if ((HAL_GetTick() - turn_start_tick) > turn_timeout_ms) {
        if (hwt101_get_yaw(&yaw, &age_ms) && age_ms <= HWT101_MAX_AGE_MS) {
            turn_end_yaw = yaw;
            turn_end_valid = 1U;
        }
        turn_status = HWT101_TURN_TIMEOUT;
        return;
    }
    if (!hwt101_get_yaw(&yaw, &age_ms) || age_ms > HWT101_MAX_AGE_MS) {
        turn_status = HWT101_TURN_NO_YAW;
        return;
    }

    turn_progress_deg += hwt101_angle_error(yaw, turn_previous_yaw);
    turn_previous_yaw = yaw;
    directed_progress = turn_delta_deg < 0.0f ? -turn_progress_deg : turn_progress_deg;
    if (directed_progress <= -HWT101_WRONG_DIRECTION_DEG) {
        turn_end_yaw = yaw;
        turn_end_valid = 1U;
        turn_status = HWT101_TURN_WRONG_DIRECTION;
    }
    else if (directed_progress >=
             (turn_delta_deg < 0.0f ? -turn_delta_deg : turn_delta_deg) - HWT101_TURN_TOLERANCE_DEG) {
        turn_end_yaw = yaw;
        turn_end_valid = 1U;
        turn_status = 1;
    }
}

int hwt101_turn_result(void)
{
    return turn_status < 0 ? -1 : turn_status;
}

hwt101_turn_state hwt101_turn_get_state(void)
{
    return (hwt101_turn_state)turn_status;
}

uint8_t hwt101_turn_get_angles(float *start, float *end, uint8_t *end_valid)
{
    uint32_t primask;
    uint8_t valid;
    if (start == NULL || end == NULL || end_valid == NULL) return 0U;
    primask = __get_PRIMASK();
    __disable_irq();
    valid = turn_angles_valid;
    *start = turn_start_yaw;
    *end = turn_end_yaw;
    *end_valid = turn_end_valid;
    if (primask == 0U) __enable_irq();
    return valid;
}
