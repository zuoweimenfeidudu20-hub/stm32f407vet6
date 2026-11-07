#include "pidtrack.h"
#include "i2c.h"
#include "pidmotor.h"
#include <stddef.h>

#define GRAY_INPUT_REGISTER       0x00U
#define GRAY_I2C_TIMEOUT_MS       10U

static const int16_t sensor_weight[GRAY_SENSOR_COUNT]={
    -1600,-1200,-850,-550,-300,-100, 100,300,550,850,1200,1600
};

static int32_t last_correction;
static uint16_t lost_samples;
static uint8_t line_seen;

static track_pid track_control = {
    .pgain=kp,
    .igain=ki,
    .dgain=kd,
    .integral=0,
    .previous_error=0,
    .int_limit=integral_limit,
    .out_limit=output_limit,
};

int line_pid(track_pid *pid, int error)
{
    int derivative;
    int output;

    pid->integral=Clamp(pid->integral + error,-pid->int_limit,pid->int_limit);

    derivative = error - pid->previous_error;

    output=(pid->pgain*error+pid->igain*pid->integral+pid->dgain*derivative)/track_scale;
    output = Clamp(output, -pid->out_limit, pid->out_limit);
    pid->previous_error = error;

    return output;
}

void trackpid_rst(track_pid *pid,int error)
{
    pid->integral=0;
    pid->previous_error=error;
}

void pidtrack_rst(void)
{
    trackpid_rst(&track_control, 0);

    last_correction = 0;
    lost_samples = 0;
    line_seen = 0;
    init_leftpwm = 0;
    init_rightpwm = 0;
}

HAL_StatusTypeDef gray_init(void)
{
    pidtrack_rst();
    return HAL_I2C_IsDeviceReady(&hi2c1,GRAY_PD9555_ADDRESS,3,GRAY_I2C_TIMEOUT_MS);
}

HAL_StatusTypeDef gray_read(uint16_t *sensor_state)
{
    uint8_t input[2];
    uint16_t raw;
    uint16_t reversed = 0U;
    HAL_StatusTypeDef status;

    if (sensor_state==NULL) return HAL_ERROR;

    status=HAL_I2C_Mem_Read(&hi2c1, GRAY_PD9555_ADDRESS,GRAY_INPUT_REGISTER, I2C_MEMADD_SIZE_8BIT,input, sizeof(input), GRAY_I2C_TIMEOUT_MS);
    if (status != HAL_OK) return status;

    raw = (uint16_t)(((uint16_t)input[1] << 8) | input[0]);
    raw &= GRAY_SENSOR_MASK;
    for (int i = 0; i < GRAY_SENSOR_COUNT; i++) {
        if ((raw & (1U << i)) != 0U) {
            reversed |= (uint16_t)(1U << (GRAY_SENSOR_COUNT - 1 - i));
        }
    }
    *sensor_state = reversed;
    return HAL_OK;
}

void set_target(int base_speed,int correction)
{
    init_leftpwm=Clamp(base_speed+correction,0,100);
    init_rightpwm=Clamp(base_speed-correction,0,100);
}

HAL_StatusTypeDef pidtrack_update(void)
{
    uint16_t state;
    int32_t weighted_sum = 0;
    int32_t black_count = 0;
    int32_t error;
    HAL_StatusTypeDef status = gray_read(&state);

    if (status != HAL_OK) {
        set_target(0, 0);
        return status;
    }

    for (int i=0;i<GRAY_SENSOR_COUNT;i++) {
        if ((state & (uint16_t)(1U << i)) != 0U) {
            weighted_sum += sensor_weight[i];
            ++black_count;
        }
    }

    if (black_count == 0) {
        if (line_seen != 0U && lost_samples < lost_ticks) {
            ++lost_samples;
            set_target(PIDTRACK_BASE_SPEED, last_correction);
        } else {
            set_target(0, 0);
        }
        return HAL_OK;
    }

    line_seen=1;
    lost_samples=0;
    error=weighted_sum/black_count;
    last_correction=line_pid(&track_control,error);
    set_target(PIDTRACK_BASE_SPEED, last_correction);
    return HAL_OK;
}

