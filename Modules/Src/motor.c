#include "motor.h"
#include "tim.h"

static uint8_t right_reverse;
static uint8_t left_reverse;

static void motor_pwm(uint32_t pwm1, uint32_t pwm2,
                      uint8_t reverse, uint32_t percent)
{
    uint32_t period = __HAL_TIM_GET_AUTORELOAD(&htim1) + 1U;
    if (percent > 100U) percent = 100U;
    __HAL_TIM_SET_COMPARE(&htim1, reverse ? pwm1 : pwm2, 0U);
    __HAL_TIM_SET_COMPARE(&htim1, reverse ? pwm2 : pwm1,
                          (period * percent) / 100U);
}

HAL_StatusTypeDef motor_init(void)
{
    static const uint32_t channels[]={
        MOTOR_LEFT_PWM1_CHANNEL, MOTOR_LEFT_PWM2_CHANNEL,
        MOTOR_RIGHT_PWM1_CHANNEL, MOTOR_RIGHT_PWM2_CHANNEL
    };
    HAL_StatusTypeDef status;
    uint32_t i;

    right_reverse = 0U;
    left_reverse = 0U;
    allstop();
    
    htim1.Instance->EGR = TIM_EGR_UG;
    for (i = 0U; i < 4U; ++i) {
        status = HAL_TIM_PWM_Start(&htim1, channels[i]);
        if (status != HAL_OK) {
            allstop();
            while (i > 0U) {
                --i;
                HAL_TIM_PWM_Stop(&htim1, channels[i]);
            }
            return status;
        }
    }
    return HAL_OK;
}

void rightmotor_pwm(uint32_t percent)
{
    motor_pwm(MOTOR_RIGHT_PWM1_CHANNEL, MOTOR_RIGHT_PWM2_CHANNEL,
              right_reverse, percent);
}

void leftmotor_pwm(uint32_t percent)
{
    motor_pwm(MOTOR_LEFT_PWM1_CHANNEL, MOTOR_LEFT_PWM2_CHANNEL,
              left_reverse, percent);
}

void rightmotor_stop(void)
{
    rightmotor_pwm(0U);
}

void rightmotor_forward(uint32_t speed)
{
    right_reverse = 0U;
    rightmotor_pwm(speed);
}

void rightmotor_reverse(uint32_t speed)
{
    right_reverse = 1U;
    rightmotor_pwm(speed);
}

void leftmotor_stop(void)
{
    leftmotor_pwm(0U);
}

void leftmotor_forward(uint32_t speed)
{
    left_reverse = 0U;
    leftmotor_pwm(speed);
}

void leftmotor_reverse(uint32_t speed)
{
    left_reverse = 1U;
    leftmotor_pwm(speed);
}

void allstop(void) { rightmotor_stop(); leftmotor_stop(); }

int32_t motor_clamp(int32_t percent)
{
    if (percent > 100) return 100;
    if (percent < -100) return -100;
    return percent;
}

void rightmotor_output(int32_t output)
{
    output = motor_clamp(output);
    if (output > 0) rightmotor_forward((uint32_t)output);
    else if (output < 0) rightmotor_reverse((uint32_t)(-output));
    else rightmotor_stop();
}

void leftmotor_output(int32_t output)
{
    output = motor_clamp(output);
    if (output > 0) leftmotor_forward((uint32_t)output);
    else if (output < 0) leftmotor_reverse((uint32_t)(-output));
    else leftmotor_stop();
}
