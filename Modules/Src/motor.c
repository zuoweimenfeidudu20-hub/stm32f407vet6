#include "motor.h"
#include "tim.h"

static uint8_t right_reverse;
static uint8_t left_reverse;
//reverse1+
static void motor_pwm(uint32_t pwm1, uint32_t pwm2,uint8_t reverse, uint32_t percent)
{
    uint32_t period=__HAL_TIM_GET_AUTORELOAD(&htim1)+1;
    if(percent>100){
        percent=100;
    }
    __HAL_TIM_SET_COMPARE(&htim1, reverse ? pwm1 : pwm2, 0U);
    __HAL_TIM_SET_COMPARE(&htim1, reverse ? pwm2 : pwm1,(period *percent)/100U);
}

HAL_StatusTypeDef motor_init(void)
{
    static const uint32_t channels[]={MOTOR_LEFT_PWM1_CHANNEL, MOTOR_LEFT_PWM2_CHANNEL,MOTOR_RIGHT_PWM1_CHANNEL, MOTOR_RIGHT_PWM2_CHANNEL};
    HAL_StatusTypeDef status;

    right_reverse=0;
    left_reverse=0;
    allstop();
    
    htim1.Instance->EGR=TIM_EGR_UG;//强制读取
    for (int i=0;i<4;++i) {
        status=HAL_TIM_PWM_Start(&htim1, channels[i]);
        if(status!=HAL_OK){
            allstop();
            while (i>0) {
                --i;
                HAL_TIM_PWM_Stop(&htim1, channels[i]);
            }
            return status;
        }
    }
    return HAL_OK;
}

void rightmotor_pwm(int percent)
{
    motor_pwm(MOTOR_RIGHT_PWM1_CHANNEL, MOTOR_RIGHT_PWM2_CHANNEL,right_reverse, percent);
}

void leftmotor_pwm(int percent)
{
    motor_pwm(MOTOR_LEFT_PWM1_CHANNEL, MOTOR_LEFT_PWM2_CHANNEL,left_reverse, percent);
}

void rightmotor_stop(void)
{
    rightmotor_pwm(0);
}

void rightmotor_forward(int speed)
{
    right_reverse=1;
    rightmotor_pwm(speed);
}

void rightmotor_reverse(int speed)
{
    right_reverse=0;
    rightmotor_pwm(speed);
}

void leftmotor_stop(void)
{
    leftmotor_pwm(0);
}

void leftmotor_forward(int speed)
{
    left_reverse=1;
    leftmotor_pwm(speed);
}

void leftmotor_reverse(int speed)
{
    left_reverse=0;
    leftmotor_pwm(speed);
}

void allstop(void) 
{ 
    rightmotor_stop(); 
    leftmotor_stop(); 
}

int motor_clamp(int percent)
{
    if(percent>100){
        return 100;
    }
    if(percent<-100){
        return -100;
    }
    return percent;
}

void rightmotor_output(int output)
{
    output=motor_clamp(output);
    if(output>0){
        rightmotor_forward(output);
    }
    else if(output<0){
        rightmotor_reverse(-output);
    }
    else rightmotor_stop();
}

void leftmotor_output(int output)
{
    output=motor_clamp(output);
    if(output>0){
        leftmotor_forward(output);
    }
    else if(output<0){
        leftmotor_reverse(-output);
    }
    else leftmotor_stop();
}
