#include "pidmotor.h"
#include "tim.h"
#include "motor.h"
#include <stdlib.h>

static int previous_cntr;//tim3
static int previous_cntl;//tim4
static encoder_cnts encoder_cnt;
static int pid_previous_cntl;
static int pid_previous_cntr;

PIDMotor_Controller left_pid = {0, 0, 0};
PIDMotor_Controller right_pid = {0, 0, 0};

volatile int init_rightpwm=0;
volatile int init_leftpwm=0;
volatile int right_count_10ms=0;
volatile int left_count_10ms=0;
volatile int right_output=0;
volatile int left_output=0;

static int encoder_delta(uint16_t current, uint16_t previous)
{
    int delta=current-previous;
    return delta;
}

HAL_StatusTypeDef encoder_init(void)
{
    HAL_StatusTypeDef status;

    status = HAL_TIM_Encoder_Start(&htim3, TIM_CHANNEL_ALL);
    if (status != HAL_OK) return status;
    status = HAL_TIM_Encoder_Start(&htim4, TIM_CHANNEL_ALL);
    if (status != HAL_OK){
        HAL_TIM_Encoder_Stop(&htim3, TIM_CHANNEL_ALL);
        return status;
    }

    previous_cntl=(uint16_t)__HAL_TIM_GET_COUNTER(&htim3);
    previous_cntr=(uint16_t)__HAL_TIM_GET_COUNTER(&htim4);
    pid_previous_cntl=previous_cntl;
    pid_previous_cntr=previous_cntr;
    encoder_cnt.lcnt=0;
    encoder_cnt.rcnt=0;

    return HAL_OK;
}

void encoder_update(void)
{
    uint16_t current_cntl=(uint16_t)__HAL_TIM_GET_COUNTER(&htim3);
    uint16_t current_cntr=(uint16_t)__HAL_TIM_GET_COUNTER(&htim4);

    encoder_cnt.lcnt=encoder_delta(current_cntl,previous_cntl);
    encoder_cnt.rcnt=encoder_delta(current_cntr,previous_cntr);
    previous_cntl=current_cntl;
    previous_cntr=current_cntr;
}

encoder_cnts get_encodercnt(void)
{
    return encoder_cnt;
}

extern int Clamp(int32_t value, int32_t min, int32_t max)
{
    if (value < min) return min;
    if (value > max) return max;
    return value;
}

int Sign(int32_t value)
{
    return (value > 0) - (value < 0);
}

extern int PercentToCounts(int32_t percent)
{
    int32_t counts;
    percent = Clamp(percent, -100, 100);
    counts = (abs(percent)*max_count_10ms+50) / 100;
    if (percent!=0&&counts==0) counts=1;
    if(percent<0){
        return -counts;
    }else{
        return counts;
    }
}

void pidmotor_rst(PIDMotor_Controller *controller)
{
    controller->integral = 0;
    controller->previous_error = 0;
    controller->previous_target = 0;
}

int pid_motor(PIDMotor_Controller *controller, int target_percent,int target_counts, int measured_counts)
{
    int error,derivative,correction,output,previous_integral;

    if (Sign(target_percent) != Sign(controller->previous_target)) {
        controller->integral=0;
        controller->previous_error=0;
    }
    error=target_counts-measured_counts;
    derivative = error - controller->previous_error;
    previous_integral = controller->integral;
    controller->integral = Clamp(controller->integral+error,-integral_limit, integral_limit);
    
    correction = (motor_kp*error+motor_ki*controller->integral+motor_kd*derivative)/pid_motor_gain_scale;

    if(target_percent>0){
        output=Clamp(target_percent + correction, 0, 100);
    }else if(target_percent<0) {
        output = Clamp(target_percent + correction, -100, 0);
    } else {
        pidmotor_rst(controller);
        output = 0;
    }

    if (((target_percent > 0) &&(((output == 100) && (error > 0)) || ((output == 0) && (error < 0)))) ||((target_percent < 0) &&(((output == -100) && (error < 0)) || ((output == 0) && (error > 0)))))
    {
        controller->integral = previous_integral;
    }

    controller->previous_error = error;
    controller->previous_target = target_percent;
    return output;
}

void pidmotor_update(void)
{
    int current_cntl=__HAL_TIM_GET_COUNTER(&htim3);
    int current_cntr=__HAL_TIM_GET_COUNTER(&htim4);
    int target_left = Clamp(init_leftpwm, -100, 100);
    int target_right = Clamp(init_rightpwm, -100, 100);

    left_count_10ms = encoder_delta(current_cntl, pid_previous_cntl);
    right_count_10ms = encoder_delta(current_cntr, pid_previous_cntr);
    pid_previous_cntl = current_cntl;
    pid_previous_cntr = current_cntr;

    right_output=pid_motor(&right_pid, target_right, PercentToCounts(target_right),right_count_10ms * encoder_right_dir);
    left_output=pid_motor(&left_pid, target_left, PercentToCounts(target_left),left_count_10ms * encoder_left_dir);
    
    rightmotor_output(right_output);
    leftmotor_output(left_output);
}

void pidmotor_init(void)
{
    uint32_t primask=__get_PRIMASK();
    __disable_irq();
    
    init_leftpwm = 0;
    init_rightpwm = 0;
    left_output = 0;
    right_output = 0;
    pidmotor_rst(&left_pid);
    pidmotor_rst(&right_pid);
    pid_previous_cntl = (uint16_t)__HAL_TIM_GET_COUNTER(&htim3);
    pid_previous_cntr = (uint16_t)__HAL_TIM_GET_COUNTER(&htim4);
    allstop();
    
    if(primask==0){
        __enable_irq();
    }
}
