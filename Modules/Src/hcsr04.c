#include "hcsr04.h"
#include "tim.h"
#include <stddef.h>

#define HCSR04_TIMER_HZ          1000000U
#define HCSR04_TRIGGER_US        10U
#define HCSR04_MIN_INTERVAL_MS   60U//最小测距间隔时间
#define HCSR04_ECHO_TIMEOUT_MS   40U
#define HCSR04_CM_PER_US         0.01715f

typedef enum {
    hcsr04_idle,
    hcsr04_rise,
    hcsr04_fall,
    hcsr04_ready,
}hcsr04_state;

static volatile hcsr04_state state=hcsr04_idle;
static volatile uint16_t rise_tick;//
static volatile uint16_t echo_us;//
static uint32_t trigger_tick;
static int initialized;
static int triggered_once;

//
HAL_StatusTypeDef hcsr04_init(void)
{
    RCC_ClkInitTypeDef clocks;
    uint32_t flash_latency;
    uint32_t timer_hz;
    HAL_StatusTypeDef status;

    if(htim9.Instance!=TIM9|| htim9.Init.Period != 0xFFFFU) return HAL_ERROR;

    HAL_RCC_GetClockConfig(&clocks, &flash_latency);
    timer_hz = HAL_RCC_GetPCLK2Freq();
    if (clocks.APB2CLKDivider != RCC_HCLK_DIV1) timer_hz *= 2U;
    if (timer_hz < HCSR04_TIMER_HZ || timer_hz % HCSR04_TIMER_HZ != 0U)
        return HAL_ERROR;

    __HAL_TIM_SET_PRESCALER(&htim9, timer_hz / HCSR04_TIMER_HZ - 1U);
    htim9.Instance->EGR = TIM_EGR_UG;//强制读取
    __HAL_TIM_SET_COUNTER(&htim9, 0U);
    HAL_GPIO_WritePin(US_TRIG_GPIO_Port, US_TRIG_Pin, GPIO_PIN_RESET);

    status = HAL_TIM_IC_Start_IT(&htim9, TIM_CHANNEL_1);
    if (status != HAL_OK) return status;
    status = HAL_TIM_IC_Start_IT(&htim9, TIM_CHANNEL_2);
    if (status != HAL_OK) {
        (void)HAL_TIM_IC_Stop_IT(&htim9, TIM_CHANNEL_1);
        return status;
    }

    state = hcsr04_idle;
    initialized = 1U;
    triggered_once = 0U;
    return HAL_OK;
}

//超声波激活
HAL_StatusTypeDef hcsr04_trigger(void)
{
    uint32_t now;
    uint16_t start_tick;

    if (initialized==0){
        return HAL_ERROR;
    }

    now=HAL_GetTick();

    if(state==hcsr04_ready){
        return HAL_BUSY;
    }
    if(state==hcsr04_rise||state==hcsr04_fall){
        if(now-trigger_tick<HCSR04_ECHO_TIMEOUT_MS){
            return HAL_BUSY;
        }
        state=hcsr04_idle;
    }
    if(triggered_once&&now-trigger_tick<HCSR04_MIN_INTERVAL_MS){
        return HAL_BUSY;
    }
    
    __HAL_TIM_CLEAR_FLAG(&htim9,TIM_FLAG_CC1|TIM_FLAG_CC2);//强制清零中断位
    
    state=hcsr04_rise;
    trigger_tick=now;
    triggered_once=1;

    start_tick=(uint16_t)__HAL_TIM_GET_COUNTER(&htim9);
    HAL_GPIO_WritePin(US_TRIG_GPIO_Port, US_TRIG_Pin, GPIO_PIN_SET);
    while((uint16_t)((uint16_t)__HAL_TIM_GET_COUNTER(&htim9)-start_tick)<HCSR04_TRIGGER_US)
    { }//阻塞定时
    HAL_GPIO_WritePin(US_TRIG_GPIO_Port, US_TRIG_Pin, GPIO_PIN_RESET);
    return HAL_OK;
}

//读值
HAL_StatusTypeDef read_cm(float *distance_cm)
{
    uint32_t primask;
    hcsr04_state state;
    uint16_t pulse_us;

    if(distance_cm==NULL||initialized==0){
        return HAL_ERROR;
    }

    primask=__get_PRIMASK();
    __disable_irq();
    
    state=state;
    pulse_us=echo_us;
    if(state==hcsr04_ready){
        state=hcsr04_idle;
    }
    
    if(primask==0) 
    __enable_irq();

    if(state==hcsr04_ready){
        *distance_cm=pulse_us*HCSR04_CM_PER_US;
        return HAL_OK;
    }
    if((state==hcsr04_rise||state==hcsr04_fall)&&(HAL_GetTick()-trigger_tick>=HCSR04_ECHO_TIMEOUT_MS)){
        state=hcsr04_idle;
        return HAL_TIMEOUT;
    }
    return HAL_BUSY;
}

//定时器输入捕获中断回调函数
void HAL_TIM_IC_CaptureCallback(TIM_HandleTypeDef *htim)
{
    uint16_t fall_tick;

    if (htim->Instance!=TIM9){
        return;
    }
    if(htim->Channel==HAL_TIM_ACTIVE_CHANNEL_1&&state==hcsr04_rise){
        rise_tick=(uint16_t)HAL_TIM_ReadCapturedValue(htim,TIM_CHANNEL_1);//上升沿捕获
        state=hcsr04_fall;
    }
    else if(htim->Channel==HAL_TIM_ACTIVE_CHANNEL_2&&state==hcsr04_fall){
        fall_tick=(uint16_t)HAL_TIM_ReadCapturedValue(htim,TIM_CHANNEL_2);//下降沿捕获
        echo_us=(uint16_t)(fall_tick-rise_tick);//定时器计数
        state=hcsr04_ready;
    }
}
