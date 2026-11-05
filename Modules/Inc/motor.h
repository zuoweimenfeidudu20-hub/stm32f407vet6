#ifndef MOTOR_H_
#define MOTOR_H_

#include <stdint.h>
#include "main.h"

#define MOTOR_LEFT_PWM1_CHANNEL  TIM_CHANNEL_1 /* PE9 */
#define MOTOR_LEFT_PWM2_CHANNEL  TIM_CHANNEL_2 /* PE11 */
#define MOTOR_RIGHT_PWM1_CHANNEL TIM_CHANNEL_3 /* PE13 */
#define MOTOR_RIGHT_PWM2_CHANNEL TIM_CHANNEL_4 /* PE14 */

HAL_StatusTypeDef motor_init(void);
void rightmotor_pwm(uint32_t percent);
void leftmotor_pwm(uint32_t percent);
void rightmotor_stop(void);
void rightmotor_forward(uint32_t speed);
void rightmotor_reverse(uint32_t speed);
void leftmotor_stop(void);
void leftmotor_forward(uint32_t speed);
void leftmotor_reverse(uint32_t speed);
void allstop(void);
int32_t motor_clamp(int32_t percent);
void rightmotor_output(int32_t output);
void leftmotor_output(int32_t output);

#endif
