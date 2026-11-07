#ifndef MOTOR_H_
#define MOTOR_H_

#include <stdint.h>
#include "main.h"

#define MOTOR_LEFT_PWM1_CHANNEL  TIM_CHANNEL_1 /* PE9 */
#define MOTOR_LEFT_PWM2_CHANNEL  TIM_CHANNEL_2 /* PE11 */
#define MOTOR_RIGHT_PWM1_CHANNEL TIM_CHANNEL_3 /* PE13 */
#define MOTOR_RIGHT_PWM2_CHANNEL TIM_CHANNEL_4 /* PE14 */

HAL_StatusTypeDef motor_init(void);
void rightmotor_pwm(int percent);
void leftmotor_pwm(int percent);
void rightmotor_stop(void);
void rightmotor_forward(int speed);
void rightmotor_reverse(int speed);
void leftmotor_stop(void);
void leftmotor_forward(int speed);
void leftmotor_reverse(int speed);
void allstop(void);
int motor_clamp(int percent);
void rightmotor_output(int output);
void leftmotor_output(int output);
extern int Clamp(int32_t value, int32_t min, int32_t max);

#endif
