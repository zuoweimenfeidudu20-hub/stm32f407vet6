#ifndef HCSR04_H_
#define HCSR04_H_

#include "main.h"

/* Call after MX_GPIO_Init() and MX_TIM9_Init(). TIM9 is dedicated to ECHO. */
HAL_StatusTypeDef hcsr04_init(void);

/* Send a 10 us pulse on PC3. Returns HAL_BUSY while a measurement is in
 * progress or until 60 ms have elapsed since the previous trigger. */
HAL_StatusTypeDef hcsr04_trigger(void);

/* Read one completed measurement in centimetres. HAL_BUSY means the result
 * is not ready; HAL_TIMEOUT means no complete echo arrived within 40 ms. */
HAL_StatusTypeDef hcsr04_read_cm(float *distance_cm);

#endif /* HCSR04_H_ */
