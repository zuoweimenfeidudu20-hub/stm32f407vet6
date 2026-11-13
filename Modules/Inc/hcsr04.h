#ifndef HCSR04_H_
#define HCSR04_H_

#include "main.h"

HAL_StatusTypeDef hcsr04_init(void);
HAL_StatusTypeDef hcsr04_trigger(void);
HAL_StatusTypeDef read_cm(float *distance_cm);

#endif /* HCSR04_H_ */
