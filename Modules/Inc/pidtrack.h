#ifndef MODULES_PIDTRACK_H_
#define MODULES_PIDTRACK_H_

#include "main.h"

#define GRAY_PD9555_ADDRESS (0x20U << 1)
#define GRAY_SENSOR_COUNT 12
#define GRAY_SENSOR_MASK  0x0FFFU

#define PIDTRACK_BASE_SPEED       15
#define kp               1
#define ki               0
#define kd               0
#define track_scale            100
#define integral_limit   500
#define output_limit     10
#define lost_ticks 300

typedef struct {
    int pgain;
    int igain;
    int dgain;

    int integral;
    int previous_error;
    
    int int_limit;
    int out_limit;
}track_pid;

HAL_StatusTypeDef gray_init(void);
HAL_StatusTypeDef gray_read(uint16_t *sensor_state);
HAL_StatusTypeDef pidtrack_update(void);
void pidtrack_rst(void);
void set_target(int base_speed, int correction);
void trackpid_rst(track_pid *pid, int error);
int line_pid(track_pid *pid, int error);
#endif /* MODULES_GRAY_H_ */
