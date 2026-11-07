#ifndef PIDMOTOR_H_
#define PIDMOTOR_H_

#include "main.h"

#define pid_motor_gain_scale 100
#define motor_kp 250
#define motor_ki 3
#define motor_kd 0
#define integral_limit 500
#define max_count_10ms 100
#define encoder_left_dir -1
#define encoder_right_dir 1

typedef struct {
    int integral;
    int previous_error;
    int previous_target;
} PIDMotor_Controller;

typedef struct {
    int lcnt;
    int rcnt;
}encoder_cnts;

HAL_StatusTypeDef encoder_init(void);
void encoder_update(void);
encoder_cnts get_encodercnt(void);

extern PIDMotor_Controller right_pid;
extern PIDMotor_Controller left_pid;

extern volatile int init_rightpwm;
extern volatile int init_leftpwm;
extern volatile int right_count_10ms;
extern volatile int left_count_10ms;
extern volatile int right_output;
extern volatile int left_output;


void encoder_update(void);
encoder_cnts get_encodercnt(void);
extern int Clamp(int32_t value, int32_t min, int32_t max);
int Sign(int32_t value);
extern int PercentToCounts(int32_t percent);
void pidmotor_rst(PIDMotor_Controller *controller);
int pid_motor(PIDMotor_Controller *controller, int target_percent,int target_counts, int measured_counts);
void pidmotor_update(void);
void pidmotor_init(void);
#endif
