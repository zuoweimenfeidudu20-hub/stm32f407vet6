#include "mission.h"
#include "pidtrack.h"
#include "pidmotor.h"
#include "hwt101.h"

#undef lost_ticks

#define base_pwm 15
#define base_pwm2 13
#define LOST_LIMIT 300U
#define pwm_limit1 10
#define pwm_limit2 5

static track_pid control1 = {
    .pgain = 2,
    .igain = 0,
    .dgain = 0,
    .integral = 0,
    .previous_error = 0,
    .int_limit = 500,
    .out_limit = pwm_limit1,
};

static track_pid control2 = {
    .pgain = 2,
    .igain = 0,
    .dgain = 0,
    .integral = 0,
    .previous_error = 0,
    .int_limit = 500,
    .out_limit = pwm_limit2
};

typedef enum
{
    pattern_line,
    pattern_allblack,
    pattern_rightbranch,
    pattern_leftbranch,
    pattern_r90,
    pattern_lost
} trackpattern;

typedef enum
{
    state1_line,
    state1_c_lockstraight1,
    state1_c_lockstraight2,
    state1_stop,
    state1_fault
} trackstate1;

typedef enum
{
    state2_line,
    state2_c_lockright,//2:锁定右转
    state2_stair1,//1楼梯内部普通巡线
    state2_teshu_lockleft1,//第一次中间识别到白色，锁定左转直到最右边传感器识别到黑线
    state2_stair2,//返回时普通循迹
    state2_branch_lockleft2,//第二次识别到左边黑色延申，锁定左转直到右边传感器识别到黑线
    state2_stair3,//2楼梯线路过程中的普通循迹
    state2_teshu_lockright2,//中间识别到白色，锁定右转
    state2_stair4,//直到最后普通循迹（这里需要直角弯结合陀螺仪进行90°转弯）
    state2_stop,
    state2_fault,
} trackstate2;

static trackstate1 current_state1;
static trackstate2 current_state2;
static keymode previous_mode = key4_idle;
static int current_cnt, recover_ticks, center_white_cnt;
static uint8_t cross_stage1, cross_stage2, center_black, lost_active;
static int last_pwm, last_correction;
static int lost_ticks = 0;

static void condition_rst1(trackstate1 next, int error);
static void condition_rst2(trackstate2 next, int error);
static uint8_t track_lost(trackpattern pattern);

//滤波
static int track_confirm(int condition, int required)
{
    if (!condition)
    {
        current_cnt = 0;
    }
    else if (current_cnt < required)
    {
        current_cnt++;
    }
    return current_cnt >= required;
}

// 分析当前地图状态
static trackpattern track_analyze(uint16_t sensor_state, int *error)
{
    static const int16_t sensor_weight[GRAY_SENSOR_COUNT] = {
        -1600, -1200, -850, -550, -300, -100, 100, 300, 550, 850, 1200, 1600};
    int weighted_sum = 0;
    int black_cnt = 0;
    int left_cnt = 0;
    int right_cnt = 0;
    int center_black;

    center_black = ((sensor_state & ((1U << 5) | (1U << 6))) == ((1U << 5) | (1U << 6)));
    // 1&1为1，其余状态全为0
    for (int i = 0; i < GRAY_SENSOR_COUNT; i++)
    {
        if ((sensor_state & (1U << i)) != 0U)
        {
            weighted_sum += sensor_weight[i];
            black_cnt++;
            if (i <= 3)
            {
                left_cnt++;
            }
            if (i >= 8)
            {
                right_cnt++;
            }
        }
    }
    if (black_cnt == 0)
    {
        *error = 0;
        return pattern_lost;
    }
    *error = weighted_sum / black_cnt;

    if (black_cnt == GRAY_SENSOR_COUNT)
    {
        return pattern_allblack;
    }

    if (center_black && right_cnt >= 4 && left_cnt == 0)
    {
        return pattern_r90;
    }

    if (center_black && right_cnt >= 2 && left_cnt == 0)
    {
        return pattern_rightbranch;
    }

    if (center_black && left_cnt >= 2 && right_cnt == 0)
    {
        return pattern_leftbranch;
    }

    return pattern_line;
}

// 丢线逻辑   -值右转  +值左转
void send_newtarget(int speed, int turn)
{
    last_pwm = speed;
    last_correction = turn;
    set_target(speed, -turn);
}

static int center_white_confirm(uint16_t bits)
{
    if ((bits & ((1U << 5) | (1U << 6))) == ((1U << 5) | (1U << 6)))
    {
        center_black = 1;
        center_white_cnt = 0;
    }
    else if (center_black && ++center_white_cnt >= 4U)
    {
        center_black = 0;
        center_white_cnt = 0;
        return 1;
    }
    return 0;
}

static void track_run_normal(trackpattern pattern, int error)
{
    if (track_lost(pattern))
    {
        return;
    }
    switch (current_state1)
    {
    case state1_line:
        if (track_confirm(pattern == pattern_rightbranch, 3))
        {
            condition_rst1(cross_stage1 == 0 ? state1_c_lockstraight1 : state1_c_lockstraight2, error);
            send_newtarget(base_pwm, 0);
        }
        else
            send_newtarget(base_pwm, line_pid(&control1, error));
        break;
    case state1_c_lockstraight1:
        send_newtarget(base_pwm, 0);
        if (track_confirm(pattern == pattern_line, 3))
        {
            cross_stage1 = 1;
            condition_rst1(state1_line, error);
        }
        break;
    case state1_c_lockstraight2:
        send_newtarget(base_pwm, 0);
        if (track_confirm(pattern == pattern_line, 3))
        {
            cross_stage1 = 0;
            condition_rst1(state1_line, error);
        }
        break;

    default:
        set_target(0, 0);
        break;
    }
}

static void track_run_abnormal(trackpattern pattern, int error, uint16_t bits)
{
    int check_lost = current_state2 == state2_line || current_state2 == state2_stair1 || current_state2 == state2_stair3 || current_state2 == state2_teshu_lockright2 || current_state2 == state2_stair4;
    
    if (check_lost&&track_lost(pattern))
    {
        return;
    }
    
    switch (current_state2)
    {
    case state2_line:
        if(track_confirm(cross_stage2==1&&pattern==pattern_rightbranch,3))
        {
            if (hwt101_turn_right_35(1U)<0)
            {
                condition_rst2(state2_fault, error);
                send_newtarget(0, 0);
                break;
            }
            condition_rst2(state2_c_lockright, error);
            send_newtarget(base_pwm2,-pwm_limit2);
        }
        else
        {
            send_newtarget(base_pwm2,line_pid(&control2,error));
        };
        break;

    case state2_c_lockright:
    {
        if (cross_stage2!=1)
        {
            condition_rst2(state2_fault, error);
            send_newtarget(0, 0);
            break;
        }

        int result=hwt101_turn_right_35(0U);
        if(result<0)//陀螺仪参数无效/超时
        {
            condition_rst2(state2_fault, error);
            send_newtarget(0, 0);
            break;
        }
        if(result==1)
        {
            cross_stage2=2;
            condition_rst2(state2_stair1, error);
            send_newtarget(base_pwm2,line_pid(&control2,error));
            break;
        }
        send_newtarget(base_pwm2,-pwm_limit2);
        break;
    }
    
    case state2_stair1:
        if (cross_stage2!=2)
        {
            condition_rst2(state2_fault, error);
            send_newtarget(0, 0);
            break;
        }
        send_newtarget(base_pwm2,line_pid(&control2,error));
        if(track_confirm(center_white_confirm(bits),3))
        {
            condition_rst2(state2_teshu_lockleft1, error);
            send_newtarget(base_pwm2,pwm_limit2);
        }
        break;
    
    case state2_teshu_lockleft1:
        if (cross_stage2!=2)
        {
            condition_rst2(state2_fault, error);
            send_newtarget(0, 0);
            break;
        }
        
        send_newtarget(base_pwm2,pwm_limit2);
        int result=hwt101_turn_right_35(0U);
        if(result<0)//陀螺仪参数无效/超时
        {
            condition_rst2(state2_fault, error);
            send_newtarget(0, 0);
            break;
        }
        if(result==1)
        {
            cross_stage2=3;
            condition_rst2(state2_stair2,error);
            send_newtarget(base_pwm2,line_pid(&control2,error));
            break;
        }
        send_newtarget(base_pwm2,pwm_limit2);
        break;

    case state2_stair2:
        if(cross_stage2!=3)
        {
            condition_rst2(state2_fault, error);
            send_newtarget(0, 0);
            break;
        }
        send_newtarget(base_pwm2,line_pid(&control2,error));
        if (track_confirm(pattern==pattern_leftbranch,3))
        {
            condition_rst2(state2_branch_lockleft2, error);
            send_newtarget(base_pwm2,pwm_limit2);
        }
        break;
    
    case state2_branch_lockleft2:
        if (cross_stage2!=3)
        {
            condition_rst2(state2_fault, error);
            send_newtarget(0, 0);
            break;
        }
        send_newtarget(base_pwm2,pwm_limit2);
        int result=hwt101_turn_right_35(0U);
        if(result<0)//陀螺仪参数无效/超时
        {
            condition_rst2(state2_fault, error);
            send_newtarget(0, 0);
            break;
        }
        if(result==1)
        {
            cross_stage2=4;
            condition_rst2(state2_stair3,error);
            send_newtarget(base_pwm2,line_pid(&control2,error));
            break;
        }
        send_newtarget(base_pwm2,pwm_limit2);
        break;

    case state2_stair3:
        if(cross_stage2!=4)
        {
            condition_rst2(state2_fault, error);
            send_newtarget(0, 0);
            break;
        }
        if (track_confirm(pattern == pattern_leftbranch, 3))
        {
            condition_rst2(state2_branch_lockleft2, error);
            send_newtarget(base_pwm2, -pwm_limit2);
        }
        else
            send_newtarget(base_pwm2, correction(error, 10));
        break;
    
    case state2_teshu_lockright2:
        if (cross_stage2 != 4)
        {
            condition_rst2(state2_fault, error);
            send_newtarget(0, 0);
            break;
        }
        send_newtarget(base_pwm2, pwm_limit2);
        if (track_confirm(maxleft, 3))
        {
            cross_stage2 = 5;
            condition_rst2(state2_stair4, error);
        }
        break;
    case state2_stair4:
        if (cross_stage2 != 5)
        {
            condition_rst2(state2_fault, error);
            send_newtarget(0, 0);
            break;
        }
        send_newtarget(base_pwm2, correction(error, 10));
        break;
    
    case state2_stop:
    case state2_fault:
        send_newtarget(0, 0);
        break;
    
    default:
        condition_rst2(state2_fault,error);
        send_newtarget(0, 0);
        break;
    }
}

static void condition_rst1(trackstate1 next, int error)
{
    current_state1=next;
    current_cnt=0;
    trackpid_rst(&control1,error);
}

static void condition_rst2(trackstate2 next,int error)
{
    current_state2 = next;
    current_cnt = 0;

    center_black = 0;
    center_white_cnt = 0;

    trackpid_rst(&control2, error);
}

// 丢线逻辑
static uint8_t track_lost(trackpattern pattern)
{
    if (pattern == pattern_lost)
    {
        lost_active = 1;
        recover_ticks = 0;
    }
    else if (lost_active && recover_ticks++ >= 3)
    {
        lost_active = 0;
        recover_ticks = 0;
        lost_ticks = 0;
    }
    if (lost_active)
    {
        current_cnt = 0;
        if (lost_ticks++ >= LOST_LIMIT)
        {
            current_state1 = state1_stop;
            current_state2 = state2_stop;
            set_target(0, 0);
        }
        else
        {
            set_target(last_pwm, -last_correction);
        }
        return 1;
    }
    return 0;
}

void mission_reset(void)
{
    current_state1=state1_line;
    current_state2=state2_line;

    cross_stage1=0;
    cross_stage2=1;  // state2_line 的进入条件要求它等于 1

    current_cnt=0;
    recover_ticks=0;//
    center_white_cnt=0;//
    lost_ticks=0;

    center_black = 0;
    lost_active = 0;

    last_pwm = 0;
    last_correction = 0;

    trackpid_rst(&control1, 0);
    trackpid_rst(&control2, 0);

    set_target(0, 0);
}

HAL_StatusTypeDef mission_update(keymode mode)
{
    uint16_t bits;
    int error;
    trackpattern pattern;
    HAL_StatusTypeDef status;
    if (mode != previous_mode)
    {
        mission_reset();
        previous_mode = mode;
    }
    if (mode != key1_putong && mode != key2_teshu)
    {
        set_target(0, 0);
        return HAL_OK;
    }
    status = gray_read(&bits);
    if (status != HAL_OK)
    {
        set_target(0, 0);
        return status;
    }
    pattern = track_analyze(bits, &error);
    if (mode == key1_putong)
        track_run_normal(pattern, error);
    else
        track_run_abnormal(pattern, error, bits);
    return HAL_OK;
}
