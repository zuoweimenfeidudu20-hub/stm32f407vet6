#include "mission.h"
#include "pidtrack.h"
#include "pidmotor.h"
#include "hwt101.h"
#include "usart.h"
#include <stdio.h>

#undef lost_ticks

#define base_pwm 15
#define base_pwm2 13
#define base_pwm3 15

#define LOST_LIMIT 300U

#define pwm_limit1 10
#define pwm_limit2 10
#define pwm_limit3 10

#define CENTER_MASK         ((1U << 5) | (1U << 6))
#define OUTER_MASK          ((1U << 0) | (1U << 1) | (1U << 10) | (1U << 11))
#define SENSOR_CONFIRM_FRAMES 4

static track_pid control1 = {
    .pgain = 2,
    .igain = 0,
    .dgain = 1,
    .integral = 0,
    .previous_error = 0,
    .int_limit = 500,
    .out_limit = pwm_limit1,
};

static track_pid control2 = {
    .pgain = 1,
    .igain = 0,
    .dgain = 1,
    .integral = 0,
    .previous_error = 0,
    .int_limit = 500,
    .out_limit = pwm_limit2
};

static track_pid control3={
    .pgain=2,
    .igain=0,
    .dgain=1,
    .integral=0,
    .previous_error=0,
    .int_limit=500,
    .out_limit=pwm_limit3,
};

static trackstate1 current_state1;
static trackstate2 current_state2;
static trackstate3 current_state3;

static keymode previous_mode = key4_idle;
static int current_cnt, recover_ticks, center_white_cnt;
static uint8_t cross_stage1, cross_stage2, center_black, lost_active;
static int last_pwm, last_correction;
static int lost_ticks = 0;
static int center_black_cnt = 0;

static float state3_degl=5.0f;
static float state3_degr=10.0f;
volatile int cnt=0;
static uint8_t state3_phase;
static float state3_start_yaw;
static uint8_t state3_start_yaw_valid;
static uint8_t state3_settling;
static uint32_t state3_settling_tick;

static float state2_deg1=40.0f; //1.第一次右转进入楼梯
static float state2_deg2=85.0f; //1.楼梯返回
static float state2_deg3=35.0f; //1.第二次左转进入楼梯
static float state2_deg4=90.0f; //1.第二次右转出楼梯


static trackpattern report_pattern;
static uint8_t report_valid;
static uint8_t r90_turn_active;
static uint8_t r90_latched;
static uint8_t r90_clear_frames;

uint8_t mission_report_snapshot(keymode *mode, trackpattern *pattern,
                                trackstate1 *state1, trackstate2 *state2, trackstate3 *state3)
{
    if (!report_valid || mode == NULL || pattern == NULL ||
        state1 == NULL || state2 == NULL || state3 == NULL) return 0U;

    *mode = previous_mode;
    *pattern = report_pattern;
    *state1 = current_state1;
    *state2 = current_state2;
    *state3 = current_state3;
    return 1U;
}


static void condition_rst1(trackstate1 next, int error);
static void condition_rst2(trackstate2 next, int error);
static void condition_rst3(trackstate3 next, int error);
static uint8_t track_lost(trackpattern pattern);
void send_newtarget(int speed, int turn);

static uint8_t track_r90(keymode mode, trackpattern pattern, int error)
{
    int result;
    int following = (mode == key1_putong && current_state1 == state1_tofinish) ||
                    (mode == key2_teshu && current_state2 == state2_stair4) ||
                    (mode == key3_baidong && current_state3 == state3_tofinish);

    if (r90_turn_active)
    {
        result = hwt101_turn_result();
        if (result < 0)
        {
            r90_turn_active = 0U;
            if (mode == key1_putong) condition_rst1(state1_fault, error);
            else if (mode == key2_teshu) condition_rst2(state2_fault, error);
            else condition_rst3(state3_fault, error);
            send_newtarget(0, 0);
        }
        else if (result > 0)
        {
            r90_turn_active = 0U;
            current_cnt = 0;
            trackpid_rst(mode == key1_putong ? &control1 : mode == key2_teshu ? &control2 : &control3, error);
            send_newtarget(0, 0);
        }
        else
            send_newtarget(mode == key3_baidong ? base_pwm3 : base_pwm2, mode == key3_baidong ? pwm_limit3 : pwm_limit2);
        return 1U;
    }

    if (r90_latched)
    {
        if (pattern == pattern_line)
        {
            if (++r90_clear_frames >= 3U) {
                r90_latched = 0U;
                r90_clear_frames = 0U;
            }
        }
        else r90_clear_frames = 0U;
    }
    if (!following || r90_latched || pattern != pattern_r90) return 0U;

    r90_latched = 1U;
    current_cnt = 0;
    lost_active = 0U;
    lost_ticks = 0;
    recover_ticks = 0;
    if (hwt101_turn_begin(90.0f, 5000U) != HAL_OK)
    {
        if (mode == key1_putong) condition_rst1(state1_fault, error);
        else if (mode == key2_teshu) condition_rst2(state2_fault, error);
        else condition_rst3(state3_fault, error);
        send_newtarget(0, 0);
    }
    else
    {
        r90_turn_active = 1U;
        send_newtarget(mode == key3_baidong ? base_pwm3 : base_pwm2, mode == key3_baidong ? pwm_limit3 : pwm_limit2);
    }
    return 1U;
}

static int center_white_confirm(uint16_t bits)
{
    uint16_t mid = bits & CENTER_MASK;

    /* 先连续确认两侧四白、中间双黑。 */
    if (!center_black)
    {
        center_white_cnt = 0;

        if ((bits & OUTER_MASK) == 0U && mid == CENTER_MASK)
        {
            if (++center_black_cnt >= SENSOR_CONFIRM_FRAMES)
                center_black = 1;
        }
        else
        {
            center_black_cnt = 0;
        }

        return 0;
    }

    /* 再连续确认两侧四黑、中间双白。 */
    if ((bits & OUTER_MASK) != OUTER_MASK || mid != 0U)
    {
        center_white_cnt = 0;
        return 0;
    }

    if (++center_white_cnt < SENSOR_CONFIRM_FRAMES)
        return 0;

    /* 本次事件只报告一次，之后重新等待初始状态。 */
    center_black = 0;
    center_black_cnt = 0;
    center_white_cnt = 0;
    return 1;
}

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

    center_black = ((sensor_state & CENTER_MASK) != 0U);
    // 中间两个传感器任意一个检测到黑色即可
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

// 转向约定：正值右转，负值左转
void send_newtarget(int speed, int turn)
{
    last_pwm = speed;
    last_correction = turn;
    set_target(speed, turn);
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
            condition_rst1(state1_tofinish, error);
        }
        break;

    case state1_tofinish:
        send_newtarget(base_pwm, line_pid(&control1, error));
        break;

    default:
        set_target(0, 0);
        break;
    }
}

static void track_run_abnormal(trackpattern pattern, int error, uint16_t bits)
{
    int result;
    int check_lost = current_state2 == state2_line || current_state2 == state2_stair1 || current_state2 == state2_stair3 || current_state2 == state2_teshu_lockright2 || current_state2 == state2_stair4;
    
    if (check_lost&&track_lost(pattern))
    {
        return;
    }
    
    switch (current_state2)
    {
    case state2_line:
        if(cross_stage2!=1){
            condition_rst2(state2_fault,error);
            send_newtarget(0,0);
            break;
        }

        if(track_confirm(pattern==pattern_rightbranch,2))
        {//右转时 yaw 增加
            if(hwt101_turn_begin(state2_deg1,2500U)!=HAL_OK)
            {
            condition_rst2(state2_fault,error);
            send_newtarget(0, 0);
            break;
            }
            condition_rst2(state2_c_lockright, error);
            send_newtarget(base_pwm2, pwm_limit2);
        }else{
            send_newtarget(base_pwm2,line_pid(&control2,error));
        }
        break;

    
case state2_c_lockright:
{
    result=hwt101_turn_result();

    if(result<0)
    {
        condition_rst2(state2_fault, error);
        send_newtarget(0, 0);
    }
    else if(result==1)
    {
        cross_stage2=2;
        condition_rst2(state2_stair1,error);
        send_newtarget(base_pwm2,line_pid(&control2,error));
    }
    else
    {
        send_newtarget(base_pwm2,pwm_limit2);
    }
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
        if(center_white_confirm(bits))
        {
            if(hwt101_turn_begin(-state2_deg2,5000u)!=HAL_OK){
                condition_rst2(state2_fault,error);
                send_newtarget(0,0);
                break;
            }
            condition_rst2(state2_teshu_lockleft1, error);
            send_newtarget(base_pwm2,-pwm_limit2);
        }
        break;
    
    case state2_teshu_lockleft1:
        if (cross_stage2!=2)
        {
            condition_rst2(state2_fault, error);
            send_newtarget(0, 0);
            break;
        }

        result=hwt101_turn_result();

        if(result<0)//陀螺仪参数无效/超时
        {
            condition_rst2(state2_fault, error);
            send_newtarget(0, 0);
            break;
        }
        else if(result==1)
        {
            cross_stage2=3;
            condition_rst2(state2_stair2,error);
            send_newtarget(base_pwm2,line_pid(&control2,error));
            break;
        }else{
            send_newtarget(base_pwm2,-pwm_limit2);
        }
        break;

    case state2_stair2:
        if(cross_stage2!=3)
        {
            condition_rst2(state2_fault, error);
            send_newtarget(0, 0);
            break;
        }
        if (track_confirm(pattern==pattern_leftbranch,3))
        {
            if(hwt101_turn_begin(-state2_deg3,5000u)!=HAL_OK){
                condition_rst2(state2_fault,error);
                send_newtarget(0,0);
                break;
            }
            condition_rst2(state2_branch_lockleft2, error);
            send_newtarget(base_pwm2,-pwm_limit2);
        }else{
            send_newtarget(base_pwm2,line_pid(&control2,error));
        }
        break;
    
    case state2_branch_lockleft2:
        if (cross_stage2!=3)
        {
            condition_rst2(state2_fault, error);
            send_newtarget(0, 0);
            break;
        }

        result=hwt101_turn_result();

        if(result<0)//陀螺仪参数无效/超时
        {
            condition_rst2(state2_fault, error);
            send_newtarget(0, 0);
            break;
        }else if(result==1)
        {
            cross_stage2=4;
            condition_rst2(state2_stair3,error);
            send_newtarget(base_pwm2,line_pid(&control2,error));
            break;
        }else{
            send_newtarget(base_pwm2,-pwm_limit2);
        }
        break;

    case state2_stair3:
        if(cross_stage2!=4)
        {
            condition_rst2(state2_fault, error);
            send_newtarget(0, 0);
            break;
        }
        if(center_white_confirm(bits))
        {
            if(hwt101_turn_begin(state2_deg4,5000u)!=HAL_OK){
                condition_rst2(state2_fault,error);
                send_newtarget(0,0);
                break;
            }

            condition_rst2(state2_teshu_lockright2, error);
            send_newtarget(base_pwm2,pwm_limit2);
        }
        else{
            send_newtarget(base_pwm2,line_pid(&control2,error));
        }
        break;
    
    case state2_teshu_lockright2:
        if (cross_stage2 != 4)
        {
            condition_rst2(state2_fault, error);
            send_newtarget(0, 0);
            break;
        }

        result=hwt101_turn_result();
        
        if(result<0){
            condition_rst2(state2_fault,error);
            send_newtarget(0,0);
        }else if(result==1){
            cross_stage2=5;
            condition_rst2(state2_stair4,error);
            send_newtarget(base_pwm2,line_pid(&control2,error));
        }else{
            send_newtarget(base_pwm2,pwm_limit2);
        }
        break;

    case state2_stair4:
        if(cross_stage2!=5)
        {
            condition_rst2(state2_fault, error);
            send_newtarget(0, 0);
            break;
        }
        send_newtarget(base_pwm2,line_pid(&control2,error));
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

static void track_run_baidong(trackpattern pattern, int error, uint16_t bits)
{
    float yaw;
    float yaw_difference;
    uint32_t yaw_age;
    (void)bits;
    if (state3_settling)
    {
        if ((HAL_GetTick() - state3_settling_tick) >= 1000U)
        {
            state3_settling = 0U;
            condition_rst3(state3_baidong, error);
            if (hwt101_turn_begin(-state3_degl, 5000U) != HAL_OK)
            {
                cnt = 0;
                current_state3 = state3_fault;
                send_newtarget(0, 0);
            }
            else
            {
                state3_phase = 1U;
                send_newtarget(base_pwm3, -pwm_limit3);
            }
        }
        else if (!track_lost(pattern))
            send_newtarget(base_pwm3, line_pid(&control3, error));
        return;
    }
    if (current_state3 == state3_line || current_state3 == state3_c_lockstraight1 ||
        current_state3 == state3_c_lockstraight2 || current_state3 == state3_line2)
    {
        if (!hwt101_get_yaw(&yaw, &yaw_age) || yaw_age > 100U)
        {
            condition_rst3(state3_fault, error);
            send_newtarget(0, 0);
            return;
        }
        if (!state3_start_yaw_valid)
        {
            state3_start_yaw = yaw;
            state3_start_yaw_valid = 1U;
        }
        yaw_difference = hwt101_angle_error(yaw, state3_start_yaw);
        if (yaw_difference >= 178.0f || yaw_difference <= -178.0f)
        {
            current_state3 = state3_line2;
            state3_settling = 1U;
            state3_settling_tick = HAL_GetTick();
            if (!track_lost(pattern))
                send_newtarget(base_pwm3, line_pid(&control3, error));
            return;
        }
    }
    if (current_state3 != state3_baidong && current_state3 != state3_stop &&
        current_state3 != state3_fault && track_lost(pattern))
    {
        return;
    }
    switch (current_state3)
    {
    case state3_line:
        if (track_confirm(pattern == pattern_rightbranch, 3))
        {
            condition_rst3(cross_stage1 == 0 ? state3_c_lockstraight1 : state3_c_lockstraight2, error);
            send_newtarget(base_pwm3, 0);
        }
        else
            send_newtarget(base_pwm3, line_pid(&control3, error));
        break;
    
    case state3_c_lockstraight1:
        send_newtarget(base_pwm3,0);
        if (track_confirm(pattern==pattern_line,3))
        {
            cross_stage1=1;
            condition_rst3(state3_line, error);
        }
        break;
    
    case state3_c_lockstraight2:
        send_newtarget(base_pwm3, 0);
        if (track_confirm(pattern==pattern_line,3))
        {
            cross_stage1=0;
            condition_rst3(state3_line2, error);
        }
        break;

    case state3_line2:
        send_newtarget(base_pwm3, line_pid(&control3, error));
        break;
    
    case state3_baidong:
    {
        /* 0: start left, 1: turning left, 2: turning right. */
        int result;

        if (state3_phase == 0U)
        {
            if (hwt101_turn_begin(-state3_degl, 5000U) != HAL_OK)
            {
                cnt = 0;
                current_state3 = state3_fault;
                send_newtarget(0, 0);
                break;
            }
            state3_phase = 1U;
            send_newtarget(base_pwm3, -pwm_limit3);
            break;
        }

        result = hwt101_turn_result();
        if (result < 0)
        {
            state3_phase = 0U;
            cnt = 0;
            current_state3 = state3_fault;
            send_newtarget(0, 0);
        }
        else if (result == 0)
        {
            send_newtarget(base_pwm3, state3_phase == 1U ? -pwm_limit3 : pwm_limit3);
        }
        else if (state3_phase == 1U)
        {
            if (hwt101_turn_begin(state3_degr, 5000U) != HAL_OK)
            {
                state3_phase = 0U;
                cnt = 0;
                current_state3 = state3_fault;
                send_newtarget(0, 0);
                break;
            }
            state3_phase = 2U;
            send_newtarget(base_pwm3, pwm_limit3);
        }
        else if (++cnt >= 5)
        {
            state3_phase = 0U;
            cnt = 0;
            current_state3 = state3_tofinish;
            trackpid_rst(&control3, error);
            send_newtarget(base_pwm3, line_pid(&control3, error));
        }
        else
        {
            state3_phase = 0U;
            send_newtarget(0, 0);
        }
        break;
    }

    case state3_tofinish:
        send_newtarget(base_pwm3, line_pid(&control3, error));
        break;

    case state3_stop:
    case state3_fault:
        send_newtarget(0,0);
        break;

    default:
        condition_rst3(state3_stop, error);
        set_target(0, 0);
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
    center_black_cnt = 0;
    center_white_cnt = 0;

    trackpid_rst(&control2, error);
}

static void condition_rst3(trackstate3 next,int error)
{
    current_state3=next;
    current_cnt=0;
    trackpid_rst(&control3,error);
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
            current_state3 = state3_stop;
            set_target(0, 0);
        }
        else
        {
            set_target(last_pwm, last_correction);
        }
        return 1;
    }
    return 0;
}

void mission_reset(void)
{
    report_valid = 0U;
    r90_turn_active = 0U;
    r90_latched = 0U;
    r90_clear_frames = 0U;
    current_state1=state1_line;
    current_state2=state2_line;
    current_state3=state3_line;
    state3_phase=0U;
    state3_start_yaw_valid=0U;
    state3_settling=0U;
    cnt=0;

    cross_stage1=0;
    cross_stage2=1;  // state2_line 的进入条件要求它等于 1

    current_cnt=0;
    recover_ticks=0;//
    center_white_cnt=0;//
    lost_ticks=0;

    center_black = 0;
    center_black_cnt = 0;
    lost_active = 0;

    last_pwm = 0;
    last_correction = 0;

    trackpid_rst(&control1, 0);
    trackpid_rst(&control2, 0);
    trackpid_rst(&control3, 0);

    set_target(0, 0);
}

HAL_StatusTypeDef mission_update(keymode mode)
{
    uint16_t bits;
    int error;
    trackpattern pattern;
    HAL_StatusTypeDef status;
    report_valid = 0U;
    if (mode != previous_mode)
    {
        mission_reset();
        previous_mode = mode;
    }
    if (mode!=key1_putong&&mode!=key2_teshu&&mode!=key3_baidong)
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
    if (pattern == pattern_r90 &&
        !((mode == key1_putong && current_state1 == state1_tofinish) ||
          (mode == key2_teshu && current_state2 == state2_stair4) ||
          (mode == key3_baidong && current_state3 == state3_tofinish)))
        pattern = pattern_rightbranch;
    if (track_r90(mode, pattern, error))
    {
        /* The turn temporarily takes control of the motors. */
    }
    else if (mode == key1_putong)
        track_run_normal(pattern, error);
    else if (mode == key2_teshu)
        track_run_abnormal(pattern, error, bits);
    else
        track_run_baidong(pattern, error, bits);
    report_pattern = pattern;
    report_valid = 1U;
    return HAL_OK;
}
