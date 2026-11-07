#ifndef MISSION_H_
#define MISSION_H_

#include "main.h"
#include "key.h"

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
    state1_tofinish,
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

typedef enum
{
    state3_line,
    state3_c_lockstraight1,
    state3_c_lockstraight2,
    state3_line2,
    state3_baidong,
    state3_tofinish,
    state3_stop,
    state3_fault
}trackstate3;

uint8_t mission_report_snapshot(keymode *mode, trackpattern *pattern,
                                trackstate1 *state1, trackstate2 *state2, trackstate3 *state3);
void mission_reset(void);
HAL_StatusTypeDef mission_update(keymode mode);
#endif
