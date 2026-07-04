#ifndef __PICK_KFS_AUTO_H__
#define __PICK_KFS_AUTO_H__

#include "arm.h"
#include "common.h"

typedef enum {
    PKFS_IDLE = 0,
    PKFS_SUCKER_ON,
    PKFS_ARM_POSE,
    PKFS_ARM_PUSH,
    PKFS_ARM_RETRACT,
    PKFS_DONE,
    PKFS_TIMEOUT,
} PickKfsState;

typedef struct {
    /* ==== 触发 (Keil watch 写 0/1) ==== */
    uint8_t  trigger;
    uint8_t  step_mode;
    uint8_t  step_next;
    uint8_t  abort;

    /* ==== 只读状态 ==== */
    PickKfsState state;
    uint16_t frame_cnt;
    uint8_t  retract_phase;

    /* ==== 高度选择 0=低(0.23) 1=中(0.435) 2=高(0.65) ==== */
    uint8_t  height_sel;

    /* ==== Step1: 吸盘延时 (ms) ==== */
    uint16_t sucker_delay_ms;

    /* ==== Step3: 前推 s 范围 [0]=起始 [1]=目标 [2]=每帧增量 ==== */
    float    push_s[3];

    /* ==== Step4: 回收完成条件 ==== */
    float    retract_done_pos;
} PickKfsDebug;

void PickKfsAuto_Init(void);
void PickKfsAuto_Run(void);
PickKfsState PickKfsAuto_State(void);
uint8_t PickKfsAuto_IsIdle(void);

#endif