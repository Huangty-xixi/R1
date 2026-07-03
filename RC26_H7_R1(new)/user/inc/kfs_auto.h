#ifndef __KFS_AUTO_H__
#define __KFS_AUTO_H__

#include "stdint.h"

/* ===== 状态枚举 ===== */
typedef enum {
    KFS_AUTO_IDLE    = 0,
    KFS_AUTO_PUMP_ON = 1,
    KFS_AUTO_POSE    = 2,
    KFS_AUTO_PUSH    = 3,
    KFS_AUTO_RETRACT = 4,
    KFS_AUTO_DONE    = 5,
} Kfs_Auto_State;

/* ===== 前推参数 ===== */
typedef struct {
    float s_start;
    float s_target;
    float s_step;
} Kfs_Auto_PushParam;

/* ===== 姿态高度参数 ===== */
typedef struct {
    float h_low;
    float h_mid;
    float h_high;
    float pose_s;
    uint8_t height_sel;
    float pose_tol;
} Kfs_Auto_PoseParam;

/* ===== 回收参数 ===== */
typedef struct {
    float th1;
    float th2;
    float target2;
    float arm1_mid;
    float arm1_deep;
    float arm3_target;
    float kp2_s1;
    float kd2_s1;
    float kp1_s2;
    float kd1_s2;
    float kp2_s2;
    float kd2_s2;
    float done_tol;
} Kfs_Auto_RetractParam;

/* ===== 顶层模块结构体 ===== */
typedef struct {
    Kfs_Auto_State        state;
    Kfs_Auto_PushParam    push;
    Kfs_Auto_PoseParam    pose;
    Kfs_Auto_RetractParam retract;

    uint8_t  debug_step;
    uint8_t  auto_run;
    uint16_t pump_delay_ms;

    float    cur_s;       /* 内部，只读 */
} Kfs_Auto_Module;

extern Kfs_Auto_Module kfs_auto;

void KFS_Auto_Init(void);
void KFS_Auto_Process(void);

#endif
