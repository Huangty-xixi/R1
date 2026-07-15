#ifndef __AUTO_MODE_H__
#define __AUTO_MODE_H__

#include "main.h"
#include "math.h"
#include "chassis.h"
#include "lift.h"
#include "kfs.h"
#include "weapon.h"
#include "motor_control.h"
#include "arm.h"
#include "policy.h"
#include "tim.h"

extern uint8_t auto_flag;
extern uint8_t auto_init_flag;
extern uint8_t arm_s_running;

extern float mid_x;  //mid360����
extern float mid_y;
extern float mid_yaw;

extern float diff_x;
extern float diff_y;
extern float diff_w;

/* by deepseek: chassis center position calculated from radar ODOM + offset */
typedef struct {
    float center_x;       /* chassis center in world frame (m) */
    float center_y;
    float yaw_rad;        /* chassis yaw (radians) */
    float radar_wx;       /* radar origin in aligned world frame after transform */
    float radar_wy;
} ChassisCoord_t;

extern ChassisCoord_t chassis_coord;

/* by deepseek: compute chassis center from ODOM, call every cycle */
void chassis_coord_update(void);

/* by deepseek: debug flags, set in Keil debugger to trigger auto move */
extern volatile uint8_t debug_go_flag;      /* set to 1 → execute auto_inplace once */
extern volatile float   debug_target_x;     /* body-frame: forward (m) */
extern volatile float   debug_target_y;     /* body-frame: left (m) */
extern volatile float   debug_target_w;     /* body-frame: yaw change (deg) */

extern void auto_mission();
extern void kfs_auto();
extern void Auto_Init();
extern void auto_second_inplace();
extern void auto_inplace(float target_x,float target_y,float target_w);
extern float auto_pid(float diff, uint8_t flag);
extern void auto_pid_reset(void);

extern float auto_set(float src, float src_low, float src_high, float dst_min, float dst_max, float deadzone);

#endif


