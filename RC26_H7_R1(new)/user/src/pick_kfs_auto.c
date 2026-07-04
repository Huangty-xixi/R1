#include "pick_kfs_auto.h"
#include "kfs.h"
#include "motor_control.h"
#include "dm_motor.h"
#include "pid.h"
#include "common.h"
#include "cmsis_os.h"

static const float HEIGHT_CFG[3] = { 0.230f, 0.435f, 0.650f };

static const float POSE_TOL[3]   = { 0.03f, 0.03f, 2.0f };
static const float PUSH_KP_ARM1  = 70.0f;
static const float PUSH_KD_ARM1  = 5.0f;
static const float PUSH_KP_ARM2  = 50.0f;
static const float PUSH_KD_ARM2  = 4.5f;
static const uint16_t TIMEOUT[4] = { 500, 4000, 3000, 8000 };

static volatile PickKfsDebug g_dbg;

static float g_push_s;
static float g_h_target;
static uint32_t g_t0;
static float g_kfs_1[5];
static float g_kfs_2[5];

static uint8_t consume_step(void) {
    if (g_dbg.step_next) { g_dbg.step_next = 0; return 1; }
    return 0;
}

static void sucker_set(uint8_t on) {
    GPIO_PinState s = on ? GPIO_PIN_SET : GPIO_PIN_RESET;
    HAL_GPIO_WritePin(GPIOE, GPIO_PIN_14, s);
    HAL_GPIO_WritePin(GPIOC, GPIO_PIN_12, s);
}

static void arm_send_pose(float s, float h) {
    arm.mode = 0;
    arm.s    = s;
    arm.h    = h;
    Arm_Inverse_Solution(&arm);
    Torque_Comp_global(&kfs_arm_1, &kfs_arm_2, &kfs_arm_3);

    kfs_arm_1.set_mit_data(&kfs_arm_1,
        angles.theta_1 + 0.43f, 0.0f, PUSH_KP_ARM1, PUSH_KD_ARM1,
        -0.25f * Torque.Torque_1);

    kfs_arm_2.set_mit_data(&kfs_arm_2,
        -angles.theta_2, 0.0f, PUSH_KP_ARM2, PUSH_KD_ARM2,
        -0.6f * Torque.Torque_2);

    kfs_arm_3.set_mit_data(&kfs_arm_3,
        25.0f * (PI/2.0f + kfs_arm_1.position - 0.43f
                 + kfs_arm_2.position - PI * 0.08f),
        0.0f, 0.2f, 0.15f, 0.8f * Torque.Torque_3);
}

static void arm_retract_tick(void) {
    Torque_Comp_global(&kfs_arm_1, &kfs_arm_2, &kfs_arm_3);

    if (kfs_arm_2.position > -1.6f) {
        g_dbg.retract_phase = 0;
        in_place(-1.6f, &g_kfs_2[0], kfs_arm_2.position, &g_kfs_2[1],
                 0.6f, &g_kfs_2[2], 30.0f, &g_kfs_2[3], 5.0f, 4.0f,
                 &g_kfs_2[4], -0.5f*Torque.Torque_2, -2.0f*Torque.Torque_2, 0.0f);
        kfs_arm_2.set_mit_data(&kfs_arm_2,
            g_kfs_2[0], g_kfs_2[1], g_kfs_2[2], g_kfs_2[3], g_kfs_2[4]);
    }
    else if (kfs_arm_2.position > -2.92f) {
        g_dbg.retract_phase = 1;
        in_place(0.2f, &g_kfs_1[0], kfs_arm_1.position, &g_kfs_1[1],
                 2.0f, &g_kfs_1[2], 55.0f, &g_kfs_1[3], 5.0f, 5.0f,
                 &g_kfs_1[4], -0.5f*Torque.Torque_1, -0.5f*Torque.Torque_1, 0.3f);
        kfs_arm_1.set_mit_data(&kfs_arm_1,
            g_kfs_1[0], g_kfs_1[1], g_kfs_1[2], g_kfs_1[3], g_kfs_1[4]);
        in_place(-2.92f, &g_kfs_2[0], kfs_arm_2.position, &g_kfs_2[1],
                 0.1f, &g_kfs_2[2], 30.0f, &g_kfs_2[3], 5.0f, 4.0f,
                 &g_kfs_2[4], -0.2f*Torque.Torque_2, -1.8f*Torque.Torque_2, 0.0f);
        kfs_arm_2.set_mit_data(&kfs_arm_2,
            g_kfs_2[0], g_kfs_2[1], g_kfs_2[2], g_kfs_2[3], g_kfs_2[4]);
    }
    else {
        g_dbg.retract_phase = 2;
        kfs_arm_3.set_mit_data(&kfs_arm_3,
            -52.0f, 0.0f, 0.13f, 0.6f, 1.0f*Torque.Torque_3);
        kfs_arm_1.set_mit_data(&kfs_arm_1,
            0.6f, 0.0f, 60.0f, 5.0f, -0.8f*Torque.Torque_1);
        in_place_1(-4.3f, &g_kfs_2[0], kfs_arm_2.position, &g_kfs_2[1],
                   -0.0f, &g_kfs_2[2], 65.0f, &g_kfs_2[3], 3.0f, 5.0f,
                   &g_kfs_2[4], -0.12f*Torque.Torque_2, -1.0f*Torque.Torque_2, 0.28f);
        kfs_arm_2.set_mit_data(&kfs_arm_2,
            g_kfs_2[0], g_kfs_2[1], g_kfs_2[2], g_kfs_2[3], g_kfs_2[4]);
    }
}

void PickKfsAuto_Init(void) {
    g_dbg.trigger       = 0;
    g_dbg.step_mode     = 0;
    g_dbg.step_next     = 0;
    g_dbg.abort         = 0;
    g_dbg.state         = PKFS_IDLE;
    g_dbg.frame_cnt     = 0;
    g_dbg.retract_phase = 0;
    g_dbg.height_sel    = 0;
    g_dbg.sucker_delay_ms  = 200;
    g_dbg.push_s[0]     = 0.32f;
    g_dbg.push_s[1]     = 0.60f;
    g_dbg.push_s[2]     = 0.005f;
    g_dbg.retract_done_pos = -4.2f;
}

PickKfsState PickKfsAuto_State(void) {
    return (PickKfsState)g_dbg.state;
}

uint8_t PickKfsAuto_IsIdle(void) {
    return (g_dbg.state == PKFS_IDLE || g_dbg.state == PKFS_DONE);
}

void PickKfsAuto_Run(void) {
    if (g_dbg.abort) {
        sucker_set(0);
        g_dbg.state = PKFS_IDLE;
        g_dbg.abort = 0;
        g_dbg.trigger = 0;
        return;
    }

    if (!g_dbg.step_mode && !g_dbg.trigger) return;

    uint32_t now = common_now_ms();

    switch (g_dbg.state) {

    case PKFS_IDLE:
        if (g_dbg.trigger) {
            g_dbg.trigger = 0;
            g_h_target    = HEIGHT_CFG[g_dbg.height_sel];
            g_push_s      = g_dbg.push_s[0];
            g_dbg.frame_cnt = 0;
            g_t0          = now;
            g_dbg.state   = PKFS_SUCKER_ON;
            if (g_dbg.step_mode) return;
        }
        break;

    case PKFS_SUCKER_ON:
        if (g_dbg.step_mode && !consume_step()) return;
        sucker_set(1);
        g_dbg.frame_cnt++;
        if ((g_dbg.frame_cnt * 5) >= g_dbg.sucker_delay_ms ||
            now - g_t0 > TIMEOUT[0]) {
            g_dbg.state = PKFS_ARM_POSE;
            g_dbg.frame_cnt = 0;  g_t0 = now;
            if (g_dbg.step_mode) return;
        }
        break;

    case PKFS_ARM_POSE:
        if (g_dbg.step_mode && !consume_step()) return;
        arm_send_pose(g_dbg.push_s[0], g_h_target);
        g_dbg.frame_cnt++;
        if ((ABS(kfs_arm_1.position - (angles.theta_1 + 0.43f)) < POSE_TOL[0]) &&
            (ABS(kfs_arm_2.position + angles.theta_2) < POSE_TOL[1]) &&
            (ABS(kfs_arm_3.position - 25.0f*(PI/2.0f + kfs_arm_1.position
                - 0.43f + kfs_arm_2.position - PI * 0.08f)) < POSE_TOL[2])) {
            g_dbg.state = PKFS_ARM_PUSH;
            g_dbg.frame_cnt = 0;  g_t0 = now;
            if (g_dbg.step_mode) return;
        }
        if (now - g_t0 > TIMEOUT[1]) {
            g_dbg.state = PKFS_TIMEOUT;
        }
        break;

    case PKFS_ARM_PUSH:
        if (g_dbg.step_mode && !consume_step()) return;
        g_push_s = clampf(g_push_s + g_dbg.push_s[2],
                          g_dbg.push_s[0], g_dbg.push_s[1]);
        arm_send_pose(g_push_s, g_h_target);
        g_dbg.frame_cnt++;
        if (g_push_s >= g_dbg.push_s[1] ||
            now - g_t0 > TIMEOUT[2]) {
            g_dbg.state = PKFS_ARM_RETRACT;
            g_dbg.frame_cnt = 0;  g_t0 = now;
            if (g_dbg.step_mode) return;
        }
        break;

    case PKFS_ARM_RETRACT:
        if (g_dbg.step_mode && !consume_step()) return;
        arm_retract_tick();
        g_dbg.frame_cnt++;
        if (kfs_arm_2.position < g_dbg.retract_done_pos ||
            now - g_t0 > TIMEOUT[3]) {
            g_dbg.state = PKFS_DONE;
            sucker_set(0);
            if (g_dbg.step_mode) return;
        }
        break;

    case PKFS_DONE:
        g_dbg.state = PKFS_IDLE;
        break;

    case PKFS_TIMEOUT:
        g_dbg.state = PKFS_IDLE;
        break;

    default: break;
    }
}