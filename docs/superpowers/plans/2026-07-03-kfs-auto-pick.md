# KFS 自动化取放 — 实现计划

> **For agentic workers:** REQUIRED SUB-SKILL: Use superpowers:subagent-driven-development (recommended) or superpowers:executing-plans to implement this plan task-by-task. Steps use checkbox (`- [ ]`) syntax for tracking.

**Goal:** 将取 KFS 的手动遥控流程（开吸盘→摆姿态→拨杆前推→回收）封装为自动化状态机，支持单步调试和全自动两种模式。

**Architecture:** 新建 `kfs_auto.c/h`，用 `Kfs_Auto_Module` 结构体封装全部参数和状态。复用 `arm.c` 逆解算、`motor_control.c` 平滑到位、`kfs.c` 电机句柄和 GPIO。`Can_Task.c` 只加一行调用，零侵入。

**Tech Stack:** C (STM32H723), CMSIS-OS, 已有 arm/kfs/motor_control 模块

## Global Constraints

- 不改动 `arm.c`、`kfs.c`、`motor_control.c` 任何已有代码
- `Can_Task.c` 只加 1 行调用，其余不改
- 所有参数 debugger Watch 窗口直接可写，在线生效
- 吸盘 GPIO: PE14、PC12，SET=开，RESET=关
- 高度选择用 arm mode 值：0=低、5=中、6=高

---

### Task 1: 创建 `kfs_auto.h` 头文件

**Files:**
- Create: `RC26_H7_R1(new)/user/inc/kfs_auto.h`

**Interfaces:**
- Produces: `Kfs_Auto_State` 枚举，`Kfs_Auto_PushParam`、`Kfs_Auto_PoseParam`、`Kfs_Auto_RetractParam`、`Kfs_Auto_Module` 结构体，`extern Kfs_Auto_Module kfs_auto`，`KFS_Auto_Init()`、`KFS_Auto_Process()` 声明

- [ ] **Step 1: 创建头文件**

```c
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
```

- [ ] **Step 2: 验证** — 确认 `user/inc/kfs_auto.h` 文件存在且内容正确

---

### Task 2: 创建 `kfs_auto.c` 实现文件

**Files:**
- Create: `RC26_H7_R1(new)/user/src/kfs_auto.c`

**Interfaces:**
- Consumes: `kfs_auto.h`, `arm.h` (`Arm_Inverse_Solution`, `Torque_Comp_global`, `arm`, `angles`, `Torque`, `PI`, `DM_MotorModule`), `kfs.h` (`kfs_arm_1/2/3`), `motor_control.h` (`in_place`, `in_place_1`), `cmsis_os.h` (`osDelay`)
- Produces: `kfs_auto` 全局实例（含默认值）, `KFS_Auto_Init()`, `KFS_Auto_Process()`

- [ ] **Step 1: 创建 .c 文件，包含全局实例初始化和全部实现**

```c
#include "kfs_auto.h"
#include "arm.h"
#include "kfs.h"
#include "motor_control.h"
#include "cmsis_os.h"
#include <math.h>
#include <string.h>

/* ========== 默认值 ========== */
#define DFL_S_START      0.32f
#define DFL_S_TARGET     0.45f
#define DFL_S_STEP       0.005f
#define DFL_H_LOW        0.23f
#define DFL_H_MID        0.435f
#define DFL_H_HIGH       0.65f
#define DFL_POSE_S       0.32f
#define DFL_HEIGHT_SEL   0
#define DFL_POSE_TOL     0.05f
#define DFL_PUMP_DELAY   300

#define DFL_RET_TH1      -1.6f
#define DFL_RET_TH2      -2.92f
#define DFL_RET_TARGET2  -4.3f
#define DFL_RET_ARM1_MID 0.2f
#define DFL_RET_ARM1_DEEP 0.6f
#define DFL_RET_ARM3_TGT -52.0f
#define DFL_RET_KP2_S1   30.0f
#define DFL_RET_KD2_S1   5.0f
#define DFL_RET_KP1_S2   55.0f
#define DFL_RET_KD1_S2   5.0f
#define DFL_RET_KP2_S2   30.0f
#define DFL_RET_KD2_S2   5.0f
#define DFL_RET_DONE_TOL 0.15f

/* ========== 全局实例 ========== */
Kfs_Auto_Module kfs_auto = {
    .state         = KFS_AUTO_IDLE,
    .push          = { DFL_S_START,  DFL_S_TARGET,  DFL_S_STEP },
    .pose          = { DFL_H_LOW,    DFL_H_MID,     DFL_H_HIGH,
                       DFL_POSE_S,   DFL_HEIGHT_SEL, DFL_POSE_TOL },
    .retract       = { DFL_RET_TH1,       DFL_RET_TH2,        DFL_RET_TARGET2,
                       DFL_RET_ARM1_MID,  DFL_RET_ARM1_DEEP,  DFL_RET_ARM3_TGT,
                       DFL_RET_KP2_S1,    DFL_RET_KD2_S1,
                       DFL_RET_KP1_S2,    DFL_RET_KD1_S2,
                       DFL_RET_KP2_S2,    DFL_RET_KD2_S2,
                       DFL_RET_DONE_TOL },
    .debug_step    = 0,
    .auto_run      = 0,
    .pump_delay_ms = DFL_PUMP_DELAY,
    .cur_s         = DFL_S_START,
};

/* ========== 内部状态 ========== */
static uint8_t  last_step = 0;
static float    buf1[5]   = {0};
static float    buf2[5]   = {0};
static uint32_t pump_tick = 0;

/* ========== 高度→h ========== */
static float get_h(void) {
    if      (kfs_auto.pose.height_sel == 5) return kfs_auto.pose.h_mid;
    else if (kfs_auto.pose.height_sel == 6) return kfs_auto.pose.h_high;
    else                                     return kfs_auto.pose.h_low;
}

/* ========== 初始化 ========== */
void KFS_Auto_Init(void) {
    kfs_auto.state      = KFS_AUTO_IDLE;
    kfs_auto.debug_step = 0;
    kfs_auto.auto_run   = 0;
    kfs_auto.cur_s      = kfs_auto.push.s_start;
    last_step           = 0;
    pump_tick           = 0;
    memset(buf1, 0, sizeof(buf1));
    memset(buf2, 0, sizeof(buf2));
}

/* ========== debug_step 上升沿检测 → 手动推进一格 ========== */
static void check_debug_trigger(void) {
    if (kfs_auto.debug_step != last_step && kfs_auto.debug_step > 0) {
        if (kfs_auto.state < KFS_AUTO_DONE) {
            kfs_auto.state++;
        }
        last_step = kfs_auto.debug_step;
    }
}

/* ========== auto_run 模式下到位自动推进 ========== */
static void try_auto_advance(void) {
    if (!kfs_auto.auto_run) return;

    switch (kfs_auto.state) {
        case KFS_AUTO_PUMP_ON:
            if (HAL_GetTick() - pump_tick >= kfs_auto.pump_delay_ms) {
                kfs_auto.state = KFS_AUTO_POSE;
            }
            break;

        case KFS_AUTO_POSE: {
            float a1 = angles.theta_1 + 0.43f;
            float a2 = -angles.theta_2;
            if (fabsf(kfs_arm_1.position - a1) < kfs_auto.pose.pose_tol &&
                fabsf(kfs_arm_2.position - a2) < kfs_auto.pose.pose_tol) {
                kfs_auto.state = KFS_AUTO_PUSH;
            }
            break;
        }

        case KFS_AUTO_PUSH:
            if (kfs_auto.cur_s >= kfs_auto.push.s_target) {
                kfs_auto.state = KFS_AUTO_RETRACT;
            }
            break;

        case KFS_AUTO_RETRACT:
            if (kfs_arm_2.position <
                kfs_auto.retract.target2 + kfs_auto.retract.done_tol) {
                kfs_auto.state = KFS_AUTO_DONE;
            }
            break;

        default:
            break;
    }
}

/* ========== 主处理函数，每帧在 HF_Task 中调用 ========== */
void KFS_Auto_Process(void) {
    check_debug_trigger();

    /* IDLE: 什么都不做 */
    if (kfs_auto.state == KFS_AUTO_IDLE) return;

    /* DONE: 关吸盘，清零 auto_run */
    if (kfs_auto.state == KFS_AUTO_DONE) {
        HAL_GPIO_WritePin(GPIOE, GPIO_PIN_14, GPIO_PIN_RESET);
        HAL_GPIO_WritePin(GPIOC, GPIO_PIN_12, GPIO_PIN_RESET);
        kfs_auto.auto_run = 0;
        return;
    }

    /* PUMP_ON 记录时刻 */
    if (kfs_auto.state == KFS_AUTO_PUMP_ON && pump_tick == 0) {
        pump_tick = HAL_GetTick();
    }

    /* PUMP_ON ~ RETRACT: 吸盘一直开 */
    HAL_GPIO_WritePin(GPIOE, GPIO_PIN_14, GPIO_PIN_SET);
    HAL_GPIO_WritePin(GPIOC, GPIO_PIN_12, GPIO_PIN_SET);

    /* 重力补偿 */
    Torque = Torque_Comp_global(&kfs_arm_1, &kfs_arm_2, &kfs_arm_3);

    switch (kfs_auto.state) {

        /* ----- PUMP_ON: 仅开吸盘，维持当前臂位 ----- */
        case KFS_AUTO_PUMP_ON:
            break;

        /* ----- POSE: 摆出取 KFS 姿态 ----- */
        case KFS_AUTO_POSE:
            arm.mode = kfs_auto.pose.height_sel;
            arm.s    = kfs_auto.pose.pose_s;
            arm.h    = get_h();
            angles   = Arm_Inverse_Solution(&arm);

            kfs_arm_1.set_mit_data(&kfs_arm_1, angles.theta_1 + 0.43f, 0.0f,
                                   70.0f, 5.0f, -0.25f * Torque.Torque_1);
            kfs_arm_2.set_mit_data(&kfs_arm_2, -angles.theta_2, 0.0f,
                                   50.0f, 4.5f, -0.6f * Torque.Torque_2);
            kfs_arm_3.set_mit_data(&kfs_arm_3,
                                   25.0f * (PI / 2.0f + kfs_arm_1.position - 0.43f
                                            + kfs_arm_2.position - PI * 0.08f),
                                   0.0f, 0.2f, 0.15f, 0.8f * Torque.Torque_3);
            break;

        /* ----- PUSH: s 渐进 ramp 替代遥控拨杆 ----- */
        case KFS_AUTO_PUSH:
            if (kfs_auto.cur_s < kfs_auto.push.s_target) {
                kfs_auto.cur_s += kfs_auto.push.s_step;
            }
            arm.mode = kfs_auto.pose.height_sel;
            arm.s    = kfs_auto.cur_s;
            arm.h    = get_h();
            angles   = Arm_Inverse_Solution(&arm);

            kfs_arm_1.set_mit_data(&kfs_arm_1, angles.theta_1 + 0.43f, 0.0f,
                                   70.0f, 5.0f, -0.25f * Torque.Torque_1);
            kfs_arm_2.set_mit_data(&kfs_arm_2, -angles.theta_2, 0.0f,
                                   50.0f, 4.5f, -0.6f * Torque.Torque_2);
            kfs_arm_3.set_mit_data(&kfs_arm_3,
                                   25.0f * (PI / 2.0f + kfs_arm_1.position - 0.43f
                                            + kfs_arm_2.position - PI * 0.08f),
                                   0.0f, 0.2f, 0.15f, 0.8f * Torque.Torque_3);
            break;

        /* ----- RETRACT: 复用 mode 3 三段回收 ----- */
        case KFS_AUTO_RETRACT: {
            Kfs_Auto_RetractParam *r = &kfs_auto.retract;

            /* 阶段1: arm_2 收到 th1 */
            if (kfs_arm_2.position > r->th1) {
                in_place(r->th1,
                         &buf2[0], kfs_arm_2.position, &buf2[1], 0.6f,
                         &buf2[2], r->kp2_s1,
                         &buf2[3], r->kd2_s1, 4.0f,
                         &buf2[4], -0.5f * Torque.Torque_2, -2.0f * Torque.Torque_2, 0.0f);
                kfs_arm_2.set_mit_data(&kfs_arm_2, buf2[0], buf2[1],
                                       buf2[2], buf2[3], buf2[4]);
            }

            /* 阶段2: arm_1 收 + arm_2 继续收到 th2 */
            if (kfs_arm_2.position > r->th2 &&
                kfs_arm_2.position < r->th1) {
                in_place(r->arm1_mid,
                         &buf1[0], kfs_arm_1.position, &buf1[1], 2.0f,
                         &buf1[2], r->kp1_s2,
                         &buf1[3], r->kd1_s2, 5.0f,
                         &buf1[4], -0.5f * Torque.Torque_1, -0.5f * Torque.Torque_1, 0.3f);
                kfs_arm_1.set_mit_data(&kfs_arm_1, buf1[0], buf1[1],
                                       buf1[2], buf1[3], buf1[4]);

                in_place(r->th2,
                         &buf2[0], kfs_arm_2.position, &buf2[1], 0.1f,
                         &buf2[2], r->kp2_s2,
                         &buf2[3], r->kd2_s2, 4.0f,
                         &buf2[4], -0.2f * Torque.Torque_2, -1.8f * Torque.Torque_2, 0.0f);
                kfs_arm_2.set_mit_data(&kfs_arm_2, buf2[0], buf2[1],
                                       buf2[2], buf2[3], buf2[4]);
            }

            /* 阶段3: 最深段 — arm_3 转, arm_2 收到 target2 */
            if (kfs_arm_2.position < r->th2) {
                kfs_arm_3.set_mit_data(&kfs_arm_3, r->arm3_target, 0.0f,
                                       0.13f, 0.6f, 1.0f * Torque.Torque_3);
                kfs_arm_1.set_mit_data(&kfs_arm_1, r->arm1_deep, 0.0f,
                                       60.0f, 5.0f, -0.8f * Torque.Torque_1);
                in_place_1(r->target2,
                           &buf2[0], kfs_arm_2.position, &buf2[1], -0.0f,
                           &buf2[2], 65.0f,
                           &buf2[3], 3.0f, 5.0f,
                           &buf2[4], -0.12f * Torque.Torque_2, -1.0f * Torque.Torque_2, 0.28f);
                kfs_arm_2.set_mit_data(&kfs_arm_2, buf2[0], buf2[1],
                                       buf2[2], buf2[3], buf2[4]);
            }
            break;
        }

        default:
            break;
    }

    /* auto_run 自动推进检查 */
    try_auto_advance();
}
```

- [ ] **Step 2: 验证** — 确认文件存在，检查编译无语法错误

---

### Task 3: 修改 `Can_Task.c` 加入调用

**Files:**
- Modify: `RC26_H7_R1(new)/user/src/Can_Task.c`

**Interfaces:**
- Consumes: `kfs_auto.h` (`KFS_Auto_Process()`)

- [ ] **Step 1: 添加 include**

在 `Can_Task.c` 顶部现有 include 块末尾添加：

```c
#include "kfs_auto.h"
```

位置：在 `#include "auto_mode.h"` 之后。

- [ ] **Step 2: 在 HF_Task 中加入 KFS_Auto_Process 调用**

在 `HF_Task()` 函数中，`Auto_Init();` 之后、`if(RCctrl.rc_lost)` 之前加一行：

```c
void HF_Task()          //高频任务，200hz
{
    Auto_Init();
    KFS_Auto_Process();  // <-- 新增这一行
    
  if(RCctrl.rc_lost)
{
```

- [ ] **Step 3: 验证** — 确认 `Can_Task.c` 修改正确，编译通过

---

### Task 4: 构建验证

**Files:**
- 验证: `RC26_H7_R1(new)/MDK-ARM/RC26_H7_R1.uvprojx` (Keil 工程)

- [ ] **Step 1: 确认文件已加入 Keil 工程**

在 Keil IDE 中手动将 `user/src/kfs_auto.c` 和 `user/inc/kfs_auto.h` 加入工程（或确认 MDK 自动识别）。

- [ ] **Step 2: 编译工程**

```powershell
# 通过 Keil 或命令行编译
# 预期: 0 errors, 0 warnings
```

- [ ] **Step 3: 烧录并验证调试功能**

1. 烧录到开发板
2. 进入 debug 模式
3. Watch 窗口添加 `kfs_auto`
4. 展开结构体，验证所有默认值正确
5. 写 `kfs_auto.debug_step = 1`，确认吸盘 GPIO 输出 SET
6. 写 `kfs_auto.debug_step = 2`，确认机械臂摆出姿态
7. 继续验证 PUSH / RETRACT / DONE
8. 验证全自动：`KFS_Auto_Init()` 后写 `kfs_auto.auto_run = 1`

- [ ] **Step 4: Commit**

```bash
git add user/inc/kfs_auto.h user/src/kfs_auto.c user/src/Can_Task.c
git commit -m "feat: add KFS auto-pick state machine with debug step and auto-run modes"
```
