# KFS 自动化取放 — 设计文档

## 目标

将取 KFS 的手动遥控流程（开吸盘→摆姿态→拨杆前推→回收）封装为自动化状态机，
支持单步调试和全自动两种模式，全部参数 debugger 在线可调。

## 架构

新建 `kfs_auto.c` / `kfs_auto.h`，零侵入集成到现有代码。
用 `Kfs_Auto_Module` 结构体封装全部状态和参数，复用 `arm.c` 逆解算、
`motor_control.c` 平滑到位、`kfs.c` 吸盘/电机句柄。

## 状态机

```
IDLE ─→ PUMP_ON ─→ POSE ─→ PUSH ─→ RETRACT ─→ DONE
        开吸盘     摆姿态    s-ramp    多段回收   关吸盘
```

### 高度选择

| height_sel | arm mode | h     | 说明 |
|-----------|----------|-------|------|
| 0         | 0        | 0.23  | 低位 |
| 5         | 5        | 0.435 | 中位 |
| 6         | 6        | 0.65  | 高位 |

三种高度共用同一逆解算分支，仅 `h` 不同。

### 前推 s-ramp

替代遥控器 `RCctrl.accel` 拨杆，程序化递增 `arm.s`：

- 起始: `push.s_start` = 0.32
- 目标: `push.s_target` = 0.45
- 步长: `push.s_step` = 0.005（每帧 5mm）

### 回收

复用 `arm.c` mode 3 的三段式回收逻辑：
- 阶段1: arm_2 ▸ th1 (-1.6)
- 阶段2: arm_1 收 + arm_2 ▸ th2 (-2.92)
- 阶段3: arm_2 ▸ target2 (-4.3), arm_3 转 arm3_target (-52)

### 吸盘控制

PUMP_ON 到 DONE 之间 GPIO PE14/PC12 保持 SET，DONE 时 RESET。

## 运行模式

| 模式 | 触发 | 行为 |
|------|------|------|
| 单步调试 | Watch 窗口写 `kfs_auto.debug_step` (1→2→3→4→5) | 上升沿推进一格 |
| 全自动 | 写 `kfs_auto.auto_run = 1` | 到位自动进入下一步，跑完清零 |

## 结构体设计

```c
Kfs_Auto_Module
├── state: Kfs_Auto_State          // 当前状态
├── push:  Kfs_Auto_PushParam      // s_start, s_target, s_step
├── pose:  Kfs_Auto_PoseParam      // h_low/mid/high, pose_s, height_sel, pose_tol
├── retract: Kfs_Auto_RetractParam // th1/2, target2, arm1/3 targets, KP/KD
├── debug_step: uint8_t            // 手动步进触发
├── auto_run: uint8_t              // 全自动触发
├── pump_delay_ms: uint16_t        // 吸盘稳定延时
└── cur_s: float                   // 内部当前 s（只读）
```

## 集成方式

`Can_Task.c` 的 `HF_Task()` 中加一行 `KFS_Auto_Process()`，放在 `Arm_task()` 之前。

## 依赖

- `arm.h/c`: `Arm_Inverse_Solution()`, `Torque_Comp_global()`, `arm`, `angles`, `Torque`
- `kfs.h/c`: `kfs_arm_1/2/3`, GPIO 吸盘 (PE14, PC12)
- `motor_control.h/c`: `in_place()`, `in_place_1()`
- `remote_control.h`: `data_convert()` (如需)
- `cmsis_os.h`: `osDelay()` (如需)
