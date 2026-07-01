/**
 * @file common.h
 * @brief 通用工具小函数，包含角度归一化、浮点约束、向量限幅等功能
 */
#ifndef COMMON_H
#define COMMON_H

#include <stdint.h>

#ifndef M_PI_F
#define M_PI_F 3.14159265358979323846f
#endif

#ifdef __cplusplus
extern "C" {
#endif

/** @brief 将角度（度）归一化到 [-180, 180] */
float wrap_deg_180(float deg);

/** @brief 获取当前时间（基于HAL_GetTick） */
uint32_t common_now_ms(void);

/** @brief 将 v 约束在 [lo, hi] 范围内，要求 lo <= hi */
float clampf(float v, float lo, float hi);

/**
 * @brief 将 (vx,vy) 的模限制在 vmax 以内，超过则按比例缩小；不改变方向。
 * @param vmax 最大速度，应 > 0
 */
void vec2_limit(float *vx, float *vy, float vmax);

#ifdef __cplusplus
}
#endif

#endif /* COMMON_H */