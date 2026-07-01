/**
 * @file odom_center_offset.h
 * @brief 雷达中心坐标偏移，根据底盘朝向0/90/180/-90建立坐标变换关系，将雷达坐标转换到底盘中心
 *
 * 坐标系定义：前方 +0.09 m，左侧 -0.12 m，雷达坐标系原点在底盘中心后方偏左。ODOM yaw 角度定义：车头朝向为0度，逆时针为正，顺时针为负。
 */
#ifndef ODOM_CENTER_OFFSET_H
#define ODOM_CENTER_OFFSET_H

#include <stdint.h>

#include "upper_pc_protocol.h"

#ifdef __cplusplus
extern "C" {
#endif

/** 雷达 -> 中心，坐标系定义：前方为正X，左侧为正Y，单位为米 */
#ifndef ODOM_CENTER_OFFSET_RADAR_FWD_M
#define ODOM_CENTER_OFFSET_RADAR_FWD_M  (0.11f)
#endif
#ifndef ODOM_CENTER_OFFSET_RADAR_LEFT_M
#define ODOM_CENTER_OFFSET_RADAR_LEFT_M (-0.09f)
#endif

/** 底盘朝向与 YawHeadingCtrl FRONT/BACK/LEFT/RIGHT 一致，对应角度为 0/±90/180 */
typedef enum {
    ODOM_CENTER_OFFSET_DIR_FRONT = 0,
    ODOM_CENTER_OFFSET_DIR_BACK  = 1,
    ODOM_CENTER_OFFSET_DIR_LEFT  = 2,
    ODOM_CENTER_OFFSET_DIR_RIGHT = 3,
} odom_center_offset_dir_t;

/**
 * @brief 根据ODOM yaw角度，确定底盘朝向之一：[-45,45]为前方，(45,135]为左方，(135,180]|(-180,-135]为后方，(-135,-45]为右方
 */
odom_center_offset_dir_t odom_center_offset_dir_from_yaw_deg(float yaw_deg);

/**
 * @brief 获取偏移量，雷达->中心的坐标变换参数 (dx,dy)，is_red_side 1=红方 0=蓝方
 */
void odom_center_offset_table_lookup_ex(uint8_t is_red_side,
                                        odom_center_offset_dir_t dir,
                                        float *dx_m,
                                        float *dy_m);

void odom_center_offset_radar_to_center_ex(uint8_t is_red_side,
                                           float radar_x_m,
                                           float radar_y_m,
                                           float yaw_deg,
                                           float *center_x_m,
                                           float *center_y_m);

void odom_center_offset_radar_to_center_by_dir_ex(uint8_t is_red_side,
                                                  float radar_x_m,
                                                  float radar_y_m,
                                                  odom_center_offset_dir_t dir,
                                                  float *center_x_m,
                                                  float *center_y_m);

void odom_center_offset_radar_to_center(float radar_x_m,
                                        float radar_y_m,
                                        float yaw_deg,
                                        float *center_x_m,
                                        float *center_y_m);

void odom_center_offset_center_to_radar_ex(uint8_t is_red_side,
                                           float center_x_m,
                                           float center_y_m,
                                           float yaw_deg,
                                           float *radar_x_m,
                                           float *radar_y_m);

void odom_center_offset_center_to_radar_by_dir_ex(uint8_t is_red_side,
                                                  float center_x_m,
                                                  float center_y_m,
                                                  odom_center_offset_dir_t dir,
                                                  float *radar_x_m,
                                                  float *radar_y_m);

void odom_center_offset_center_to_radar(float center_x_m,
                                        float center_y_m,
                                        float yaw_deg,
                                        float *radar_x_m,
                                        float *radar_y_m);

void odom_center_offset_odom_to_center(const rc_odom_t *radar_odom,
                                       float *center_x_m,
                                       float *center_y_m);

uint8_t odom_center_offset_latest_center(float *center_x_m, float *center_y_m);

#ifdef __cplusplus
}
#endif

#endif /* ODOM_CENTER_OFFSET_H */