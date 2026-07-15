#ifndef __REMOTE_CONTROL_H__
#define __REMOTE_CONTROL_H__

#include "stdint.h"
#include "stdbool.h"
#include "stdlib.h"
#include "string.h"
#include "math.h"
#include <stdio.h>
#include <string.h>
#include "policy.h"

#define SBUS_RX_BUF_NUM 50u
#define RC_FRAME_LENGTH 25u
#define RC_CH_VALUE_OFFSET 1024U
#define REMOTE_BUF_SIZE   32

//accel
#define ACCEL_LOW  0
#define ACCEL_HIGH  200
//vw
#define VW_LOW  -100
#define VW_MID  0
#define VW_HIGH  100
//vx
#define VX_LOW  -100
#define VX_MID  0
#define VX_HIGH  100
//vy
#define VY_LOW  -100
#define VY_MID  0
#define VY_HIGH  100

#define DEADZONE  10

#define CH1_LOW     1693        //LEFT
#define CH1_HIGH    306         //RIGHT
#define CH1_MID     1000        //MID
#define CH2_LOW     309         //DOWN
#define CH2_HIGH    1686        //UP
#define CH2_MID     1000        //MID
#define CH3_LOW     299         //DOWN
#define CH3_HIGH    1659        //UP
#define CH4_LOW     314         //LEFT
#define CH4_HIGH    1700        //RIGHT
#define CH4_MID     999         //MID

#define SPEED_ACCEL   remote_control_readaccel(RCctrl.accel, ACCEL_LOW, ACCEL_HIGH, 0, 40)
#define SPEED_X       remote_control_read(RCctrl.vx, VX_LOW, VX_MID, VX_HIGH, -1, 0, 1)
#define SPEED_Y       remote_control_read(RCctrl.vy, VY_LOW, VY_MID, VY_HIGH, -1, 0, 1)
#define SPEED_WO       remote_control_read(RCctrl.vw, VW_LOW, VW_MID, VW_HIGH, -1.8, 0, 1.8)

//#define SBUS_SPEED_ACCEL   remote_control_readaccel(SBUS_RC.CH3, CH3_LOW, CH3_HIGH, 0, 40)
//#define SBUS_SPEED_X       remote_control_read(SBUS_RC.CH2, CH2_LOW, CH2_MID, CH2_HIGH, -1, 0, 1)
//#define SBUS_SPEED_Y       remote_control_read(SBUS_RC.CH1, CH1_LOW, CH1_MID, CH1_HIGH, -1, 0, 1)
//#define SBUS_SPEED_WO       remote_control_read(SBUS_RC.CH4, CH4_LOW, CH4_MID, CH4_HIGH, -1.8, 0, 1.8)


typedef struct
{
    uint16_t CH1;
    uint16_t CH2;
    uint16_t CH3;
    uint16_t CH4;
    uint16_t CH5;
    uint16_t CH6;
    uint16_t CH7;
    uint16_t CH8;
    uint16_t CH9;
    uint16_t CH10;
	uint16_t CH11;
	uint16_t CH12;
    uint16_t CH13;
    uint16_t CH14;
    uint16_t CH15;
    uint16_t CH16;

	bool rc_lost;   /*!< lost flag */
	uint8_t online_cnt;   /*!< online count */
} Remote_Info_Typedef;

typedef  struct
{
   uint8_t frame_head1;
   uint8_t frame_head2;
   uint8_t frame_tail;

   uint8_t key;
   uint8_t takePos;
   uint8_t liftPos;
   uint8_t rodPos;
   uint8_t topPos;
   uint8_t bottomPos;

   uint8_t chassis;
   uint8_t channel;
   uint8_t zone;

   uint8_t pump;
   uint8_t kfs_catch;
   uint8_t hold;
   uint8_t led_1;
   uint8_t led_2;
   uint8_t led_3;
   uint8_t switch_dir;
   uint8_t lock;

   uint8_t accel;
   int8_t  vw;
   int8_t  vy;
   int8_t  vx;

   uint8_t area;
   uint8_t cmd;

   uint8_t cells[12];
   uint8_t modi_x;

   bool rc_lost;
   bool received;
   bool fir_rec;
   uint8_t th_rec;
   bool th_put;

   uint16_t online_cnt;
}Remote_Info_Typedef_New;


extern uint8_t SBUS_MultiRx_Buf[2][SBUS_RX_BUF_NUM];
extern uint8_t REMOTE_BUF[REMOTE_BUF_SIZE];
extern Remote_Info_Typedef_New RCctrl;
extern Remote_Info_Typedef SBUS_RC;
void SBUS_TO_RC(volatile const uint8_t *sbus_buf, Remote_Info_Typedef *Remote_Ctrl);
extern void REMOTE_ParseData(volatile const uint8_t *remote_buf, Remote_Info_Typedef_New *rc);
extern void parse_config(Remote_Info_Typedef_New *rc, volatile const uint8_t *b);
extern void Check_Cells_Status(Remote_Info_Typedef_New  *rc, int *kfs1, int *kfs2, int *g_kf);

float data_convert(int src, int src_min, int src_max, float dst_low, float dst_high);
float remote_control_read(int src, int src_low, int src_mid, int src_high, float dst_min, float dst_mid, float dst_max);
float remote_control_readaccel(int src, int src_low, int src_high, float dst_min, float dst_max);

#endif