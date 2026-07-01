/**
 * @file    upper_pc_protocol.h
 * @brief   R2 雷达位置信息通信协议（在 STM32 上实现，此头文件用于上位机协议）
 *
 * 帧格式: [0xA5 0x5A CMD LEN(2B LE) PAYLOAD CHKSUM]
 *
 * 上位机 -> 下位机 (Uplink):
 *   CMD 0x01  ODOM         里程计 (p0,p1,z,roll,pitch,yaw) float32[6]，p0/p1 映射到 handle_odom，yaw 与 x/y 同坐标系
 *   CMD 0x02  PATH         路径点   uint8=n, float32[n*2]
 *   CMD 0x03  KFS          KFS数据  uint8=n, [uint8 id, float32 xyz]*n
 *   CMD 0x05  ZONE_I_PATH  I区路径  uint8 start,end,n, [uint8 block_id]*n
 *
 * 下位机 -> 上位机 (Downlink):
 *   CMD 0x10  ACK          确认      uint8 cmd, uint8 code(0=OK 1=ERR)
 *   CMD 0x12  STATUS       状态      uint8 state
 *   CMD 0x13  ZONE_I_INFO  I区信息   uint8 n, [uint8 block_id, kfs_type]*n
 *   CMD 0x14  DOCK_OK      R1对接成功  无
 *   CMD 0x15  GO_ZONE_I    进入I区    无
 *   CMD 0x20  DEBUG_HEADING_HOLD  调试航向PID信息 (float[6])
 *   CMD 0x21  DEBUG_NAV_GOTO     调试导航信息 (float[6])
 *
 * 说明：SBUS 遥控器相关的用 remote_control.h，上位机通信相关的用 upper_pc_protocol.h。
 */
#ifndef UPPER_PC_PROTOCOL_H
#define UPPER_PC_PROTOCOL_H

#include <stdint.h>

/* ---------- 帧定义 ---------- */
#define RC_SYNC1               0xA5
#define RC_SYNC2               0x5A
#define RC_FRAME_HEADER_SIZE   5   /* SYNC1 SYNC2 CMD LEN_LO LEN_HI */
#define RC_FRAME_MAX_PAYLOAD   64
#define RC_FRAME_MAX_SIZE      (RC_FRAME_HEADER_SIZE + RC_FRAME_MAX_PAYLOAD + 1)  /* +1 CHKSUM */
#define RC_ODOM_PAYLOAD_SIZE   24  /* 6*float */

/* ---------- 命令定义 ---------- */
typedef enum {
    RC_CMD_ODOM        = 0x01,  /* 上位机: 里程计 */
    RC_CMD_PATH        = 0x02,  /* 上位机: 路径 */
    RC_CMD_KFS         = 0x03,  /* 上位机: KFS数据 */
    RC_CMD_CMD_RSP     = 0x04,  /* 上位机: 命令响应 */
    RC_CMD_ZONE_I_PATH = 0x05,  /* 上位机: I区路径 */
    RC_CMD_ACK         = 0x10,  /* 下位机: 确认 */
    RC_CMD_STATUS      = 0x12,  /* 下位机: 状态 */
    RC_CMD_ZONE_I_INFO = 0x13,  /* 下位机: I区KFS信息 */
    RC_CMD_DOCK_OK     = 0x14,  /* 下位机: R1对接成功 */
    RC_CMD_GO_ZONE_I   = 0x15,  /* 下位机: 进入I区 */

    /* PID 调试通信 */
    RC_CMD_DEBUG_HEADING_HOLD = 0x20,  /* 下位机: 调试航向PID状态 (float[6]) */
    RC_CMD_DEBUG_NAV_GOTO    = 0x21,  /* 下位机: 调试导航信息 (float[6]) */
} rc_cmd_t;

/* ---------- 数据结构 ---------- */

/** 里程计数据（收到 CMD_ODOM 时解析） */
typedef struct {
    float x;      /* x */
    float y;      /* y */
    float z;      /* z */
    float roll;   /* 横滚 */
    float pitch;  /* 俯仰 */
    float yaw;    /* 航向 */
} rc_odom_t;

/** 路径点定义 */
typedef struct {
    float x;
    float y;
} rc_waypoint_t;

/** 路径数据 */
typedef struct {
    uint8_t       num;//路径点数量
    rc_waypoint_t points[16];//路径点列表
} rc_path_t;

/** 单个 KFS 检测数据 */
typedef struct {
    uint8_t id;//KFS编号
    float   x, y, z; /* 坐标系中位置，米 */
} rc_kfs_detect_t;

/** KFS 检测列表 */
typedef struct {
    uint8_t          num;//KFS检测数量
    rc_kfs_detect_t  detections[8];//KFS检测列表
} rc_kfs_t;

/** I区指定路径 */
typedef struct {
    uint8_t start_block;//起始块ID
    uint8_t end_block;//终止块ID
    uint8_t num_blocks;//块数量
    uint8_t block_ids[32];//块ID列表
} rc_zone_i_path_t;

/** I区 KFS 类型信息（下发到上位机） */
typedef struct {
    uint8_t block_id;//块ID
    uint8_t kfs_type;   /* 1=R1_KFS, 2=R2_KFS, 3=FAKE */
} rc_zone_i_kfs_t;

/** 调试航向 PID 通信数据结构 */
typedef struct {
    float yaw_ref_deg;    /* 目标角度（deg） */
    float yaw_deg;        /* 实际角度值（deg） */
    float err_deg;        /* 角度误差（deg） */
    float i_term;         /* 积分项 */
    float output;         /* PID 输出（Vx分量） */
    float yaw_rate_dps;   /* 陀螺仪角速度（deg/s） */
} rc_debug_heading_hold_t;

/** 调试导航数据结构 */
typedef struct {
    float ex;          /* X方向位置误差（m） */
    float ey;          /* Y方向位置误差（m） */
    float dist;        /* 到目标距离（m） */
    float zone;        /* 0=自动, 1=手动 */
    float vy_fwd;      /* 前进速度输出 */
    float vw_str;      /* 旋转速度输出 */
} rc_debug_nav_goto_t;

/** R2 下位机状态 */
typedef enum {
    RC_STATE_IDLE       = 0,
    RC_STATE_MOVING     = 1,
    RC_STATE_AT_TARGET  = 2,
    RC_STATE_GRABBING   = 3,
    RC_STATE_DONE       = 4,
    RC_STATE_ERROR      = 5,
} rc_state_t;

/* ---------- 回调函数类型 ---------- */
typedef void (*rc_odom_callback_t)(const rc_odom_t *odom);
typedef void (*rc_path_callback_t)(const rc_path_t *path);
typedef void (*rc_kfs_callback_t)(const rc_kfs_t *kfs);
typedef void (*rc_zone_i_path_callback_t)(const rc_zone_i_path_t *path);

/* ---------- 初始化 ---------- */

/**
 * @brief 初始化协议，将ODOM帧写入内部 latest_odom，可通过 rc_get_latest_odom / rc_odom_is_valid访问
 * @param uart_send  发送字节函数（对 HAL_UART_Transmit 的封装）
 * @param get_ms     获取当前时间函数（用于超时）
 */
void rc_init(void (*uart_send)(uint8_t byte), uint32_t (*get_ms)(void));

/** 注册回调 */
void rc_set_odom_callback(rc_odom_callback_t cb);
void rc_set_path_callback(rc_path_callback_t cb);
void rc_set_kfs_callback(rc_kfs_callback_t cb);
void rc_set_zone_i_path_callback(rc_zone_i_path_callback_t cb);

/**
 * @brief 逐个喂入接收到的字节（在 UART RX 中断/回调中调用）
 * @param byte 接收到的字节
 */
void rc_feed_byte(uint8_t byte);

/** 主循环中调用，处理数据超时 */
void rc_poll(void);

/* ---------- 发送（下位机到上位机） ---------- */

/** 发送 ACK */
void rc_send_ack(uint8_t cmd, uint8_t code);

/** 发送下位机状态 */
void rc_send_status(rc_state_t state);

/** 发送 I区 KFS 类型信息（R1 识别后回传给上位机） */
void rc_send_zone_i_info(uint8_t num, const rc_zone_i_kfs_t *kfs_list);

/** 发送 R1 对接成功 */
void rc_send_dock_ok(void);

/** 发送进入 I区 */
void rc_send_go_zone_i(void);

/** 发送调试航向 PID 状态（调试通信） */
void rc_send_debug_heading_hold(const rc_debug_heading_hold_t *dbg);

/** 发送调试导航信息（调试通信） */
void rc_send_debug_nav_goto(const rc_debug_nav_goto_t *dbg);

/* ---------- 读取 ---------- */

/** 获取最近一次收到的里程计数据 */
const rc_odom_t *rc_get_latest_odom(void);

/** ODOM 数据是否有效（未超时，默认 2 秒超时） */
uint8_t rc_odom_is_valid(void);

/**
 * @brief 获取最近一次 ODOM 数据的时间戳（ms）
 * @return ms时间，若未初始化 get_ms，则返回 0xFFFFFFFF
 */
uint32_t rc_get_odom_age_ms(void);

#endif /* UPPER_PC_PROTOCOL_H */