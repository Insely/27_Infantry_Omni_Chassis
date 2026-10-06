#ifndef INCLUDED_BOARD_LINK_H
#define INCLUDED_BOARD_LINK_H

#include "robot_param.h"
#include "fdcan.h"
#include "usart.h"
#include <stdint.h>

#define BOARD_LINK_FRAME_LEN              8U     /* 板间协议逻辑消息长度，不是串行物理帧长度 */
#define BOARD_LINK_FAST_RESEND_COUNT      5U     /* 收到请求后快速重发的次数 */
#define BOARD_LINK_YAW_SPEED_LIMIT_RAD_S  20.0f  /* Yaw 速度指令限幅，单位：rad/s */

#define BL_ID_CONTROL_MODE 0x104U
#define BL_ID_TRIGGER_MODE 0x108U

/* 线协议枚举值固定，不依赖应用层枚举的排列顺序（协议版本 2）。 */
enum { BL_RC = 0, BL_LOCK = 1, BL_KEY = 2 };
enum { BL_FLOW = 0, BL_SPIN_P = 1, BL_SPIN_N = 2, BL_NAV = 3, BL_NO_FOLLOW = 4 };
enum { BL_CLOSE = 0, BL_HIGH = 1, BL_MID = 2, BL_LOW = 3, BL_SINGLE = 4, BL_DEBUG = 5 };
enum { BOARD_LINK_RX_UNHANDLED = 0, BOARD_LINK_RX_HANDLED = 1 };

typedef struct
{
    /* 云台板发送到底盘板。 */
    float vx;
    float vy;
    float w;
    volatile float yaw_speed_cmd;       /* Yaw 电机速度指令，单位：rad/s */
    float pitch;
    float yaw_cnt;
    float gyro[3];
    uint8_t control_mode;
    uint8_t chassis_mode;
    uint8_t trigger_mode;
    uint8_t auto_mode;

    uint8_t cap_mode, reset, ui_sequence, shoot_mode;
    float body_gyro_z; /* 单位：rad/s，符号方向与原底盘 IMU_data.gyro[2] 一致 */
    /* 底盘板发送到云台板。 */
    volatile float yaw_angle_cnt;
    volatile float yaw_spd;
    float initial_speed;
    uint16_t barrel_heat;
    uint16_t heat_limit;
    uint16_t projectile_allowance_17mm;
    uint8_t launching_frequency;
} BoardLink_t;

extern volatile BoardLink_t BoardLink;

void BoardLink_Init(void);
void BoardLink_TxStep(void);
void BoardLink_RequestFastResend(uint16_t message_id);

/** CAN 接收帧入口：仅在 BOARD_LINK_TRANSPORT_CAN 配置下消费 BoardLink 帧。 */
uint8_t BoardLink_RxDispatch(FDCAN_HandleTypeDef *hfdcan, uint16_t id, uint8_t data[8]);

/** RS485/普通 UART 字节流入口：支持拆包、粘包、帧头重同步和 CRC16 校验。 */
uint8_t BoardLink_UartRxDispatch(UART_HandleTypeDef *huart, const uint8_t *data, uint16_t len);

/** 串行 DMA 发送完成入口：释放 BoardLink 静态发送缓冲区。 */
void BoardLink_UartTxCpltCallback(UART_HandleTypeDef *huart);

/** 串行 UART 异常入口：重置发送忙标志和接收解析状态。 */
void BoardLink_UartErrorCallback(UART_HandleTypeDef *huart);

#endif /* INCLUDED_BOARD_LINK_H */
