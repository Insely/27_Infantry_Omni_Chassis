/**
 * @file  robot_param.h
 * @brief 全向轮步兵的兵种、双板通信及硬件参数配置。
 *
 * @note Gimbal 与 Chassis 工程各保留一份本文件。
 *       除 BOARD_GIMBAL / BOARD_CHASSIS 外，两块板的配置必须保持一致。
 */

#ifndef INCLUDED_ROBOT_PARAM_H
#define INCLUDED_ROBOT_PARAM_H

#ifdef __cplusplus
extern "C" {
#endif

/*================================================ 板卡身份 ===========================================================*/

/* 板卡身份由工程目录决定，不作为车型选项修改。 */
#define BOARD_GIMBAL  0
#define BOARD_CHASSIS 1

/*================================================ 兵种选择 ===========================================================*/

#define ROBOT_INFANTRY 0
#define ROBOT_SENTRY   1

/* ★ 切换兵种只需修改这一项；两块板必须保持一致。 */
#ifndef ROBOT_TYPE
#define ROBOT_TYPE ROBOT_INFANTRY
#endif

/* 当前 ROBOT_SENTRY 仅预留目录和通信框架，不驱动步兵执行机构。 */
#if (ROBOT_TYPE != ROBOT_INFANTRY) && (ROBOT_TYPE != ROBOT_SENTRY)
#error "ROBOT_TYPE 必须设为 ROBOT_INFANTRY 或 ROBOT_SENTRY"
#endif

/*================================================ 底盘选择 ===========================================================*/

#define CHASSIS_OMNI 1

/* 本工程仅适配并验证全向轮步兵底盘。 */
#ifndef CHASSIS_TYPE
#define CHASSIS_TYPE CHASSIS_OMNI
#endif

#if (CHASSIS_TYPE != CHASSIS_OMNI)
#error "当前工程仅支持 CHASSIS_OMNI 全向轮底盘"
#endif

/* 底盘解算和底盘电机只在 Chassis 板参与编译。 */
#define USE_CHASSIS_HELM 0
#define USE_CHASSIS_OMNI BOARD_CHASSIS
#define USE_CHASSIS      BOARD_CHASSIS

/*================================================ 双板通信 ===========================================================*/

/*
 * 两块板必须选择相同的传输方式和端口。
 * CAN 使用独立逻辑帧；RS485 与普通 UART 共用串行组帧、CRC 和拆包协议。
 */
#define BOARD_LINK_TRANSPORT_CAN    1U
#define BOARD_LINK_TRANSPORT_RS485  2U
#define BOARD_LINK_TRANSPORT_UART   3U

/* ★ 默认使用原步兵双板接线：CAN2。 */
#ifndef BOARD_LINK_TRANSPORT
#define BOARD_LINK_TRANSPORT BOARD_LINK_TRANSPORT_CAN
#endif

/* CAN: 1=CAN1，2=CAN2，3=CAN3。 */
#ifndef BOARD_LINK_CAN_BUS
#define BOARD_LINK_CAN_BUS 2U
#endif

/* RS485: 2=USART2，3=USART3；当前默认 USART2。 */
#define BOARD_LINK_RS485_UART 2U

/* 普通 UART: 当前仅支持 USART1。 */
#define BOARD_LINK_UART_PORT 1U

#if (BOARD_LINK_TRANSPORT < BOARD_LINK_TRANSPORT_CAN) || \
    (BOARD_LINK_TRANSPORT > BOARD_LINK_TRANSPORT_UART)
#error "BOARD_LINK_TRANSPORT 必须设为 CAN、RS485 或 UART"
#endif

#if (BOARD_LINK_CAN_BUS < 1U) || (BOARD_LINK_CAN_BUS > 3U)
#error "BOARD_LINK_CAN_BUS 必须设为 1、2 或 3"
#endif

/*================================================ 全局功能 ===========================================================*/

/* 模式开关：0=关闭，1=启用。 */
#define CONTROL_TYPE (1)  /* 正常控制模式 */
#define DEBUG_TYPE   (0)  /* 调试模式 */

/* 功能模块：0=关闭，1=启用。 */
#define USE_GIMBAL (1)  /* 云台 */
#define USE_SHOOT  (1)  /* 发射机构 */

/* 电机驱动：0=不编译，1=编译。 */
#define USE_DJIMotor   (1)  /* DJI M2006 / M3508 / GM6020 */
#define USE_DMMotor    (1)  /* 达妙 MIT 电机 */
#define USE_DMMotor124 (0)  /* 达妙 1 拖 4 模式 */
#define USE_LZMotor    (0)  /* 灵足电机 */

/*================================================ 电机类型 ===========================================================*/

typedef enum
{
    DJI_M2006 = 0,
    DJI_M3508,
    DJI_GM6020,
    DM_4310,
    DM_4340,
    LZ_00,
} Motor_Type_e;

/*
 * 达妙反馈 MASTER_ID 基址。
 * 原步兵电机配置为 0x01~0x06；不要直接改成 fold 使用的 0x11~0x16。
 */
#ifndef DM_FEEDBACK_ID_BASE
#define DM_FEEDBACK_ID_BASE 0x01U
#endif

/*================================================ 全向轮底盘 =========================================================*/

/* 底盘几何参数。 */
#define R_body     (0.23775f)  /* m，底盘中心到轮组安装中心的距离 */
#define r_wheel    (0.07697f)  /* m，轮子半径 */
#define WHEEL_RATIO (14.00f)   /* 底盘 M3508 减速比 */

/* 驱动电机 CAN ID；顺序为左前、右前、左后、右后。 */
#define WHEEL_FL CAN_1_4
#define WHEEL_FR CAN_1_3
#define WHEEL_BL CAN_1_2
#define WHEEL_BR CAN_1_1

/*================================================ 云台参数 ===========================================================*/

/* Pitch 机械限位，单位：degree。 */
#define PITCHI_MAX_ANGLE (30.0f)
#define PITCHI_MIN_ANGLE (-24.0f)

/* 电机减速比。 */
#define YAW_RATIO   (1)
#define PITCH_RATIO (1)

/* 电机 CAN ID：Yaw 位于 Chassis 板，Pitch 位于 Gimbal 板。 */
#define YAWMotor   DM_CAN_3_2
#define PITCHMotor DM_CAN_1_1

/* 电机型号。 */
#define GIMBAL_YAW_MOTOR_TYPE   ((Motor_Type_e)DM_4310)
#define GIMBAL_PITCH_MOTOR_TYPE ((Motor_Type_e)DM_4310)

/* 电机零点，单位：degree；保留原车标定值。 */
#define YAW_ZERO   (102.5f)
#define PITCH_ZERO (-115.58f)

/*================================================ 发射机构 ===========================================================*/

/* 机械参数。 */
#define FRIC_RADIUS (0.03f)  /* m，摩擦轮半径 */
#define BULLET_NUM  (9)      /* 拨弹盘一圈容纳的弹丸数 */

/* 电机 CAN ID：摩擦轮位于 Gimbal 板，拨弹电机位于 Chassis 板。 */
#define ShootMotor_L  CAN_1_5
#define ShootMotor_R  CAN_1_7
#define TRIGGER_MOTOR CAN_3_1

/* 电机型号。 */
#define TRIGGER_MOTOR_TYPE ((Motor_Type_e)DJI_M2006)
#define SHOOT_MOTOR_TYPE   ((Motor_Type_e)DJI_M3508)

/* 拨弹速度，单位：rpm。 */
#define TRIGGER_SPEED_H (9000)
#define TRIGGER_SPEED_M (7000)
#define TRIGGER_SPEED_L (3000)

/* 摩擦轮速度，单位：rpm。FRIC_SPEED_REDAY 沿用现有代码中的宏名称。 */
#define FRIC_SPEED_BEGIN (-2000)
#define FRIC_SPEED_REDAY (6000)
#define FRIC_SPEED_DEBUG (1500)

/*================================================ 遥控参数 ===========================================================*/

#define MOVE_SENSITIVITY  (10.0f)   /* 底盘移动灵敏度 */
#define PITCH_SENSITIVITY (0.005f)  /* Pitch 鼠标灵敏度 */
#define YAW_SENSITIVITY   (0.003f)  /* Yaw 鼠标灵敏度 */

#ifdef __cplusplus
}
#endif

#endif /* INCLUDED_ROBOT_PARAM_H */
