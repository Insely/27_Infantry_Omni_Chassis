/**
  * @file       robot_param.h
  * @brief      机器人参数配置文件（舵轮/全向轮 通过 ROBOT_TYPE 切换）
  */

#ifndef INCLUDED_ROBOT_PARAM_H
#define INCLUDED_ROBOT_PARAM_H

/*================================================ 车型选择 ===========================================================*/
// 0 = 舵轮步兵 (HELM)
// 1 = 全向轮步兵 (OMNI)
// 2 = 不使用底盘（仅云台+发射）
// ★ 切换车型只需改这一行 ★
#define ROBOT_TYPE  1

/* 根据 ROBOT_TYPE 自动派生底盘宏 */
#if   (ROBOT_TYPE == 0)
  #define USE_CHASSIS_HELM  (1)
  #define USE_CHASSIS_OMNI  (0)
#elif (ROBOT_TYPE == 1)
  #define USE_CHASSIS_HELM  (0)
  #define USE_CHASSIS_OMNI  (1)
#elif (ROBOT_TYPE == 2)
  #define USE_CHASSIS_HELM  (0)
  #define USE_CHASSIS_OMNI  (0)
#else
  #error "ROBOT_TYPE 未定义或不合法，请设为 0(舵轮) 1(全向轮) 2(无底盘)"
#endif

/*------------------------------------------------- Global -----------------------------------------------------------*/

//模式选择      （0不使用  1使用）
#define CONTROL_TYPE     (1)   // 控制模式
#define DEBUG_TYPE       (0)   // 调试模式模式    

//模块类型选择
#define USE_GIMBAL       (1)   // 启用云台
#define USE_SHOOT        (1)   // 启用发射机构

//电机类型选择  （0不使用  1使用）
#define USE_DJIMotor     (1)   // 大疆电机
#define USE_DMMotor      (1)   // 达妙电机
#define USE_DMMotor124   (0)   // 达妙电机1拖4
#define USE_LZMotor      (0)   // 灵足电机


/*------------------------------------------------- Chassis ----------------------------------------------------------*/

#if (USE_CHASSIS_HELM != 0)
  /* ========== 舵轮底盘参数 ========== */
  #define WHEEL_RADIUS     (0.04050f)   // (m)轮子半径
  #define WHEEL_TRACK      (0.2262741699f)  // (m)轮组到旋转中心的距离
  #define WHEEL_RATIO      (15.8f)      // 底盘电机减速比

  // 驱动电机 CAN ID
  #define WHEEL_MOVE_FR CAN_3_1
  #define WHEEL_MOVE_FL CAN_3_2
  #define WHEEL_MOVE_BR CAN_1_1
  #define WHEEL_MOVE_BL CAN_1_2

  // 舵向电机 CAN ID
  #define WHEEL_TURN_FR CAN_3_5
  #define WHEEL_TURN_FL CAN_3_6
  #define WHEEL_TURN_BR CAN_1_5
  #define WHEEL_TURN_BL CAN_1_6

#elif (USE_CHASSIS_OMNI != 0)
  /* ========== 全向轮底盘参数 ========== */
  #define R_body   0.23775f   // 底盘安装半径（m）
  #define r_wheel  0.07697f   // 轮子半径（m）
  #define WHEEL_RATIO (14.00f)  // 底盘电机减速比

  // 驱动电机 CAN ID
  #define WHEEL_FL CAN_1_4
  #define WHEEL_FR CAN_1_3
  #define WHEEL_BL CAN_1_2
  #define WHEEL_BR CAN_1_1

#else
  /* ========== 无底盘模式默认值 ========== */
  #define WHEEL_RATIO (15.8f)  // motor.h 需要此宏

#endif



/*------------------------------------------------- Gimbal -----------------------------------------------------------*/

//云台物理参数
#define PITCHI_MAX_ANGLE (45.0f)      // 最大仰角
#define PITCHI_MIN_ANGLE (-3.0f)      // 最大俯角 
#define YAW_RATIO   (1)               // yaw轴电机减速比
#define PITCH_RATIO (1)               // pitch轴电机减速比

//电机CAN ID
#define YAWMotor   DM_CAN_3_2
#define PITCHMotor DM_CAN_3_1

//电机种类
#define GIMBAL_YAW_MOTOR_TYPE   ((Motor_Type_e)DM_4310)
#define GIMBAL_PITCH_MOTOR_TYPE ((Motor_Type_e)DM_4310)

//电机零点设置
#define YAW_ZERO   (102.5f)
#define PITCH_ZERO (-115.58f)


/*------------------------------------------------- Shoot -------------------------------------------------------------*/

//发射机构物理参数
#define FRIC_RADIUS 0.03f              // (m)摩擦轮半径
#define BULLET_NUM 9                  // 拨弹盘容纳弹丸个数

//电机ID
#define ShootMotor_L  CAN_1_5
#define ShootMotor_R  CAN_1_7
#define TRIGGER_MOTOR CAN_3_1

//电机种类
#define TRIGGER_MOTOR_TYPE  ((Motor_Type_e)DJI_M2006)
#define SHOOT_MOTOR_TYPE    ((Motor_Type_e)DJI_M3508)

//拨弹速度
#define TRIGGER_SPEED_H     (9000) // 高射频
#define TRIGGER_SPEED_M     (7000) // 中射频
#define TRIGGER_SPEED_L     (3000) // 低射频

//摩擦轮速度
#define FRIC_SPEED_BEGIN    (-2000)  // 开始反转
#define FRIC_SPEED_REDAY    (6000) // 正常工作值
#define FRIC_SPEED_DEBUG    (1500) // 退弹低速值


/*------------------------------------------------- REMOTE -------------------------------------------------------------*/

//灵敏度
#define MOVE_SENSITIVITY 10.0f   // 移动灵敏度
#define PITCH_SENSITIVITY 0.005f // pitch轴灵敏度
#define YAW_SENSITIVITY 0.003f     // yaw轴灵敏度

/*------------------------------------------------- MOTOR -------------------------------------------------------------*/

// 可用电机类型
typedef enum __MotorType {
    DJI_M2006 = 0,       // 大疆M2006
    DJI_M3508,           // 大疆M3508
    DJI_GM6020,          // 大疆GM6020
    DM_4310,             // 达妙4310
    LZ_00,               // 灵足00
} Motor_Type_e;

#endif /* INCLUDED_ROBOT_PARAM_H */
