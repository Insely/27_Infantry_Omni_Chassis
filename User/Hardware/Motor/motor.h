#ifndef __MOTOR_H__

#ifdef __cplusplus
extern "C" {
#endif

#define __MOTOR_H__
#include "stdint.h"

#include "User_math.h"
#include "robot_param.h"
#include "CAN_receive_send.h"


// CAN通道数量
#define QUANTITY_OF_CAN 3       // 你所用的can的数量，（比所用的多就行了，喵板上设3别改了）


#if (USE_DJIMotor == 1)

#define QUANTITY_OF_DJIMOTOR 11 // 挂载电机最多的一路can上所用的电机
// 大疆电机参数
#define ECD_TO_ANGEL_DJI 0.043945f //(360/8192),将编码器值转化为角度制
#define DJIMOTOR_T_A 0.3           // 3508转矩常数
#define ECD_MAX 8192.0f            // 编码器最大值
#define M3508_P WHEEL_RATIO        // M3508电机减速比（不同车的减速比不同，所以在参数配置文件中设置）
#define M2006_P 36.0f              // M2006电机减速比
#define MAX_CURRENT 16384          // M2006+M3508最大电流 20A / MAX_CURRENT
#define MAX_6020_VOL 30000         // 6020最大电压 24V / MAX_6020_VOL

/* DJImotorCAN send and receive ID */
typedef enum
{
    CAN_20063508_1_4_send_ID = 0x200,

    CAN_20063508_5_8_send_ID = 0x1FF,

    CAN_6020_1_4_send_ID = 0x1FF,

    CAN_6020_5_7_send_ID = 0x2FF,

    CAN_ID1 = 0x201,

    DJI_CAN_1 = 0,

    DJI_CAN_2 = 1,

    DJI_CAN_3 = 2,

    CAN_20063508_1_4_ID = 3,

    CAN_20063508_5_8_ID = 4,

    CAN_6020_1_4_ID = 5,

    CAN_6020_5_7_ID = 6,
} DJIcan_send_id_e;


// 6020从 CAN_1_5(对应ID1) 到 CAN_1_6020_7（对应ID7）
// 3508/2006从 CAN_1_1(对应ID1) 到 CAN_1_8（对应ID8）
// 显然，6020与3508/2006存在重叠ID,设置时请注意
typedef enum
{
    CAN_1_1 = 0,  // 0
    CAN_1_2,      // 1
    CAN_1_3,      // 2
    CAN_1_4,      // 3
    CAN_1_5,      // 4
    CAN_1_6,      // 5
    CAN_1_7,      // 6
    CAN_1_8,      // 7
    CAN_1_6020_5, // 8
    CAN_1_6020_6, // 9
    CAN_1_6020_7, // 10

    CAN_2_1, // 11
    CAN_2_2, // 12
    CAN_2_3,
    CAN_2_4,
    CAN_2_5, // 15
    CAN_2_6,
    CAN_2_7, // 17
    CAN_2_8,
    CAN_2_6020_5,
    CAN_2_6020_6,
    CAN_2_6020_7, // 21

    CAN_3_1, // 22
    CAN_3_2, // 23
    CAN_3_3,
    CAN_3_4,
    CAN_3_5,
    CAN_3_6,
    CAN_3_7,
    CAN_3_8,
    CAN_3_6020_5,
    CAN_3_6020_6,
    CAN_3_6020_7, // 32
    DJI_MOTOR_NUM,
} DJIcan_id;

typedef struct
{
    // 控制数据
    int16_t set; // 设定的电流 / 电压
    Motor_Type_e Motor_type;

    // 原始数据
    uint16_t ecd;          // 编码器数值
    int16_t speed_rpm;     // 转速RPM
    int16_t given_current; // 实际转矩电流
    uint8_t temperate;     // 温度（获取不到）
    uint16_t last_ecd;     // 上一次编码器的数值

    // 计算数据
    long long ecd_cnt;  // 编码器计数器
    double angle_cnt;   // 转过的总角度 degree
    double angle_zero;  // 编码器0点角度 degree
    double angle;       // -180~180 degree
    double round_speed; // 出轴转速 rpm
} DJI_motor_data_s;

void DJIMotor_Init(Motor_Type_e motor_type, DJIcan_id motor_id);
void DJIMotor_SetZero(double zero_angle, DJIcan_id motor_id);
void DJIMotor_Set(int16_t val, DJIcan_id motor_id);
void DJIMotor_DecodeCandata(FDCAN_HandleTypeDef *hfdcan, uint32_t id, uint8_t *data);
void DJIMotor_SendCurrent(DJIcan_send_id_e CAN_Send_ID , DJIcan_send_id_e CAN_Type);
DJI_motor_data_s DJIMotor_GetData(DJIcan_id motor_id);

#endif // USE_DJIMotor



#if (USE_DMMotor == 1)

#include "dm_motor_drv.h"
#include "dm_motor_ctrl.h"

// 每个CAN总线的电机数量
#define QUANTITY_OF_DMMOTOR 6

typedef enum
{
    DM_CAN_1_1 = 0,  // 0
    DM_CAN_1_2,      // 1
    DM_CAN_1_3,      // 2
    DM_CAN_1_4,      // 3
    DM_CAN_1_5,      // 4
    DM_CAN_1_6,      // 5

    DM_CAN_2_1, // 6
    DM_CAN_2_2, // 7
    DM_CAN_2_3,
    DM_CAN_2_4,
    DM_CAN_2_5, // 
    DM_CAN_2_6,

    DM_CAN_3_1, // 
    DM_CAN_3_2, // 
    DM_CAN_3_3,
    DM_CAN_3_4,
    DM_CAN_3_5,
    DM_CAN_3_6,
    DM_MOTOR_NUM,

} DMcan_id;

// 达妙电机数据结构
typedef struct
{
    float T;
    float W;
    float Pos;
    
    uint32_t feedback_tick;
    uint8_t feedback_ready;
    DM_motor_t motor_data;
} DM_motor_data_s;

void DMMotor_Init(Motor_Type_e motor_type,DMcan_id motor_id);
void DMMotor_Set(DMcan_id motor_id, float pos, float vel, float tor, float kp, float kd);
void DMMotor_SetZero(double zero_angle, DMcan_id motor_id);
int DMMotor_SendCtrl(DMcan_id motor_id);
void DMMotor_DecodeCandata(FDCAN_HandleTypeDef *hfdcan, uint32_t id, uint8_t *data);
DM_motor_data_s DMMotor_GetData(DMcan_id motor_id);

extern DM_motor_data_s DM_Motor_data[QUANTITY_OF_CAN][6];

#endif // USE_DAMIAO_MOTOR

#if (USE_DMMotor124 == 1)

#define QUANTITY_OF_DMMOTOR124 8
#define ECD_TO_ANGEL_DM 0.043945f //(360/8192),将编码器值转化为角度制

/* DJImotorCAN send and receive ID */
typedef enum
{
    DM_J4310_1_4_send_ID = 0x3FE,

    DM_J4310_5_8_send_ID = 0x4FE,

    DM_CAN_1 = 0,

    DM_CAN_2 = 1,

    DM_CAN_3 = 2,

    DM_J4310_1_4_ID = 3,

    DM_J4310_5_8_ID = 4,

    DM_ID1 = 0x301,
} DMcan_send_id_e;



typedef enum
{
    DM_1_1 = 0,  // 0
    DM_1_2,      // 1
    DM_1_3,      // 2
    DM_1_4,      // 3
    DM_1_5,      // 4
    DM_1_6,      // 5
    DM_1_7,      // 6
    DM_1_8,      // 7

    DM_2_1,  // 8
    DM_2_2,     
    DM_2_3,      
    DM_2_4,     
    DM_2_5,    
    DM_2_6,     
    DM_2_7,     
    DM_2_8,
    
    DM_3_1,  // 16
    DM_3_2,      
    DM_3_3,      
    DM_3_4,      
    DM_3_5,      
    DM_3_6,     
    DM_3_7,      
    DM_3_8,      
    DM_NUM,

} DM_can_id;

typedef enum
{
   J4310 = 0,
} DMMotor_Type_e;

typedef struct
{
    // 控制数据
    int16_t set; // 设定的电流 / 电压
    DMMotor_Type_e Motor_type;

    // 原始数据
    uint16_t ecd;          // 编码器数值
    uint16_t last_ecd;     // 上一次编码器的数值
    float speed_rpm;        // 转速RPM
    int16_t given_current; // 实际转矩电流 mA
    uint8_t coil_temp;     // 线圈温度 摄氏度
    uint8_t pcb_temp;     // pcb温度
    

    // 计算数据
    long long ecd_cnt;  // 编码器计数器
    double angle_cnt;   // 转过的总角度 degree
    double angle_zero;  // 编码器0点角度 degree
    double angle;       // -180~180 degree
    double round_speed; // 出轴转速 rpm
} DM_motor124_data_s;

void DMMotor124_Init(DMMotor_Type_e motor_type, DM_can_id motor_id);
void DMMotor124_Set(int16_t val, DM_can_id motor_id);
void DMMotor124_GetProcessMotorData(DM_motor124_data_s *ptr, uint8_t data[]);
void DMMotor124_DecodeCandata(FDCAN_HandleTypeDef *hfdcan, uint32_t id, uint8_t *data);
void DMMotor124_SendCurrent(DMcan_send_id_e CAN_Send_ID , DMcan_send_id_e CAN_Type);
void DMMotor124_SetZero(double zero_angle, DM_can_id motor_id);
extern DM_motor124_data_s DMMotor124_GetData(DM_can_id motor_id);


#endif // USE_DAMIAO_MOTOR

#if (USE_LZMotor == 1)
#include "LZ_motor_driver.h"

// 每个CAN总线的电机数量
#define QUANTITY_OF_LZMOTOR 6

// 电机ID枚举（支持3路CAN，每路6个电机）
typedef enum {
    LZ_CAN_1_1 = 0,
    LZ_CAN_1_2,
    LZ_CAN_1_3,
    LZ_CAN_1_4,
    LZ_CAN_1_5,
    LZ_CAN_1_6,
    
    LZ_CAN_2_1,
    LZ_CAN_2_2,
    LZ_CAN_2_3,
    LZ_CAN_2_4,
    LZ_CAN_2_5,
    LZ_CAN_2_6,
    
    LZ_CAN_3_1,
    LZ_CAN_3_2,
    LZ_CAN_3_3,
    LZ_CAN_3_4,
    LZ_CAN_3_5,
    LZ_CAN_3_6,
    
    LZ_MOTOR_NUM
} LZ_Motor_ID_t;

void LZMotor_init(LZ_Motor_ID_t motor_id);
void LZMotor_enable(LZ_Motor_ID_t motor_id);
void LZMotor_disable(LZ_Motor_ID_t motor_id);
void LZMotor_set_mode(LZ_Motor_ID_t motor_id, LZ_Mode_t mode);
void LZMotor_set_params(LZ_Motor_ID_t motor_id, float pos, float vel, float tor, float kp, float kd, float current_limit);
void LZMotor_send_command(LZ_Motor_ID_t motor_id);
void LZMotor_decode_candata(FDCAN_HandleTypeDef *hfdcan, uint32_t id, uint8_t *data);

// 获取电机对象指针
LZ_Motor_t* LZMotor_get(LZ_Motor_ID_t motor_id);

#endif // USE_LINGZU_MOTOR

#ifdef __cplusplus
}
#endif
#endif // !__MOTOR_H__
