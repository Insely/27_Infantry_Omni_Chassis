#ifndef __CHASSIS_OMNI_H__
#define __CHASSIS_OMNI_H__

#include "robot_param.h"
#if (USE_CHASSIS_OMNI != 0)

#include "CAN_receive_send.h"

#include "pid.h"
#include "User_math.h"
#include "ramp_generator.h"
#include "robot_param.h"

#include "motor.h"


//---------------------------
// 逆运动学宏定义（速度 → 轮速）（M_S → RAD_S）
//---------------------------
// 右手坐标系：前 X、左 Y、上 Z；vx 向前、vy 向左、omega 俯视逆时针为正。
// 旧坐标转换：vx_old = -vy，vy_old = vx，omega_old = -omega。
// 输出：四个轮子的转速（浮点型）

//逆解左前轮 FL 速度计算
#define IK_WHEEL_FL(vx, vy, omega) \
    ((+SQRT1_2 * (vx) - SQRT1_2 * (vy) - R_body * (omega)) / r_wheel)
//逆解右前轮 FR 速度计算
#define IK_WHEEL_FR(vx, vy, omega) \
    ((-SQRT1_2 * (vx) - SQRT1_2 * (vy) - R_body * (omega)) / r_wheel)
//逆解左后轮 BL 速度计算
#define IK_WHEEL_BL(vx, vy, omega) \
    ((+SQRT1_2 * (vx) + SQRT1_2 * (vy) - R_body * (omega)) / r_wheel)
//逆解右后轮 BR 速度计算
#define IK_WHEEL_BR(vx, vy, omega) \
    ((-SQRT1_2 * (vx) + SQRT1_2 * (vy) - R_body * (omega)) / r_wheel)

//---------------------------
// 正运动学宏定义（轮速 → 速度）（RAD_S → M_S）
//---------------------------
// 输入：四个轮速 wFL, wFR, wBL, wBR
// 输出：底盘速度 vx, vy, 角速度 omega（通过指针返回）

//正解底盘速度 vx 计算
#define FK_VX(wFL, wFR, wBL, wBR) \
    ((((wFL) + (wBL) - (wFR) - (wBR)) * SQRT2 * r_wheel) / 4.0f)
//正解底盘速度 vy 计算
#define FK_VY(wFL, wFR, wBL, wBR) \
    ((((wBL) + (wBR) - (wFL) - (wFR)) * SQRT2 * r_wheel) / 4.0f)

// 计算 omega（角速度）
#define FK_OMEGA(wFL, wFR, wBL, wBR) \
    (-(((wFL) + (wFR) + (wBL) + (wBR)) * r_wheel) / (4.0f * R_body))



//---------------------------
// 逆动力学宏定义（底盘驱动力 → 轮毂力矩）（F → T）
//---------------------------
// 输入：Fx 向前、Fy 向左、T 绕 +Z 俯视逆时针为正。
// 输出：四个轮子的力矩（浮点型）

#define FORCE_COEF (0.5f * SQRT1_2)  // 1 / (2*sqrt(2))

#define IK_WHEEL_FL_T(Fx, Fy, T) \
    ((((Fx) - (Fy)) * FORCE_COEF - (T) / (4.0f * R_body)) * r_wheel)

#define IK_WHEEL_FR_T(Fx, Fy, T) \
    (((-(Fx) - (Fy)) * FORCE_COEF - (T) / (4.0f * R_body)) * r_wheel)

#define IK_WHEEL_BL_T(Fx, Fy, T) \
    ((((Fx) + (Fy)) * FORCE_COEF - (T) / (4.0f * R_body)) * r_wheel)

#define IK_WHEEL_BR_T(Fx, Fy, T) \
    (((-(Fx) + (Fy)) * FORCE_COEF - (T) / (4.0f * R_body)) * r_wheel)

/*电机配置*/
//电机初始化函数
#define CHASSISMotor_init(type, id)  DJIMotor_Init(type ,id)
#define CHASSISMotor_set(val, id)    DJIMotor_Set(val, id)
#define CHASSISMotor_get_data(id)    DJIMotor_GetData(id)


/*电机参数*/
#define CHASSISMOTOR_MAX_CURRENT MAX_CURRENT
#define CHASSISMOTOR_T_A DJIMOTOR_T_A



/*内部数据类型*/
typedef struct
{
    /*-------PID-------*/
    pid_t chassis_speed_pid_FL;
    pid_t chassis_speed_pid_FR;
    pid_t chassis_speed_pid_BL;
    pid_t chassis_speed_pid_BR;
    pid_t chassis_T_pid_x;
    pid_t chassis_T_pid_y;
    pid_t chassis_T_pid_w;
    pid_t chassis_follow_pid;
    pid_t chassis_power_pid;

    /*-------状态量-------*/
    //车速
    float Vx_now;
    float Vy_now;
    float W_now;
    //轮速
    float speed_now_FL;
    float speed_now_FR;
    float speed_now_BL;
    float speed_now_BR;
    //角度
    float chassis_pitch_angle;
    float chassis_yaw_angle;

    /*-------目标量-------*/
    //车速
    float Vx_set;
    float Vy_set;
    float W_set;
    //轮速
    float speed_set_FL;
    float speed_set_FR;
    float speed_set_BL;
    float speed_set_BR;
    //斜坡
    RampGenerator Vx_ramp;
    RampGenerator Vy_ramp;
    RampGenerator Vw_ramp;
    //电流
    float current[4];
    
}Chassis_t;

/*外部调用*/
void Chassis_Init();
void Chassis_Tasks();
void Chassis_SetX(float x);
void Chassis_SetY(float y);
void Chassis_SetR(float r);
void Chassis_SetAccel(float acc);

extern Chassis_t Chassis;

#define CHASSIS_TASK_TIME 1 // 底盘任务刷新间隔

#endif /* USE_CHASSIS_OMNI */

#endif /* __CHASSIS_OMNI_H__ */