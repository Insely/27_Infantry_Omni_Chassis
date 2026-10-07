#ifndef __CHASSIS_HELM_H__
#define __CHASSIS_HELM_H__

#include "robot_param.h"
#if (USE_CHASSIS_HELM != 0)

#include "CAN_receive_send.h"

#include "pid.h"
#include "User_math.h"
#include "ramp_generator.h"

#include "motor.h"

/*电机配置*/

#define CHASSISMotor_init(type, id)  DJIMotor_Init(type ,id)
#define CHASSISMotor_set(val, id)    DJIMotor_Set(val, id)
#define CHASSISMotor_get_data(id)    DJIMotor_GetData(id)

/*电机参数*/
#define CHASSISMOTOR_MAX_CURRENT MAX_CURRENT
#define CHASSISMOTOR_T_A DJIMOTOR_T_A

// 轮子向前时6020编码器的值
#define X_AXIS_ECD_FL 331                         
#define X_AXIS_ECD_FR 7836
#define X_AXIS_ECD_BL 2365
#define X_AXIS_ECD_BR 3089

/*内部数据类型*/
typedef struct
{
    /**************************以下是轮向电机结构体定义***************************/

    struct
    {
        pid_t chassis_speed_pid_forward_FL;
        float V_FL;
        float opposite_direction_FL;
        float target_velocity_FL, current_velocity_FL;
        int16_t wheel_current_FL;
    } forward_FL;
    struct
    {
        pid_t chassis_speed_pid_forward_FR;       
        float V_FR;
        float opposite_direction_FR;
        float target_velocity_FR, current_velocity_FR;
        int16_t wheel_current_FR;
    } forward_FR;
    struct
    {
        pid_t chassis_speed_pid_forward_BL;
        float V_BL;
        float opposite_direction_BL;
        float target_velocity_BL, current_velocity_BL;
        int16_t wheel_current_BL;
    } forward_BL;
    struct
    {
        pid_t chassis_speed_pid_forward_BR;
        float V_BR;
        float opposite_direction_BR;
        float target_velocity_BR, current_velocity_BR;
        int16_t wheel_current_BR;
    } forward_BR;

    /**************************以下是舵向电机结构体定义***************************/


    struct
    {
        pid_t chassis_location_pid_turn_FL;
        pid_t chassis_speed_pid_turn_FL;
        float set, now;
        float set_turn_FL_speed;
        float target_angle_turn_FL;
        float set_angle_FL;
        float set_ECD_FL;
        int16_t current_steer_FL;
    } turn_FL;
    struct
    {
        pid_t chassis_location_pid_turn_FR;
        pid_t chassis_speed_pid_turn_FR;
        float set, now;
        float set_turn_FR_speed;
        float target_angle_turn_FR;
        float set_angle_FR;
        float set_ECD_FR;
        int16_t current_steer_FR;
    } turn_FR;
    struct
    {
        pid_t chassis_location_pid_turn_BL;
        pid_t chassis_speed_pid_turn_BL;
        float set, now;
        float set_turn_BL_speed;
        float target_angle_turn_BL;
        float set_angle_BL;
        float set_ECD_BL;
        int16_t current_steer_BL;
    } turn_BL;
    struct
    {
        pid_t chassis_location_pid_turn_BR;
        pid_t chassis_speed_pid_turn_BR;
        float set, now;
        float set_turn_BR_speed;
        float target_angle_turn_BR;
        float set_angle_BR;
        float set_ECD_BR;
        int16_t current_steer_BR;
    } turn_BR;
    struct
    {
        float x, y, r, yaw;
        float now_x, now_y, now_r;
        float max_x, max_y, max_r; // m/s
    } speed;

    struct
    {
        float x, y, r;
    } speed_RC;

    struct
    {
        float now_x, now_y, now_r;
        float max_x, max_y, max_r; // m/s^2
    } acc;
    pid_t chassis_follow_pid;
    uint8_t is_open_cap;
    float relative_angle;
    float feedforward_angle;

}Chassis_t;
extern Chassis_t chassis;

/*外部调用*/
void Chassis_Init();
void Chassis_Tasks();
void Chassis_SetX(float x);
void Chassis_SetY(float y);
void Chassis_SetR(float r);
void Chassis_SetAccel(float acc);
void Rudder_Angle_Calculation(float x, float y, float w);
void Nearby_Transposition();
void Chassis_Calculater(float vx,float vy,float vw);
void Mode_Switch();
#define CHASSIS_TASK_TIME 1 // 底盘任务刷新间隔

#endif /* USE_CHASSIS_HELM */

#endif /* __CHASSIS_HELM_H__ */
