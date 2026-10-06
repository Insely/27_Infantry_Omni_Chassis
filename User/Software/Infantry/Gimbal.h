#ifndef __GIMBAL_DIRECT_H__
#define __GIMBAL_DIRECT_H__

#include "CAN_receive_send.h"
#include "pid.h"
#include "ramp_generator.h"

#include "IMU_updata.h"
#include "motor.h"

/*函数*/
#define GIMBALMotor_init(type, id)                        DMMotor_Init(type, id) 
#define GIMBALMotor_set(id,pos, vel, tor, kp, kd)         DMMotor_Set(id, pos, vel, tor, kp, kd)
#define GIMBALMotor_get_data(id)                          DMMotor_GetData(id)
#define GIMBAL_IMU_DATA  dm_imu_gimbal

/*电机参数*/
#define GIMBALMOTOR_MAX_CURRENT MAX_CURRENT

/*内部数据类型*/
typedef struct
{
    /* PID 控制器。 */
    pid_t pitch_speed_pid;
    pid_t pitch_location_pid;
    pid_t yaw_speed_pid;
    pid_t yaw_location_pid;
    pid_t pitch_auto_speed_pid;
    pid_t pitch_auto_location_pid;
    pid_t yaw_auto_speed_pid;
    pid_t yaw_auto_location_pid;


    /*-------状态量-------*/
    //速度
    float yaw_speed_now;
    float pitch_speed_now;
    //位置 
    float yaw_location_now;
    float pitch_location_now;
    enum gimbal_state_e
        {
            RIGHTING,   // 纠偏
            BEREADY,    // 就绪
            NORMALLY,   // 正常状态

        } State;
    /*-------目标量-------*/
    //速度
    float yaw_speed_set;
    float pitch_speed_set;
    //位置 
    float yaw_location_set;
    float pitch_location_set;
    // 斜坡
    RampGenerator pitch_ramp;
    RampGenerator yaw_ramp;
    //力矩
    float force[2];
    //位置
    float position[3];

}Gimbal_t;

extern Gimbal_t Gimbal;

void Gimbal_Init();
void Gimbal_Tasks();
void Gimbal_SetPitchAngle(float angle);
void Gimbal_SetYawAngle(float angle);

#endif 
