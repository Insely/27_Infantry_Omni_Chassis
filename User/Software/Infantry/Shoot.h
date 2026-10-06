#ifndef __SHOOT_H__
#define __SHOOT_H__

#include "CAN_receive_send.h"
#include "pid.h"

#include "motor.h"
/*发射机构参数*/
enum shoot_speed_e // 摩擦轮速度
{
    SHOOT_SPEED_BEGIN = FRIC_SPEED_BEGIN,// 开始反转值
    SHOOT_SPEED_CLOSE = 0,    // 停止速度值
    SHOOT_SPEED_READY = FRIC_SPEED_REDAY,  // 正常工作值
    SHOOT_SPEED_DEBUG = FRIC_SPEED_DEBUG, // 退弹低速值
};

enum trigger_speed_e // 拨弹电机速度
{
    TRIGGER_SPEED_CLOSE = 0,
    TRIGGER_SPEED_HIGH  = TRIGGER_SPEED_H,
    TRIGGER_SPEED_MID   = TRIGGER_SPEED_M,
    TRIGGER_SPEED_LOW   = TRIGGER_SPEED_L,
    TRIGGER_SPEED_DEBUG = 3000,

};

/* 内部调用 */
#define SHOOTMotor_init(type, id)    DJIMotor_Init(type, id)
#define SHOOTMotor_set(val, id)      DJIMotor_Set(val, id)
#define TriggerMotor_init(type, id)  DJIMotor_Init(type, id)
#define TriggerMotor_set(val, id)    DJIMotor_Set(val, id)
#define TriggerMotor_get_data(id)    DJIMotor_GetData(id)
#define SHOOTMotor_get_data(id)      DJIMotor_GetData(id)

/* 电机参数 */
#define SHOOTMOTOR_MAX_CURRENT MAX_CURRENT
#define TRIGGER_ANGLE_INC 2880.0f   //拨弹盘转一个弹丸需要的角度
#define SINGLE_FIRE_INTERVAL 800   //单发最小间隔时间(ms)

/*内部数据类型*/
typedef struct
{
    /* PID 控制器。 */
    pid_t shoot_L_speed_pid;
    pid_t shoot_R_speed_pid;
    pid_t trigger_speed_pid;
    pid_t trigger_location_pid;

    /*-------状态量-------*/
    float shoot_speed_L_now;
    float shoot_speed_R_now;
    float trigger_speed_now;
    float trigger_current_now; // 电流

    /*-------目标量-------*/
    enum trigger_speed_e trigger_speed_set;
    enum shoot_speed_e shoot_speed_set;
    float current[3];  // 电流
    double trigger_angle_now;
}Shoot_t;
extern Shoot_t Shoot;

/* 外部调用 */
void Shoot_Init();
void Shoot_Tasks();

#endif 
