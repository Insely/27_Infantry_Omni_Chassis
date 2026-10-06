#include "Chassis_omni.h"
#include "Gimbal.h"
#include "Global_status.h"

#include "referee_system.h"
#include "supercup.h"
#include "stm32_time.h"
#include "IMU_updata.h"

#include "User_math.h"
#include "robot_param.h"

#if (USE_CHASSIS_OMNI != 0)
Chassis_t Chassis;

/*-------------------- PowerLimit --------------------*/

/**
 * @brief          功率限制
 * @param          FL_current,FR_current,BL_current,BR_current:四个轮子的电流值
 * @param          FL_current,FR_current,BL_current,BR_current:四个轮子的电流值
 * @retval         percentage：功率限制的百分比
 */
float Chassis_PowerLimit(float FL_current, float FR_current, float BL_current, float BR_current,
                         float FL_speed, float FR_speed, float BL_speed, float BR_speed, float max_p)
{
    float current[4] = {FL_current, FR_current, BL_current, BR_current};
    float speed[4] = {FL_speed, FR_speed, BL_speed, BR_speed};
    float p = 0.0f;

    float a00 = 0.48872161;
    float a01 = -2.93589057e-04;
    float a10 = 5.3241338928e-05;
    float a02 = 2.0e-07;
    float a11 = 2.03985936e-06;
    float a20 = 1.25e-07;
    /*最大功率设置*/
    Supercap_SetPower(max_p - 4.0f);
    cap.cache_energy = Referee_data.Buffer_Energy;
    if ((cap.remain_vol <= 12) || (Global.Cap.mode == Not_FULL))
    {
        max_p -= 2.0f; // 2w余量
        if (cap.remain_vol <= 10)
            max_p -= 2.0f;
        if (cap.remain_vol <= 8)
            max_p -= 4.0f;
    }

    else if (cap.remain_vol > 12)
    {
        max_p += cap.remain_vol * 10;
    }
    /*估算当前功率*/
    for (int i = 0; i < 4; i++)
    {
        p += fabs(a00 + a01 * speed[i] + a10 * current[i] +
                  a02 * speed[i] * speed[i] +
                  a11 * speed[i] * current[i] +
                  a20 * current[i] * current[i]);
    }
    float percentage = max_p / p;
    if (cap.Chassis_power >= (max_p / 3.0f) ) // 底盘当前功率过小不使用闭环  && (PID_Cal(&Chassis.chassis_power_pid, cap.Chassis_power, max_p) < 0
        percentage += PID_Cal(&Chassis.chassis_power_pid, cap.Chassis_power, max_p);
    if (percentage < 0)
        return 0.0f;
    if (percentage > 1.0f) // 防止输出过大
        return 1.0f;

    return percentage;
}

/*-------------------- AngleLimit --------------------*/

/**
 * @brief          多圈角度限制
 * @param          angle:要化简的角度
 * @retval         angle:化简后的角度
 */
float Chassis_AngleLimit(float angle)
{
    uint32_t mul = fabs(angle) / 180.0f;
    if (angle > 180.0f)
    {
        if (mul % 2 == 1) // 处于-180度
            angle -= (mul + 1) * 180.0f;
        else // 处于180度
            angle -= mul * 180.0f;
    }
    if (angle < -180.0f)
    {
        if (mul % 2 == 1) // 处于180度
            angle += (mul + 1) * 180.0f;
        else // 处于-180度
            angle += mul * 180.0f;
    }
    return angle;
}

/*-------------------- Init --------------------*/

/**
 * @brief          初始化
 * @param          none
 * @retval         none
 */
void Chassis_Init()
{
    // 底盘电机初始化
    CHASSISMotor_init(DJI_M3508, WHEEL_FL);
    CHASSISMotor_init(DJI_M3508, WHEEL_FR);
    CHASSISMotor_init(DJI_M3508, WHEEL_BL);
    CHASSISMotor_init(DJI_M3508, WHEEL_BR);

    // PID初始化
    /*PID速度环初始化*/
    PID_Set(&Chassis.chassis_speed_pid_FL, 10.5f, 0.0, 0, CHASSISMOTOR_MAX_CURRENT, 10000);
    PID_Set(&Chassis.chassis_speed_pid_FR, 10.5f, 0.0, 0, CHASSISMOTOR_MAX_CURRENT, 10000);
    PID_Set(&Chassis.chassis_speed_pid_BL, 10.5f, 0.0, 0, CHASSISMOTOR_MAX_CURRENT, 10000);
    PID_Set(&Chassis.chassis_speed_pid_BR, 10.5f, 0.0, 0, CHASSISMOTOR_MAX_CURRENT, 10000);
    /*底盘跟随PID*/
    PID_Set(&Chassis.chassis_follow_pid, 10.0f, 0.0f, 1.0f, 200, 40);
    /*底盘力控PID*/
    PID_Set(&Chassis.chassis_T_pid_x, 40.0f, 0.0, 0, CHASSISMOTOR_MAX_CURRENT, 10000);
    PID_Set(&Chassis.chassis_T_pid_y, 40.0f, 0.0, 0, CHASSISMOTOR_MAX_CURRENT, 10000);
    PID_Set(&Chassis.chassis_T_pid_w, 30.0f, 0.0, 20.0, CHASSISMOTOR_MAX_CURRENT, 10000);
    /*底盘功率控制pid*/
    PID_Set(&Chassis.chassis_power_pid, 0.01f, 0.0f, 0.0f, 0.1f, 0.1f);

    // 底盘运动斜坡
    RampGenerator_Init(&Chassis.Vx_ramp, CHASSIS_TASK_TIME, 40, 40, 2);
    RampGenerator_Init(&Chassis.Vy_ramp, CHASSIS_TASK_TIME, 40, 40, 4);
    RampGenerator_Init(&Chassis.Vw_ramp, CHASSIS_TASK_TIME, 300, 300, 4);

    // 默认底盘不跟随模式
    Global.Chassis.mode = FLOW;
}

/*-------------------- Update --------------------*/

/**
 * @brief          控制量更新（包括状态量和目标量）
 * @param          none
 * @retval         none
 */

void Chassis_Updater()
{

    /*---------------状态量更新----------------*/
    // 轮速(rad/s)
    Chassis.speed_now_FL = CHASSISMotor_get_data(WHEEL_FL).speed_rpm / WHEEL_RATIO * RPM_TO_RAD_S;
    Chassis.speed_now_FR = CHASSISMotor_get_data(WHEEL_FR).speed_rpm / WHEEL_RATIO * RPM_TO_RAD_S;
    Chassis.speed_now_BL = CHASSISMotor_get_data(WHEEL_BL).speed_rpm / WHEEL_RATIO * RPM_TO_RAD_S;
    Chassis.speed_now_BR = CHASSISMotor_get_data(WHEEL_BR).speed_rpm / WHEEL_RATIO * RPM_TO_RAD_S;
    // 角度(rad)
    Chassis.chassis_pitch_angle = IMU_data.AHRS.pitch * RAD_TO_DEG;
    Chassis.chassis_yaw_angle = DMMotor_GetData(YAWMotor).motor_data.para.pos;

    /*---------------目标量更新----------------*/
    // 车速
    RampGenerator_Update(&Chassis.Vx_ramp, Get_SysTime_ms());
    RampGenerator_Update(&Chassis.Vy_ramp, Get_SysTime_ms());
    Chassis.Vx_set = RampGenerator_GetCurrent(&Chassis.Vx_ramp);
    Chassis.Vy_set = RampGenerator_GetCurrent(&Chassis.Vy_ramp);
    Chassis.W_set = Global.Chassis.input.r;
}

/*-------------------- Calculate --------------------*/

/**
 * @brief          控制量解算
 * @param          none
 * @retval         none
 */
#define K (0.0001f)
void Chassis_Calculater()
{
    // 底盘电机前馈力矩定义
    float T_FR, T_FL, T_BR, T_BL; // 转动解算的电机前馈力矩
    float F_FR, F_FL, F_BR, F_BL; // 移动解算的电机前馈力矩
    float Fx, Fy, T_w;            // PID计算的底盘前馈力矩
    // 输出最大电流
    float max_current = 0;

    /*-------速度解算-------*/
    // 目标车速解算
    if (Global.Chassis.mode == FLOW)
    {
        float Vx = Chassis.Vx_set;                                                                                    
        float Vy = Chassis.Vy_set;
        float yaw = Chassis.chassis_yaw_angle; // 已是弧度
        Chassis.Vx_set = Vx * cosf(yaw) - Vy * sinf(yaw);
        Chassis.Vy_set = Vx * sinf(yaw) + Vy * cosf(yaw);
        // yaw 为云台相对底盘的逆时针角度；yaw > 0 时底盘应以 W > 0 跟随。
        Chassis.W_set = -PID_Cal(&Chassis.chassis_follow_pid, yaw, 0.0f);
    }
    else if (Global.Chassis.mode == SPIN_P || Global.Chassis.mode == SPIN_N)
    {
        float Vx = Chassis.Vx_set;
        float Vy = Chassis.Vy_set;
        // 底盘逆时针旋转时，相对 yaw 减小；按反馈角速度补偿延迟。
        float angle = Chassis.chassis_yaw_angle - K * Chassis.W_now;
        Chassis.Vx_set = Vx * cosf(angle) - Vy * sinf(angle);
        Chassis.Vy_set = Vx * sinf(angle) + Vy * cosf(angle);
        // SPIN_P：俯视逆时针；SPIN_N：俯视顺时针。
        Chassis.W_set = (Global.Chassis.mode == SPIN_P) ? (60 * RPM_TO_RAD_S) : (-60 * RPM_TO_RAD_S);
    }
    else if (Global.Chassis.mode == NO_FOLLOW)
    {
        // 移动方向跟随云台视角，但底盘不自转跟随
        float Vx = Chassis.Vx_set;
        float Vy = Chassis.Vy_set;
        float yaw = Chassis.chassis_yaw_angle;
        Chassis.Vx_set = Vx * cosf(yaw) - Vy * sinf(yaw);
        Chassis.Vy_set = Vx * sinf(yaw) + Vy * cosf(yaw);
        Chassis.W_set = 0;
    }

    // 逆解目标轮速
    Chassis.speed_set_FL = RAD_S_TO_RPM * IK_WHEEL_FL(Chassis.Vx_set, Chassis.Vy_set, Chassis.W_set);
    Chassis.speed_set_FR = RAD_S_TO_RPM * IK_WHEEL_FR(Chassis.Vx_set, Chassis.Vy_set, Chassis.W_set);
    Chassis.speed_set_BL = RAD_S_TO_RPM * IK_WHEEL_BL(Chassis.Vx_set, Chassis.Vy_set, Chassis.W_set);
    Chassis.speed_set_BR = RAD_S_TO_RPM * IK_WHEEL_BR(Chassis.Vx_set, Chassis.Vy_set, Chassis.W_set);

    // 正解当前车速
    Chassis.Vx_now = FK_VX(Chassis.speed_now_FL, Chassis.speed_now_FR, Chassis.speed_now_BL, Chassis.speed_now_BR);
    Chassis.Vy_now = FK_VY(Chassis.speed_now_FL, Chassis.speed_now_FR, Chassis.speed_now_BL, Chassis.speed_now_BR);
    Chassis.W_now = FK_OMEGA(Chassis.speed_now_FL, Chassis.speed_now_FR, Chassis.speed_now_BL, Chassis.speed_now_BR);

    /*-------力矩解算-------*/
    Fx = PID_Cal(&Chassis.chassis_T_pid_x, Chassis.Vx_now, Chassis.Vx_set);
    Fy = PID_Cal(&Chassis.chassis_T_pid_y, Chassis.Vy_now, Chassis.Vy_set);
    T_w = PID_Cal(&Chassis.chassis_T_pid_w, Chassis.W_now, Chassis.W_set);
    T_FR = IK_WHEEL_FR_T(0, 0, T_w) / CHASSISMOTOR_T_A * CHASSISMOTOR_MAX_CURRENT / 20.0f;
    T_FL = IK_WHEEL_FL_T(0, 0, T_w) / CHASSISMOTOR_T_A * CHASSISMOTOR_MAX_CURRENT / 20.0f;
    T_BR = IK_WHEEL_BR_T(0, 0, T_w) / CHASSISMOTOR_T_A * CHASSISMOTOR_MAX_CURRENT / 20.0f;
    T_BL = IK_WHEEL_BL_T(0, 0, T_w) / CHASSISMOTOR_T_A * CHASSISMOTOR_MAX_CURRENT / 20.0f;
    F_FR = IK_WHEEL_FR_T(Fx, Fy, 0) / CHASSISMOTOR_T_A * CHASSISMOTOR_MAX_CURRENT / 20.0f;
    F_FL = IK_WHEEL_FL_T(Fx, Fy, 0) / CHASSISMOTOR_T_A * CHASSISMOTOR_MAX_CURRENT / 20.0f;
    F_BR = IK_WHEEL_BR_T(Fx, Fy, 0) / CHASSISMOTOR_T_A * CHASSISMOTOR_MAX_CURRENT / 20.0f;
    F_BL = IK_WHEEL_BL_T(Fx, Fy, 0) / CHASSISMOTOR_T_A * CHASSISMOTOR_MAX_CURRENT / 20.0f;
    //上坡补偿
    //  if (fabsf(Chassis.chassis_pitch_angle) >= 15)
    //  {
    //      T_FR *= 0.5f;
    //      T_FL *= 0.5f;
    //      T_BL *= 2.0f;
    //      T_BR *= 2.0f;
    //      F_FR *= 0.5f;
    //      F_FL *= 0.5f;
    //      F_BR *= 2.0f;
    //      F_BL *= 2.0f;
    //  }

    /*-------电流计算-------*/
    Chassis.current[0] = (T_FL + F_FL) + PID_Cal(&Chassis.chassis_speed_pid_FL, CHASSISMotor_get_data(WHEEL_FL).speed_rpm / WHEEL_RATIO, Chassis.speed_set_FL);
    Chassis.current[1] = (T_FR + F_FR) + PID_Cal(&Chassis.chassis_speed_pid_FR, CHASSISMotor_get_data(WHEEL_FR).speed_rpm / WHEEL_RATIO, Chassis.speed_set_FR);
    Chassis.current[2] = (T_BL + F_BL) + PID_Cal(&Chassis.chassis_speed_pid_BL, CHASSISMotor_get_data(WHEEL_BL).speed_rpm / WHEEL_RATIO, Chassis.speed_set_BL);
    Chassis.current[3] = (T_BR + F_BR) + PID_Cal(&Chassis.chassis_speed_pid_BR, CHASSISMotor_get_data(WHEEL_BR).speed_rpm / WHEEL_RATIO, Chassis.speed_set_BR);
    //统一限幅
    for (int i = 0; i < 4; i++)
    { // 求最大值
        if (fabsf(Chassis.current[i]) > max_current)
            max_current = fabsf(Chassis.current[i]);
    }
    if (max_current > MAX_CURRENT)
        for (int j = 0; j < 4; j++)
        { // 限幅
            Chassis.current[j] *= (MAX_CURRENT / max_current);
        }
}

/*-------------------- Control --------------------*/

/**
 * @brief          电流值设置
 * @param          none
 * @retval         none
 */
void Chassis_Controller()
{
    float Plimit = 1.0f;
    // if (Referee_data.Chassis_Power_Limit == 0)
    //     Plimit = 1.0f;
    // else
    //     Plimit = Chassis_PowerLimit(Chassis.current[0], Chassis.current[1], Chassis.current[2], Chassis.current[3],
    //                                 CHASSISMotor_get_data(WHEEL_FL).speed_rpm,
    //                                 CHASSISMotor_get_data(WHEEL_FR).speed_rpm,
    //                                 CHASSISMotor_get_data(WHEEL_BL).speed_rpm,
    //                                 CHASSISMotor_get_data(WHEEL_BR).speed_rpm,
    //                                 Referee_data.Chassis_Power_Limit);
    if (Global.Control.mode != LOCK)
    {
        CHASSISMotor_set(Plimit * Chassis.current[0], WHEEL_FL);
        CHASSISMotor_set(Plimit * Chassis.current[1], WHEEL_FR);
        CHASSISMotor_set(Plimit * Chassis.current[2], WHEEL_BL);
        CHASSISMotor_set(Plimit * Chassis.current[3], WHEEL_BR);
    }
    else
    {
        CHASSISMotor_set(0, WHEEL_FL);
        CHASSISMotor_set(0, WHEEL_FR);
        CHASSISMotor_set(0, WHEEL_BL);
        CHASSISMotor_set(0, WHEEL_BR);
    }
}

/*-------------------- Task --------------------*/

/**
 * @brief          底盘任务
 * @param          none
 * @retval         none
 */
void Chassis_Tasks(void)
{

    // 底盘数据更新
    Chassis_Updater();
    // 底盘运动解算
    Chassis_Calculater();
    // 底盘电机控制
    Chassis_Controller();
}

/*-------------------- Set --------------------*/

// @brief 设置底盘水平移动速度

void Chassis_SetX(float x)
{
    RampGenerator_SetTarget(&Chassis.Vx_ramp, x);
    if (x * RampGenerator_GetCurrent(&Chassis.Vx_ramp) < 0) // 符号相反
        RampGenerator_SetCurrent(&Chassis.Vx_ramp, 0.0f);
}

// @brief 设置底盘竖直移动速度

void Chassis_SetY(float y)
{
    RampGenerator_SetTarget(&Chassis.Vy_ramp, y);
    if (y * RampGenerator_GetCurrent(&Chassis.Vy_ramp) < 0) // 符号相反
        RampGenerator_SetCurrent(&Chassis.Vy_ramp, 0.0f);
}

// @brief 设置底盘角速度

void Chassis_SetR(float r)
{
    Global.Chassis.input.r = r;
}

// @brief 设置斜坡规划器加速度

void Chassis_SetAccel(float acc)
{
    RampGenerator_SetAccel(&Chassis.Vx_ramp, acc);
    RampGenerator_SetAccel(&Chassis.Vy_ramp, acc);
}

#endif
