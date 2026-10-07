#include "robot_param.h"
#if ROBOT_TYPE == ROBOT_INFANTRY
#include "math.h"

#include "Auto_control.h"
#include "Chassis_helm.h"
#include "Gimbal.h"
#include "Global_status.h"
#include "power_controller.h"

#include "referee_system.h"
#include "supercup.h"
#include "Stm32_time.h"
#include "IMU_updata.h"

#include "User_math.h"
#include "robot_param.h"
#include "ramp_generator.h"

#if BOARD_CHASSIS && (USE_CHASSIS_HELM != 0)

Chassis_t chassis ;
static fp32 deta[4] = {45.0f, 45.0f, 45.0f, 45.0f};
RampGenerator Vx_ramp, Vy_ramp, Vw_ramp;
float X_speed, Y_speed, R_speed;
pid_t chassis_power_pid; // 功率闭环控制pid


/*-------------------- Init --------------------*/

/**
 * @brief          初始化
 * @param          none
 * @retval         none
 */
void Chassis_Init()
{
    //底盘电机初始化
    CHASSISMotor_init(DJI_M3508, WHEEL_MOVE_FL);
    CHASSISMotor_init(DJI_M3508, WHEEL_MOVE_FR);
    CHASSISMotor_init(DJI_M3508, WHEEL_MOVE_BL);
    CHASSISMotor_init(DJI_M3508, WHEEL_MOVE_BR);

    CHASSISMotor_init(DJI_GM6020, WHEEL_TURN_FL);
    CHASSISMotor_init(DJI_GM6020, WHEEL_TURN_FR);
    CHASSISMotor_init(DJI_GM6020, WHEEL_TURN_BL);
    CHASSISMotor_init(DJI_GM6020, WHEEL_TURN_BR);
    /*
    //PID初始化
    /*PID轮向电机速度环初始化*/
    PID_Set(&chassis.forward_FL.chassis_speed_pid_forward_FL, 8000.0f, 0, 0, 0.0f, CHASSISMOTOR_MAX_CURRENT, 1500);
    PID_Set(&chassis.forward_FR.chassis_speed_pid_forward_FR, 8000.0f, 0, 0, 0.0f, CHASSISMOTOR_MAX_CURRENT, 1500);
    PID_Set(&chassis.forward_BL.chassis_speed_pid_forward_BL, 8000.0f, 0, 0, 0.0f, CHASSISMOTOR_MAX_CURRENT, 1500);
    PID_Set(&chassis.forward_BR.chassis_speed_pid_forward_BR, 8000.0f, 0, 0, 0.0f, CHASSISMOTOR_MAX_CURRENT, 1500);
    /*PID舵向电机位置环初始化*/
    PID_Set(&chassis.turn_FL.chassis_location_pid_turn_FL,5.0f, 0.0f, 1.0f, 0.0f, CHASSISMOTOR_MAX_CURRENT, 1500);
    PID_Set(&chassis.turn_FR.chassis_location_pid_turn_FR,5.0f, 0.0f, 1.0f, 0.0f, CHASSISMOTOR_MAX_CURRENT, 1500);
    PID_Set(&chassis.turn_BL.chassis_location_pid_turn_BL,5.0f, 0.0f, 1.0f, 0.0f, CHASSISMOTOR_MAX_CURRENT, 1500);
    PID_Set(&chassis.turn_BR.chassis_location_pid_turn_BR,5.0f, 0.0f, 1.0f, 0.0f, CHASSISMOTOR_MAX_CURRENT, 1500); 
    /*PID舵向电机速度环初始化*/
    PID_Set(&chassis.turn_FL.chassis_speed_pid_turn_FL,3.0f, 0.0f, 0.0f, 0.0f, CHASSISMOTOR_MAX_CURRENT, 1500);
    PID_Set(&chassis.turn_FR.chassis_speed_pid_turn_FR,3.0f, 0.0f, 0.0f, 0.0f, CHASSISMOTOR_MAX_CURRENT, 1500);
    PID_Set(&chassis.turn_BL.chassis_speed_pid_turn_BL,3.0f, 0.0f, 0.0f, 0.0f, CHASSISMOTOR_MAX_CURRENT, 1500);
    PID_Set(&chassis.turn_BR.chassis_speed_pid_turn_BR,3.0f, 0.0f, 0.0f, 0.0f, CHASSISMOTOR_MAX_CURRENT, 1500);

    /*底盘跟随PID*/
    PID_Set(&chassis.chassis_follow_pid, 5.0f, 0.0f, 0.0f, 0.0f, 200, 40);
    /*底盘功率控制pid*/
    PID_Set(&chassis_power_pid, 0.01f, 0.0f, 0.03f, 0.0f, 0.09f, 0.09f);
    //底盘运动斜坡
    /* RampGenerator_Init(&Vx_ramp, CHASSIS_TASK_TIME, 2.0f, 1.5f, 6.0f);
    RampGenerator_Init(&Vy_ramp, CHASSIS_TASK_TIME, 2.0f, 1.5f, 6.0f);
    RampGenerator_Init(&Vw_ramp, CHASSIS_TASK_TIME, 5.0f, 5.0f, 10.0f); 
 */
    //默认地盘跟随模式
    Global.Chassis.mode = FLOW;
    /* 舵向电机参数初始化 */
    chassis.turn_FL.now = 0;
    chassis.turn_FL.set = X_AXIS_ECD_FL;

    chassis.turn_FR.now = 0;
    chassis.turn_FR.set = X_AXIS_ECD_FR;

    chassis.turn_BL.now = 0;
    chassis.turn_BL.set = X_AXIS_ECD_BL;

    chassis.turn_BR.now = 0;
    chassis.turn_BR.set = X_AXIS_ECD_BR;

    /* 旋转速度分解的角度deta */
    for (int i = 0; i < 4; i++)
    {
        deta[i] = deta[i] * PI / 180.0f; // 角度转弧度
    }
    Power_Control_Init();
}




/*-------------------- PowerLimit --------------------*/

/**
 * @brief          功率限制
 * @param          FL_current,FR_current,BL_current,BR_current:四个轮子的电流值
 * @param          FL_current,FR_current,BL_current,BR_current:四个轮子的电流值
 * @retval         percentage：功率限制的百分比
 */
float power_limt(float FL_current, float FR_current, float BL_current, float BR_current,
                 float FL_speed, float FR_speed, float BL_speed, float BR_speed, float max_p)
{
    float current[4] = {FL_current, FR_current, BL_current, BR_current};
    float speed[4] = {FL_speed, FR_speed, BL_speed, BR_speed};
    float now_p = 0.0f;
    
    float a00 = 0.48872161;
    float a01 = -2.93589057e-04;
    float a10 = 5.3241338928e-05;
    float a02 = 2.70936086e-07;
    float a11 = 2.03985936e-06;
    float a20 = 2.17417767e-07;

    //功率限制平滑过渡：等级变化时max_p不直接跳变，而是斜坡渐变
    // static float smooth_max_p = 0.0f;
    // #define POWER_RAMP_RATE 0.02f  // 每ms增加的功率(W)，即每秒20W的爬升速率
    // if (smooth_max_p < 1.0f) // 首次初始化
    //     smooth_max_p = max_p;
    // if (smooth_max_p < max_p)
    // {
    //     smooth_max_p += POWER_RAMP_RATE;
    //     if (smooth_max_p > max_p)
    //         smooth_max_p = max_p;
    // }
    // else
    // {
    //     smooth_max_p = max_p; // 功率降低时立即响应，防止超功率
    // }
    // max_p = smooth_max_p;

    /*最大功率设置*/
    Supercap_SetPower(max_p - 5.0f); //
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
        max_p += cap.remain_vol * 10; // 12
    }
    /*估算当前功率*/
    for (int i = 0; i < 4; i++)
    {
        now_p += fabs(a00 + a01 * speed[1] + a10 * current[i] +
                      a02 * speed[i] * speed[i] +
                      a11 * speed[i] * current[i] +
                      a20 * current[i] * current[i]);
    }
    float percentage = max_p / now_p;
    if (cap.Chassis_power >= (max_p / 3.0f)) // 底盘当前功率过小不使用闭环
        percentage += PID_Cal(&chassis_power_pid, cap.Chassis_power, max_p);
    // if (Global.Chssis.input.y == 0 && Global.Chssis.input.x == 0)//急刹车
    //     percentage = 1;
    // if ((/*(-Global.Chssis.input.x > 0.5 && Vx_now < -25) || (-Global.Chssis.input.x < -0.5 && Vx_now > 25) ||*/
    //     (Global.Chassis.input.y > 0.5 && Vy_now < -25) || (Global.Chassis.input.y < -0.5 && Vy_now > 25)&&Global.Chassis.mode == FLOW))
    //     percentage = 1;

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

/*-----------------------obtain_modulus_normalization-----------------------*/
/**
 * @brief 求取模归化  转动角度控制在-PI----PI
 *
 * @param x
 * @param modulus
 * @return float
 */
float Obtain_Modulus_Normalization(float x, float modulus)
{
    float tmp;
    tmp = fmod(x + modulus / 2.0f, modulus);
    if (tmp < 0.0f)
    {
        tmp += modulus;
    }
    return (tmp - modulus / 2.0f);
}

/*************************Rudder_Angle_calculation****************************/
/**
 * @brief 舵向角度解算
 * 
 * @param x
 * @param y
 * @param w
 */
void Rudder_Angle_Calculation(float x, float y, float w)
{
    // 线速度
    w = w * WHEEL_TRACK;
    int16_t angle_temp[4];

    // 旋转运动
    if (fabs(x) <= 0.01f && fabs(y) <= 0.01f && fabs(w) <= 0.01f && fabs(chassis.forward_FL.current_velocity_FL) <= 0.01f && fabs(chassis.forward_FR.current_velocity_FR) <= 0.01f && fabs(chassis.forward_BL.current_velocity_BL) <= 0.01f && fabs(chassis.forward_BR.current_velocity_BR) <= 0.01f)
    {
        chassis.turn_FL.set_angle_FL = 0.0f;
        chassis.turn_FR.set_angle_FR = 0.0f;
        chassis.turn_BL.set_angle_BL = 0.0f;
        chassis.turn_BR.set_angle_BR = 0.0f;
    }
    else
    {
        chassis.turn_FL.set_angle_FL = atan2((y - w * 0.707107f), (x - w * 0.707107f)) * 180.0f / PI;
        chassis.turn_FR.set_angle_FR = atan2((y - w * 0.707107f), (x + w * 0.707107f)) * 180.0f / PI;
        chassis.turn_BL.set_angle_BL = atan2((y + w * 0.707107f), (x - w * 0.707107f)) * 180.0f / PI;
        chassis.turn_BR.set_angle_BR = atan2((y + w * 0.707107f), (x + w * 0.707107f)) * 180.0f / PI;
    }

    chassis.turn_FL.set_ECD_FL = X_AXIS_ECD_FL + chassis.turn_FL.set_angle_FL * 8192.0 / 360.0;
    chassis.turn_FR.set_ECD_FR = X_AXIS_ECD_FR + chassis.turn_FR.set_angle_FR * 8192.0 / 360.0;
    chassis.turn_BL.set_ECD_BL = X_AXIS_ECD_BL + chassis.turn_BL.set_angle_BL * 8192.0 / 360.0;
    chassis.turn_BR.set_ECD_BR = X_AXIS_ECD_BR + chassis.turn_BR.set_angle_BR * 8192.0 / 360.0;

    // 角度赋值
    //--在这里设置的需要的角度对应在编码器上的位置
    chassis.turn_FL.set = (int32_t)chassis.turn_FL.set_ECD_FL;
    chassis.turn_FR.set = (int32_t)chassis.turn_FR.set_ECD_FR;
    chassis.turn_BL.set = (int32_t)chassis.turn_BL.set_ECD_BL;
    chassis.turn_BR.set = (int32_t)chassis.turn_BR.set_ECD_BR;
}
/*----------------------------Nearby_Transposition----------------------------*/
/**
 * @brief 就近转位
 * 
 */
void Nearby_Transposition()
{
    chassis.turn_FL.target_angle_turn_FL = Obtain_Modulus_Normalization(chassis.turn_FL.set - chassis.turn_FL.now, 8192.0f);
    chassis.turn_FR.target_angle_turn_FR = Obtain_Modulus_Normalization(chassis.turn_FR.set - chassis.turn_FR.now, 8192.0f);
    chassis.turn_BL.target_angle_turn_BL = Obtain_Modulus_Normalization(chassis.turn_BL.set - chassis.turn_BL.now, 8192.0f);
    chassis.turn_BR.target_angle_turn_BR = Obtain_Modulus_Normalization(chassis.turn_BR.set - chassis.turn_BR.now, 8192.0f);
    

    // 根据转动角度范围决定是否需要就近转位 FL
    if (-2048.0f <= chassis.turn_FL.target_angle_turn_FL && chassis.turn_FL.target_angle_turn_FL <= 2048.0f)
    {
        // ±PI / 2之间无需反向就近转位
        chassis.turn_FL.set = chassis.turn_FL.target_angle_turn_FL + chassis.turn_FL.now;
        chassis.forward_FL.opposite_direction_FL = -1.0f;
    }
    else
    {
        // 需要反转扣圈情况
        chassis.turn_FL.set = Obtain_Modulus_Normalization(chassis.turn_FL.target_angle_turn_FL + 4096.0f, 8192.0f) + chassis.turn_FL.now;
        chassis.forward_FL.opposite_direction_FL = 1.0f;
    }

    // 根据转动角度范围决定是否需要就近转位 FR
    if (-2048.0f <= chassis.turn_FR.target_angle_turn_FR && chassis.turn_FR.target_angle_turn_FR <= 2048.0f)
    {
        // ±PI / 2之间无需反向就近转位
        chassis.turn_FR.set = chassis.turn_FR.target_angle_turn_FR + chassis.turn_FR.now;
        chassis.forward_FR.opposite_direction_FR = 1.0f;
    }
    else
    {
        // 需要反转扣圈情况
        chassis.turn_FR.set = Obtain_Modulus_Normalization(chassis.turn_FR.target_angle_turn_FR + 4096.0f, 8192.0f) + chassis.turn_FR.now;
        chassis.forward_FR.opposite_direction_FR = -1.0f;
    }

    // 根据转动角度范围决定是否需要就近转位 BL
    if (-2048.0f <= chassis.turn_BL.target_angle_turn_BL && chassis.turn_BL.target_angle_turn_BL <= 2048.0f)
    {
        // ±PI / 2之间无需反向就近转位
        chassis.turn_BL.set = chassis.turn_BL.target_angle_turn_BL + chassis.turn_BL.now;
        chassis.forward_BL.opposite_direction_BL = -1.0f;
    }
    else
    {
        // 需要反转扣圈情况
        chassis.turn_BL.set = Obtain_Modulus_Normalization(chassis.turn_BL.target_angle_turn_BL + 4096.0f, 8192.0f) + chassis.turn_BL.now;
        chassis.forward_BL.opposite_direction_BL = 1.0f;
    }

    // 根据转动角度范围决定是否需要就近转位 BR
    if (-2048.0f <= chassis.turn_BR.target_angle_turn_BR && chassis.turn_BR.target_angle_turn_BR <= 2048.0f)
    {
        // ±PI / 2之间无需反向就近转位
        chassis.turn_BR.set = chassis.turn_BR.target_angle_turn_BR + chassis.turn_BR.now;
        chassis.forward_BR.opposite_direction_BR = 1.0f;
    }
    else
    {
        // 需要反转扣圈情况
        chassis.turn_BR.set = Obtain_Modulus_Normalization(chassis.turn_BR.target_angle_turn_BR + 4096.0f, 8192.0f) + chassis.turn_BR.now;
        chassis.forward_BR.opposite_direction_BR = -1.0f;
    }
}

/***************************Mode_Switch*****************************/


#define K_w 0.03f
void Mode_Switch()
{
    //float X_speed, Y_speed, R_speed;
    float sin_beta, cos_beta;
    
    // YAW电机零点校准：防止电机重新使能后零点偏移
    static float yaw_pos_offset = 0;
    static uint8_t yaw_offset_inited = 0;
    float raw_pos = DMMotor_GetData(DM_CAN_2_1).motor_data.para.pos;
    
    if (!yaw_offset_inited) {
        // 首次记录偏移：上电时电机位置应为设定的零点
        yaw_pos_offset = raw_pos;
        yaw_offset_inited = 1;
    }
    // 检测突变（电机重新使能导致零点重置）
    float corrected = raw_pos - yaw_pos_offset;
    float prev_angle = chassis.relative_angle;
    if (fabsf(corrected - prev_angle) > 1.0f && fabsf(raw_pos) < 0.1f) {
        // pos突然跳到接近0，说明电机零点被重置了，重新计算偏移
        yaw_pos_offset = raw_pos - prev_angle;
        corrected = prev_angle; // 保持连续
    }
    chassis.relative_angle = corrected;

    switch(Global.Chassis.mode)
    {
        case FLOW:
        //  if(fabs(chassis.relative_angle)>(2.0f * DEG_TO_RAD))
        // {
        //     Global.Chassis.input.r = -PID_Cal(&chassis.chassis_follow_pid, chassis.relative_angle,0);
        // }
        // else   
        // {
        //     Global.Chassis.input.r = 0;
        // }
        // // Global.Chassis.input.r = 0;
        Global.Chassis.input.r = -PID_Cal(&chassis.chassis_follow_pid, chassis.relative_angle,0);
        // float err = chassis.relative_angle;
        // float deadband = 1.0f * DEG_TO_RAD;
        // if (err > deadband) {
        //   err = err - deadband;
        // } else if (err < -deadband) {
        //   err = err + deadband;
        // } else {
        //   err = 0.0f;
        // }
        
        // float r_out = -PID_Cal(&chassis.chassis_follow_pid, err, 0);
        // float limit_speed = 6.0f; 
        // if (r_out > limit_speed)       r_out = limit_speed;
        // else if (r_out < -limit_speed) r_out = -limit_speed;

        // Global.Chassis.input.r = r_out; 
        // chassis.feedforward_angle = 0;
        
        break;
        case SPIN_P:
        Global.Chassis.input.r = 80.0f * RPM_TO_RAD_S;
        chassis.feedforward_angle = Global.Chassis.input.r * K_w;
        break;
        case SPIN_N:
        Global.Chassis.input.r = -80.0f * RPM_TO_RAD_S;
        chassis.feedforward_angle = Global.Chassis.input.r * K_w;
        break;
        case NO_FOLLOW:
        Global.Chassis.input.r = 0;
        chassis.feedforward_angle = 0;
        break;
        default:
        {
            Global.Chassis.input.x = 0;
            Global.Chassis.input.y = 0;
            Global.Chassis.input.r = 0;
            chassis.feedforward_angle = 0;
        }
        break;
    }

    sin_beta = sinf(chassis.relative_angle - chassis.feedforward_angle);
    cos_beta = cosf(chassis.relative_angle - chassis.feedforward_angle);

    X_speed = Global.Chassis.input.x * cos_beta + sin_beta * Global.Chassis.input.y;           
    Y_speed = -Global.Chassis.input.x * sin_beta + Global.Chassis.input.y * cos_beta;
    R_speed = Global.Chassis.input.r;
    Rudder_Angle_Calculation(X_speed, Y_speed, R_speed);
        // 底盘运动解算
    Chassis_Calculater(X_speed, Y_speed, R_speed); 

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
    //--编码器赋值
    chassis.turn_FL.now = CHASSISMotor_get_data(WHEEL_TURN_FL).ecd;
    chassis.turn_FR.now = CHASSISMotor_get_data(WHEEL_TURN_FR).ecd;
    chassis.turn_BL.now = CHASSISMotor_get_data(WHEEL_TURN_BL).ecd;
    chassis.turn_BR.now = CHASSISMotor_get_data(WHEEL_TURN_BR).ecd;
    //--出轴转速rpm赋值
    chassis.forward_FL.current_velocity_FL = CHASSISMotor_get_data(WHEEL_MOVE_FL).round_speed * RPM_TO_RAD_S * WHEEL_RADIUS;
    chassis.forward_FR.current_velocity_FR = CHASSISMotor_get_data(WHEEL_MOVE_FR).round_speed * RPM_TO_RAD_S * WHEEL_RADIUS;
    chassis.forward_BL.current_velocity_BL = CHASSISMotor_get_data(WHEEL_MOVE_BL).round_speed * RPM_TO_RAD_S * WHEEL_RADIUS;
    chassis.forward_BR.current_velocity_BR = CHASSISMotor_get_data(WHEEL_MOVE_BR).round_speed * RPM_TO_RAD_S * WHEEL_RADIUS;
    //--转速RPM赋值
    
    /*---------------目标量更新----------------*/
    //车速

}


/*-------------------- Calculate --------------------*/

/**
 * @brief          控制量解算
 * @param          none
 * @retval         none
 */

void Chassis_Calculater(float vx,float vy,float vw)
{
    Nearby_Transposition();

    chassis.turn_FL.set_turn_FL_speed = PID_Cal(&chassis.turn_FL.chassis_location_pid_turn_FL, chassis.turn_FL.now, chassis.turn_FL.set);
    chassis.turn_FR.set_turn_FR_speed = PID_Cal(&chassis.turn_FR.chassis_location_pid_turn_FR, chassis.turn_FR.now, chassis.turn_FR.set);
    chassis.turn_BL.set_turn_BL_speed = PID_Cal(&chassis.turn_BL.chassis_location_pid_turn_BL, chassis.turn_BL.now, chassis.turn_BL.set);
    chassis.turn_BR.set_turn_BR_speed = PID_Cal(&chassis.turn_BR.chassis_location_pid_turn_BR, chassis.turn_BR.now, chassis.turn_BR.set);
    

    // 计算和速度
    vw = vw * WHEEL_TRACK;
    // 分别计算四个轮子的速度大小
    chassis.forward_FL.V_FL = sqrt((-vx - vw * sinf(deta[1])) * (-vx - vw * sinf(deta[1])) + (-vy + vw * cosf(deta[1])) * (-vy + vw * cosf(deta[1])));
    chassis.forward_FR.V_FR = sqrt((vx + vw * sinf(deta[0])) * (vx + vw * sinf(deta[0])) + (vy + vw * cosf(deta[0])) * (vy + vw * cosf(deta[0])));
    chassis.forward_BL.V_BL = sqrt((vx - vw * sinf(deta[2])) * (vx - vw * sinf(deta[2])) + (vy - vw * cosf(deta[2])) * (vy - vw * cosf(deta[2])));
    chassis.forward_BR.V_BR = sqrt((vx + vw * sinf(deta[3])) * (vx + vw * sinf(deta[3])) + (vy - vw * cosf(deta[3])) * (vy - vw * cosf(deta[3])));

    chassis.turn_FL.current_steer_FL = PID_Cal(&chassis.turn_FL.chassis_speed_pid_turn_FL, 
                                     CHASSISMotor_get_data(WHEEL_TURN_FL).speed_rpm, 
                                     chassis.turn_FL.set_turn_FL_speed);
                                     
    chassis.turn_FR.current_steer_FR = PID_Cal(&chassis.turn_FR.chassis_speed_pid_turn_FR, 
                                     CHASSISMotor_get_data(WHEEL_TURN_FR).speed_rpm, 
                                     chassis.turn_FR.set_turn_FR_speed);
                                     
    chassis.turn_BL.current_steer_BL = PID_Cal(&chassis.turn_BL.chassis_speed_pid_turn_BL, 
                                     CHASSISMotor_get_data(WHEEL_TURN_BL).speed_rpm, 
                                     chassis.turn_BL.set_turn_BL_speed);
                                     
    chassis.turn_BR.current_steer_BR = PID_Cal(&chassis.turn_BR.chassis_speed_pid_turn_BR, 
                                     CHASSISMotor_get_data(WHEEL_TURN_BR).speed_rpm, 
                                     chassis.turn_BR.set_turn_BR_speed);
    // 最大速度限制
/*  val_limit(&vx, chassis.speed.max_x);
    val_limit(&vy, chassis.speed.max_y);
    val_limit(&vw, chassis.speed.max_r); */

    //--确定目标速度的正负
    chassis.forward_FL.target_velocity_FL = chassis.forward_FL.V_FL * chassis.forward_FL.opposite_direction_FL;
    chassis.forward_FR.target_velocity_FR = chassis.forward_FR.V_FR * chassis.forward_FR.opposite_direction_FR;
    chassis.forward_BL.target_velocity_BL = chassis.forward_BL.V_BL * chassis.forward_BL.opposite_direction_BL;
    chassis.forward_BR.target_velocity_BR = chassis.forward_BR.V_BR * chassis.forward_BR.opposite_direction_BR;

    // 计算马达电流
    chassis.forward_FL.wheel_current_FL = PID_Cal(&chassis.forward_FL.chassis_speed_pid_forward_FL,chassis.forward_FL.current_velocity_FL, chassis.forward_FL.target_velocity_FL);
    chassis.forward_FR.wheel_current_FR = PID_Cal(&chassis.forward_FR.chassis_speed_pid_forward_FR,chassis.forward_FR.current_velocity_FR, chassis.forward_FR.target_velocity_FR);
    chassis.forward_BL.wheel_current_BL = PID_Cal(&chassis.forward_BL.chassis_speed_pid_forward_BL,chassis.forward_BL.current_velocity_BL, chassis.forward_BL.target_velocity_BL);
    chassis.forward_BR.wheel_current_BR = PID_Cal(&chassis.forward_BR.chassis_speed_pid_forward_BR,chassis.forward_BR.current_velocity_BR, chassis.forward_BR.target_velocity_BR);

}


/*-------------------- Control --------------------*/
/**
 * @brief          电流值设置
 * @param          none
 * @retval         none
 */
void Chassis_Controller()
{
    
    
    // float Plimit = 0.0f;
    // if (Referee_data.Chassis_Power_Limit == 0)
    //     Plimit = 1.0f;
    // else
    //     Plimit = power_limt(chassis.forward_FL.wheel_current_FL, chassis.forward_FR.wheel_current_FR, chassis.forward_BL.wheel_current_BL, chassis.forward_BR.wheel_current_BR,
    //                         CHASSISMotor_get_data(WHEEL_MOVE_FL).speed_rpm,
    //                         CHASSISMotor_get_data(WHEEL_MOVE_FR).speed_rpm,
    //                         CHASSISMotor_get_data(WHEEL_MOVE_BL).speed_rpm,
    //                         CHASSISMotor_get_data(WHEEL_MOVE_BR).speed_rpm,
    //                         Referee_data.Chassis_Power_Limit);
    
    // CHASSISMotor_set(Plimit * chassis.forward_FL.wheel_current_FL, WHEEL_MOVE_FL);
    // CHASSISMotor_set(Plimit * chassis.forward_FR.wheel_current_FR, WHEEL_MOVE_FR);
    // CHASSISMotor_set(Plimit * chassis.forward_BL.wheel_current_BL, WHEEL_MOVE_BL);
    // CHASSISMotor_set(Plimit * chassis.forward_BR.wheel_current_BR, WHEEL_MOVE_BR);

    // CHASSISMotor_set(chassis.turn_FL.current_steer_FL, WHEEL_TURN_FL);
    // CHASSISMotor_set(chassis.turn_FR.current_steer_FR, WHEEL_TURN_FR);
    // CHASSISMotor_set(chassis.turn_BL.current_steer_BL, WHEEL_TURN_BL);
    // CHASSISMotor_set(chassis.turn_BR.current_steer_BR, WHEEL_TURN_BR);
    
    CHASSISMotor_set(chassis.forward_FL.wheel_current_FL, WHEEL_MOVE_FL);
    CHASSISMotor_set(chassis.forward_FR.wheel_current_FR, WHEEL_MOVE_FR);
    CHASSISMotor_set(chassis.forward_BL.wheel_current_BL, WHEEL_MOVE_BL);
    CHASSISMotor_set(chassis.forward_BR.wheel_current_BR, WHEEL_MOVE_BR); 

    //===== 舵电机 =====
    float steer_curr[4];
    float steer_spd[4];

    steer_curr[0] = (float)chassis.turn_FL.current_steer_FL;
    steer_curr[1] = (float)chassis.turn_FR.current_steer_FR;
    steer_curr[2] = (float)chassis.turn_BL.current_steer_BL;
    steer_curr[3] = (float)chassis.turn_BR.current_steer_BR;

    steer_spd[0] = (float)CHASSISMotor_get_data(WHEEL_TURN_FL).speed_rpm;
    steer_spd[1] = (float)CHASSISMotor_get_data(WHEEL_TURN_FR).speed_rpm;
    steer_spd[2] = (float)CHASSISMotor_get_data(WHEEL_TURN_BL).speed_rpm;
    steer_spd[3] = (float)CHASSISMotor_get_data(WHEEL_TURN_BR).speed_rpm;

    // ===== 轮电机 =====
    float wheel_curr[4];
    float wheel_spd[4];

    wheel_curr[0] = (float)chassis.forward_FL.wheel_current_FL;
    wheel_curr[1] = (float)chassis.forward_FR.wheel_current_FR;
    wheel_curr[2] = (float)chassis.forward_BL.wheel_current_BL;
    wheel_curr[3] = (float)chassis.forward_BR.wheel_current_BR;

    wheel_spd[0] = (float)CHASSISMotor_get_data(WHEEL_MOVE_FL).speed_rpm;
    wheel_spd[1] = (float)CHASSISMotor_get_data(WHEEL_MOVE_FR).speed_rpm;
    wheel_spd[2] = (float)CHASSISMotor_get_data(WHEEL_MOVE_BL).speed_rpm;
    wheel_spd[3] = (float)CHASSISMotor_get_data(WHEEL_MOVE_BR).speed_rpm;

    Chassis_PowerControl(steer_curr, steer_spd, wheel_curr, wheel_spd, Referee_data.Chassis_Power_Limit);

    // ===== 输出 =====
    CHASSISMotor_set((int16_t)steer_curr[0], WHEEL_TURN_FL);
    CHASSISMotor_set((int16_t)steer_curr[1], WHEEL_TURN_FR);
    CHASSISMotor_set((int16_t)steer_curr[2], WHEEL_TURN_BL);
    CHASSISMotor_set((int16_t)steer_curr[3], WHEEL_TURN_BR);

    CHASSISMotor_set((int16_t)wheel_curr[0], WHEEL_MOVE_FL);
    CHASSISMotor_set((int16_t)wheel_curr[1], WHEEL_MOVE_FR);
    CHASSISMotor_set((int16_t)wheel_curr[2], WHEEL_MOVE_BL);
    CHASSISMotor_set((int16_t)wheel_curr[3], WHEEL_MOVE_BR);

    // ===== 调试输出 =====
    //VOFA_UploadData(steer_total_pred, wheel_total_pred, total_power, filtered_power);

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
    Mode_Switch();
	// 底盘电机控制
	Chassis_Controller();
}

/*-------------------- Set --------------------*/

// @brief 设置底盘水平移动速度

void Chassis_SetX(float x)
{
    RampGenerator_SetTarget(&Vx_ramp, x);
    if (x * RampGenerator_GetCurrent(&Vx_ramp) < 0) // 符号相反
        RampGenerator_SetCurrent(&Vx_ramp, 0.0f);
     Global.Chassis.input.x = x; 
}


// @brief 设置底盘竖直移动速度

void Chassis_SetY(float y)
{
    RampGenerator_SetTarget(&Vy_ramp, y);
    if (y * RampGenerator_GetCurrent(&Vy_ramp) < 0) // 符号相反
        RampGenerator_SetCurrent(&Vy_ramp, 0.0f);
     Global.Chassis.input.y = y;
}


// @brief 设置底盘角速度

void Chassis_SetR(float r)
{
    Global.Chassis.input.r = r;
}


// @brief 设置斜坡规划器加速度
 
void Chassis_SetAccel(float acc)
{
   RampGenerator_SetAccel(&Vx_ramp, acc);
   RampGenerator_SetAccel(&Vy_ramp, acc);
}

#endif /* BOARD_CHASSIS && USE_CHASSIS_HELM */


#endif
