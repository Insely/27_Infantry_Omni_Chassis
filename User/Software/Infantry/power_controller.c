/*
 * @Author: Kale
 * @Date: 2025-11-03 00:07:24
 * @LastEditors: Kale
 * @LastEditTime: 2026-05-05
 * @FilePath: \Hero4-2026-Cavalier\User\Software\power_controller.c
 */

/*
  ****************************(C) COPYRIGHT 2026 ADAM****************************
  * @file       power_controller.c/h
  * @brief      通用底盘功率控制器
  * @note       三层架构：物理模型 → 单电机限制 → 底盘策略
  *             Layer 1: 反电动势衰减 + 物理功率预测
  *             Layer 2: 单电机功率限制（平方根非线性 + 方向保护）
  *             Layer 3: 底盘能量管理 + 功率分配
  @verbatim
  ==============================================================================
  功率预测：P = k0*I*ω + k1*ω + k2*I? + k3
  电流衰减：I_eff = I_target × exp(-max(0, ω-ω0) / τ)
  非线性限制：ratio = sqrt(max(0, P_target - P_fixed) / max(ε, P_pred - P_fixed))
  ==============================================================================
  @endverbatim
  ****************************(C) COPYRIGHT 2026 ADAM****************************
*/

#include "power_controller.h"
#include <math.h>
#include "Global_status.h"
#include "pid.h"
#include "referee_system.h"
#include "supercup.h"

/* =================================================================== */
/*  Layer 0: 电机物理参数（按电机类型索引）                              */
/* =================================================================== */

typedef struct
{
    float k0, k1, k2, k3;
} MotorParams;

static const MotorParams PARAMS[] = {
    [MOTOR_6020] = {0.000025f, 0.100f, 1.07e-7f, 1.30f},
    [MOTOR_3508] = {0.000020f, 0.015f, 2.36e-7f, 1.50f},
};

/* =================================================================== */
/*  Layer 0: 反电动势衰减参数                                           */
/* =================================================================== */

#define BEMF_START_RPM 5000.0f // 电流开始衰减的转速
#define BEMF_TAU_RPM 1500.0f   // 衰减时间常数 (RPM)
#define BEMF_MIN_RATIO 0.001f  // 最小电流比例

#define RPM2RAD 0.104719755f

/* =================================================================== */
/*  Layer 1: 反电动势电流预测                                           */
/* =================================================================== */

float Power_Control_PredictCurrent(float target, float speed_rpm, MotorType type)
{
    (void)type;
    float abs_speed = fabsf(speed_rpm);
    float sign = (target >= 0.0f) ? 1.0f : -1.0f;

    // 低速区：电流完全响应
    if (abs_speed <= BEMF_START_RPM)
        return target;

    // 高速区：指数衰减
    float ratio = expf(-(abs_speed - BEMF_START_RPM) / BEMF_TAU_RPM);
    if (ratio < BEMF_MIN_RATIO)
        ratio = BEMF_MIN_RATIO;
    if (ratio > 1.0f)
        ratio = 1.0f;

    return sign * fabsf(target) * ratio;
}

/* =================================================================== */
/*  Layer 1: 物理功率预测                                               */
/* =================================================================== */

float Power_Control_GetPredPower(float target, float speed_rpm, MotorType type)
{
    const MotorParams *p = &PARAMS[type];
    float I_eff = Power_Control_PredictCurrent(target, speed_rpm, type);
    float omega = fabsf(speed_rpm) * RPM2RAD;
    float I_abs = fabsf(I_eff);
    float torque = I_abs * p->k0;

    return torque * omega          // 机械功率
           + p->k1 * omega         // 速度损耗
           + p->k2 * I_abs * I_abs // 电流热损
           + p->k3;                // 静态损耗
}

/* =================================================================== */
/*  Layer 2: 单电机功率限制器                                            */
/* =================================================================== */

#define LIMIT_RATIO_MIN 0.05f     // 限制比例下限
#define LIMIT_RATIO_FLOOR 0.01f   // smooth 下限
#define LIMIT_SIGN_DEADBAND 50.0f // 方向变化死区

/* 定义有名字的结构体类型，消除匿名结构体不兼容问题 */
typedef struct
{
    float smooth;
    float last_current;
    int last_sign;
} LimiterState;

static LimiterState g_limiter[2]; // [0]=6020, [1]=3508

static void limiter_init(int idx)
{
    g_limiter[idx].smooth = 1.0f;
    g_limiter[idx].last_current = 0.0f;
    g_limiter[idx].last_sign = 0;
}

float Power_Control_Limit(float *current, float speed_rpm, float max_power, MotorType type)
{
    if (current == NULL || max_power <= 0.0f)
        return 0.0f;

    const MotorParams *p = &PARAMS[type];
    LimiterState *L = &g_limiter[type]; // 使用有名字的结构体类型

    float original = *current;
    float pred = Power_Control_GetPredPower(original, speed_rpm, type);

    // --- 方向变化检测 ---
    int cur_sign = (original > LIMIT_SIGN_DEADBAND)    ? 1
                   : (original < -LIMIT_SIGN_DEADBAND) ? -1
                                                       : 0;
    int sign_changed = (L->last_sign != 0 && cur_sign != 0 && L->last_sign != cur_sign);
    L->last_current = original;
    L->last_sign = cur_sign;

    float effective_max = max_power;
    if (sign_changed)
    {
        effective_max = fmaxf(0.125f * max_power, 3.0f);
    }

    // --- 恢复 ---
    if (pred <= effective_max)
    {
        L->smooth = L->smooth * 0.7f + 1.0f * 0.3f;
        if (L->smooth > 1.0f)
            L->smooth = 1.0f;
        *current = original * L->smooth;
        return pred;
    }

    // --- 强制恢复（电容充足时） ---
    if (L->smooth < 0.2f && Cap_GetEnergyPercent(cap.remain_vol) >= 0.5f)
    {
        L->smooth *= 1.05f;
        if (L->smooth > 1.0f)
            L->smooth = 1.0f;
        *current = original * L->smooth;
        return pred;
    }

    // --- 限制 ---
    float omega = fabsf(speed_rpm) * RPM2RAD;
    float fixed_loss = p->k3 + p->k1 * omega;
    float variable = pred - fixed_loss;
    float target_var = effective_max - fixed_loss;
    if (target_var < 0.0f)
        target_var = 0.0f;

    float target_ratio = (variable > 0.0f) ? sqrtf(target_var / variable) : 1.0f;
    if (target_ratio > 1.0f)
        target_ratio = 1.0f;
    if (target_ratio < LIMIT_RATIO_MIN)
        target_ratio = LIMIT_RATIO_MIN;

    L->smooth = L->smooth * 0.5f + target_ratio * 0.5f;
    if (L->smooth > 1.0f)
        L->smooth = 1.0f;
    if (L->smooth < LIMIT_RATIO_FLOOR)
        L->smooth = LIMIT_RATIO_FLOOR;

    *current = original * L->smooth;
    return pred;
}

pid_t power_pid;
void Power_Control_Init(void)
{
    limiter_init(0);
    limiter_init(1);
    PID_Set(&power_pid, 0.1f, 0.01f, 0.0f, 0.0f, 10.0f, 10.0f);
}

/* =================================================================== */
/*  Layer 3: 电容能量计算                                               */
/* =================================================================== */

#define CAP_VALUE 6.67f       // 总电容 (F)
#define CAP_VOLTAGE_MAX 24.3f // 满电电压 (V)

float Cap_GetEnergy(float voltage)
{
    return 0.5f * CAP_VALUE * voltage * voltage;
}

float Cap_GetEnergyPercent(float voltage)
{
    float ratio = voltage / CAP_VOLTAGE_MAX;
    return ratio * ratio;
}

/* =================================================================== */
/*  Layer 3: 底盘功率策略                                               */
/* =================================================================== */

#define POWER_LIMIT_ABS_MAX 250.0f
#define POWER_LIMIT_ABS_MIN 5.0f
#define STEER_RATIO 0.8f
#define STEER_MIN_PER_MOTOR 0.5f
#define WHEEL_MIN_TOTAL_RATIO 0.2f
#define WHEEL_MIN_PER_MOTOR 0.5f

static float s_filtered_power;

void Chassis_PowerControl(float *steer_curr, float *steer_spd,
                          float *wheel_curr, float *wheel_spd,
                          float set_max_power)
{
    // 0. 机器人死亡保护：功率上限为0时，清零输出并重置控制器状态
    if (set_max_power <= 0.0f)
    {
        for (int i = 0; i < 4; i++)
        {
            steer_curr[i] = 0.0f;
            wheel_curr[i] = 0.0f;
        }
        // 持续重置，确保复活时从干净状态启动
        limiter_init(0);
        limiter_init(1);
        s_filtered_power = 0.0f;
        PID_Set(&power_pid, 0.1f, 0.01f, 0.0f, 0.0f, 10.0f, 10.0f);
        Supercap_SetPower(0);
        return;
    }

    // 1. 超电功率（防止 uint8_t 下溢）
    Supercap_SetPower((uint8_t)(set_max_power > 2.0f ? set_max_power - 2.0f : 0));

    // 2. 实测功率滤波（首次从零恢复时直接跳到当前值，避免长时间爬升）
    if (s_filtered_power < 1.0f && cap.Chassis_power > 1.0f)
        s_filtered_power = cap.Chassis_power;
    else
        s_filtered_power = s_filtered_power * 0.95f + cap.Chassis_power * 0.05f;

    // 3. 电容能量剩余
    float energy_pct = Cap_GetEnergyPercent(cap.remain_vol);

    // 4. 总功率上限
    float total;

    if (Global.Cap.mode != FULL)
    {
        if (energy_pct > 0.8f)
            total = 0.90f * set_max_power;
        else if (energy_pct > 0.5f)
            total = 0.80f * set_max_power;
        else if (energy_pct > 0.2f)
            total = 0.75f * set_max_power;
        else
            total = set_max_power * 0.5f + PID_Cal(&power_pid, s_filtered_power, set_max_power * 0.5f);
    }
    else
    {
        if (energy_pct > 0.2f)
            total = fminf(1.5f * set_max_power, POWER_LIMIT_ABS_MAX);
        else
            total = set_max_power + PID_Cal(&power_pid, s_filtered_power, set_max_power);
    }

    // 5. 裁判系统保底
    if (Referee_data.Buffer_Energy <= 50 && Referee_data.Buffer_Energy != 0)
        total = fminf(total, 0.5f * set_max_power);

    // 6. 钳位
    if (total > POWER_LIMIT_ABS_MAX)
        total = POWER_LIMIT_ABS_MAX;
    if (total < POWER_LIMIT_ABS_MIN)
        total = POWER_LIMIT_ABS_MIN;

    // 7. 舵电机组
    float steer_pred_total = 0.0f;
    for (int i = 0; i < 4; i++)
        steer_pred_total += Power_Control_GetPredPower(steer_curr[i], steer_spd[i], MOTOR_6020);

    float steer_budget = fminf(steer_pred_total, total * STEER_RATIO);

    for (int i = 0; i < 4; i++)
    {
        float limit = steer_budget * 0.25f;
        if (limit < STEER_MIN_PER_MOTOR)
            limit = STEER_MIN_PER_MOTOR;
        Power_Control_Limit(&steer_curr[i], steer_spd[i], limit, MOTOR_6020);
    }

    // 8. 轮电机组
    float wheel_budget = total - steer_budget;
    if (wheel_budget < total * WHEEL_MIN_TOTAL_RATIO)
        wheel_budget = total * WHEEL_MIN_TOTAL_RATIO;

    for (int i = 0; i < 4; i++)
    {
        float limit = wheel_budget * 0.25f;
        if (limit < WHEEL_MIN_PER_MOTOR)
            limit = WHEEL_MIN_PER_MOTOR;

        Power_Control_Limit(&wheel_curr[i], wheel_spd[i], limit, MOTOR_3508);
    }
}
