#include "ramp_generator.h"
#include <math.h>

/* ================================================================
 * Ramp Generator (S 曲线 / 线性斜坡发生器)
 * ================================================================
 * 核心思路：
 *   传统梯形规划: 加速度阶跃 -> jerk 无穷大 -> 机械冲击
 *   S形规划:      加速度由 jerk 渐变 -> 平滑无冲击
 *
 * 状态变量：
 *   current_value  -- 当前输出值 (位置/速度)
 *   current_accel  -- 当前实际加速度，由 jerk 驱动渐变
 *
 * 每次 Update 的逻辑：
 *   1. 计算"期望加速度"：根据离目标的距离，
 *      决定应该加速、匀速还是制动
 *   2. 用 jerk 限制实际加速度向期望加速度靠拢
 *   3. 用实际加速度更新 current_value
 *   4. 全局限幅
 * ================================================================ */

void RampGenerator_Init(RampGenerator *ramp,
                        unsigned int interval_ms,
                        float accel,
                        float decel,
                        float max_limit)
{
    ramp->current_value = 0.0f;
    ramp->current_accel = 0.0f;
    ramp->target_value = 0.0f;
    ramp->interval_ms = (interval_ms > 0) ? interval_ms : 1;
    ramp->max_accel = fabsf(accel);
    ramp->max_decel = fabsf(decel);
    ramp->jerk = ramp->max_accel * 20.0f;//请注意20是经验默认值，从 0 加速到 max_accel 需要50ms，没有物理推导
    ramp->max_limit = fabsf(max_limit);
    ramp->mode = RAMP_MODE_S_CURVE;
    ramp->last_update_time = 0;
}

void RampGenerator_SetTarget(RampGenerator *ramp, float target)
{
    ramp->target_value = target;
}

float RampGenerator_GetCurrent(const RampGenerator *ramp)
{
    return ramp->current_value;
}

/**
 * @brief S形斜坡核心更新函数
 *
 * 算法步骤：
 * 1. 计算误差 diff = target - current
 * 2. 估算制动距离：从当前加速度减到 0 需要的值变化量
 *    brake_dist = current_accel^2 / (2 * jerk)
 * 3. 决策期望加速度 desired_accel
 * 4. 用 jerk 限制 current_accel 向 desired_accel 靠拢
 * 5. current_value += current_accel * dt
 */
static void RampGenerator_UpdateSCurve(RampGenerator *ramp)
{
    const float dt = (float)ramp->interval_ms * 0.001f;

    float diff = ramp->target_value - ramp->current_value;

    /* ---------- 到达判定 ---------- */
    if (fabsf(diff) < 1e-4f && fabsf(ramp->current_accel) < 1e-4f)
    {
        ramp->current_value = ramp->target_value;
        ramp->current_accel = 0.0f;
        return;
    }

    float sign = (diff > 0.0f) ? 1.0f : -1.0f;

    /* ---------- 计算制动距离 ---------- */
    float abs_accel = fabsf(ramp->current_accel);
    float brake_dist = 0.0f;
    if (ramp->jerk > 1e-6f)
    {
        brake_dist = (abs_accel * abs_accel) / (2.0f * ramp->jerk);//这是前瞻制动距离
                                                                   //——从当前加速度减到 0 的过程中，输出值还会"滑行"多远
                                                                   //即使现在立刻开始制动，输出值还会再走这么远才停下来。
    }

    /* ---------- 决策期望加速度 ---------- */
    float desired_accel;
    float abs_diff = fabsf(diff);

    int accel_towards_target = (ramp->current_accel * diff >= 0.0f);

    if (accel_towards_target && abs_diff <= brake_dist)
    {
        desired_accel = -sign * ramp->max_decel;
    }
    else
    {
        desired_accel = sign * ramp->max_accel;
    }

    /* ---------- Jerk 限制：平滑加速度变化 ---------- */
    float accel_diff = desired_accel - ramp->current_accel;
    float max_accel_change = ramp->jerk * dt;

    if (accel_diff > max_accel_change)
        ramp->current_accel += max_accel_change;
    else if (accel_diff < -max_accel_change)
        ramp->current_accel -= max_accel_change;
    else
        ramp->current_accel = desired_accel;

    /* ---------- 更新输出值 ---------- */
    ramp->current_value += ramp->current_accel * dt;

    /* ---------- 过冲保护 ---------- */
    float new_diff = ramp->target_value - ramp->current_value;
    if (new_diff * diff < 0.0f)
    {
        ramp->current_value = ramp->target_value;
        ramp->current_accel = 0.0f;
    }

    /* ---------- 全局限幅 ---------- */
    if (ramp->current_value > ramp->max_limit)
    {
        ramp->current_value = ramp->max_limit;
        if (ramp->current_accel > 0.0f) ramp->current_accel = 0.0f;
    }
    else if (ramp->current_value < -ramp->max_limit)
    {
        ramp->current_value = -ramp->max_limit;
        if (ramp->current_accel < 0.0f) ramp->current_accel = 0.0f;
    }
}

/* 线性斜坡：按目标绝对值判断加减速，越过目标时停在目标值。 */
static void RampGenerator_UpdateLinear(RampGenerator *ramp)
{
    const float error = ramp->target_value - ramp->current_value;
    float delta = 0.0f;
    if (fabsf(error) > 1e-6f)
    {
        float rate = ramp->max_decel;
        if (fabsf(ramp->target_value) > fabsf(ramp->current_value))
            rate = ramp->max_accel;

        delta = rate * (ramp->interval_ms / 1000.0f);
        if (error < 0.0f)
            delta = -delta;
    }

    float next = ramp->current_value + delta;
    if ((delta > 0.0f && next > ramp->target_value) ||
        (delta < 0.0f && next < ramp->target_value))
        next = ramp->target_value;

    ramp->current_value = fmaxf(fminf(next, ramp->max_limit), -ramp->max_limit);
    ramp->current_accel = 0.0f; // 线性模式不保留 S 曲线的加速度状态
    if (fabsf(ramp->current_value - ramp->target_value) < 1e-6f)
        ramp->current_value = ramp->target_value;
}

void RampGenerator_Update(RampGenerator *ramp)
{
    if (ramp->mode == RAMP_MODE_LINEAR)
        RampGenerator_UpdateLinear(ramp);
    else
        RampGenerator_UpdateSCurve(ramp);
}

void RampGenerator_UpdateTimed(RampGenerator *ramp, uint32_t now_ms)
{
    if (ramp->last_update_time == 0)
    {
        ramp->last_update_time = now_ms;
        return;
    }

    const uint32_t elapsed_ms = now_ms - ramp->last_update_time;
    if (elapsed_ms < ramp->interval_ms)
        return;

    const uint32_t steps = elapsed_ms / ramp->interval_ms;
    ramp->last_update_time += steps * ramp->interval_ms;
    for (uint32_t i = 0; i < steps; i++)
    {
        RampGenerator_Update(ramp);
        if (ramp->current_value == ramp->target_value && ramp->current_accel == 0.0f)
            break;
    }
}

void RampGenerator_SetMode(RampGenerator *ramp, RampGeneratorMode mode)
{
    ramp->mode = mode;
    ramp->current_accel = 0.0f;
}

void RampGenerator_SetInterval(RampGenerator *ramp, unsigned int interval_ms)
{
    ramp->interval_ms = (interval_ms > 0) ? interval_ms : 1;
}

void RampGenerator_SetAccel(RampGenerator *ramp, float accel)
{
    ramp->max_accel = fabsf(accel);
}

void RampGenerator_SetDecel(RampGenerator *ramp, float decel)
{
    ramp->max_decel = fabsf(decel);
}

void RampGenerator_SetJerk(RampGenerator *ramp, float jerk)
{
    ramp->jerk = fabsf(jerk);
}

void RampGenerator_SetMaxLimit(RampGenerator *ramp, float max_limit)
{
    ramp->max_limit = fabsf(max_limit);
    if (ramp->current_value > ramp->max_limit) ramp->current_value = ramp->max_limit;
    if (ramp->current_value < -ramp->max_limit) ramp->current_value = -ramp->max_limit;
}

void RampGenerator_SetCurrent(RampGenerator *ramp, float current_value)
{
    ramp->current_value = current_value;
    ramp->current_accel = 0.0f;
}

void RampGenerator_Reset(RampGenerator *ramp)
{
    ramp->current_value = 0.0f;
    ramp->current_accel = 0.0f;
    ramp->target_value = 0.0f;
    ramp->last_update_time = 0;
}

uint8_t RampGenerator_IsFinished(const RampGenerator *ramp)
{
    return (fabsf(ramp->current_value - ramp->target_value) < 1e-4f &&
            fabsf(ramp->current_accel) < 1e-4f) ? 1 : 0;
}
