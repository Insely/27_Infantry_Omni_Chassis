#include "ramp_generator.h"
#include <math.h>

void RampGenerator_Init(RampGenerator *ramp,
                        unsigned int interval_ms,
                        float accel,
                        float decel,
                        float max_limit)
{
    ramp->current_value = 0.0f;
    ramp->target_value = 0.0f;
    ramp->interval_ms = (interval_ms > 0) ? interval_ms : 1;
    ramp->accel = fmaxf(accel, 0.0f);
    ramp->decel = fmaxf(decel, 0.0f);
    ramp->max_limit = fabsf(max_limit);
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

void RampGenerator_Update(RampGenerator *ramp, unsigned long current_time_ms)
{
    // 如果是第一次调用，初始化 last_update_time 并返回
    if (ramp->last_update_time == 0)
    {
        ramp->last_update_time = current_time_ms;
        return;
    }

    // 计算自上次更新以来的时间间隔
    unsigned long elapsed = current_time_ms - ramp->last_update_time;
    if (elapsed < ramp->interval_ms)
    {
        return;
    }

    unsigned int num_steps = elapsed / ramp->interval_ms;
    ramp->last_update_time += num_steps * ramp->interval_ms;

    for (unsigned int i = 0; i < num_steps; ++i)
    {
        float delta = 0.0f;
        const float time_step = ramp->interval_ms / 1000.0f;
        const float error = ramp->target_value - ramp->current_value;

        if (fabsf(error) > 1e-6f)
        {
            const int is_positive_direction = (error > 0);

            const int is_increasing_magnitude =
                (is_positive_direction && (fabsf(ramp->target_value) > fabsf(ramp->current_value))) ||
                (!is_positive_direction && (fabsf(ramp->target_value) > fabsf(ramp->current_value)));

            const float rate = is_increasing_magnitude ? ramp->accel : ramp->decel;
            delta = rate * time_step * (is_positive_direction ? 1.0f : -1.0f);
        }

        // 防止过冲
        float new_value = ramp->current_value + delta;
        if ((delta > 0 && new_value > ramp->target_value) ||
            (delta < 0 && new_value < ramp->target_value))
        {
            new_value = ramp->target_value;
        }

        // 应用限幅
        new_value = fminf(new_value, ramp->max_limit);
        new_value = fmaxf(new_value, -ramp->max_limit);

        ramp->current_value = new_value;

        // 精度处理
        if (fabsf(ramp->current_value - ramp->target_value) < 1e-6f)
        {
            ramp->current_value = ramp->target_value;
            break;
        }
    }
}

void RampGenerator_SetInterval(RampGenerator *ramp, unsigned int interval_ms)
{
    ramp->interval_ms = (interval_ms > 0) ? interval_ms : 1;
}

void RampGenerator_SetAccel(RampGenerator *ramp, float accel)
{
    ramp->accel = fmaxf(accel, 0.0f);
}

void RampGenerator_SetDecel(RampGenerator *ramp, float decel)
{
    ramp->decel = fmaxf(decel, 0.0f);
}

void RampGenerator_SetMaxLimit(RampGenerator *ramp, float max_limit)
{
    ramp->max_limit = fabsf(max_limit);
    ramp->current_value = fminf(ramp->current_value, ramp->max_limit);
    ramp->current_value = fmaxf(ramp->current_value, -ramp->max_limit);
}

void RampGenerator_SetCurrent(RampGenerator *ramp, float current_value)
{
    ramp->current_value = current_value;
}