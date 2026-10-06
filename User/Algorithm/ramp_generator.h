#ifndef RAMP_GENERATOR_H
#define RAMP_GENERATOR_H

#ifdef __cplusplus
extern "C"
{
#endif

    /**
     * @brief 斜坡发生器模块
     *
     * 提供带加速度控制和平滑过渡的数值生成功能，支持动态参数调整和限幅保护
     * 典型应用：电机控制、温度调节、运动控制等需要平滑过渡的场景
     */

    typedef struct
    {
        float current_value;            // 当前输出值（单位：用户定义单位）
        float target_value;             // 目标值
        unsigned int interval_ms;       // 控制间隔（单位：毫秒）
        float accel;                    // 加速度（单位：数值/秒?）
        float decel;                    // 减速度（单位：数值/秒?）
        float max_limit;                // 绝对值最大值限制
        unsigned long last_update_time; // 上次更新时间戳（毫秒）
    } RampGenerator;

    /**
     * @brief 初始化斜坡发生器
     * @param ramp 实例指针
     * @param interval_ms 控制间隔（≥1ms）
     * @param accel 加速度（≥0）
     * @param decel 减速度（≥0）
     * @param max_limit 最大绝对值限制
     */
    void RampGenerator_Init(RampGenerator *ramp,
                            unsigned int interval_ms,
                            float accel,
                            float decel,
                            float max_limit);

    /**
     * @brief 设置目标值（允许超出限制范围）
     */
    void RampGenerator_SetTarget(RampGenerator *ramp, float target);

    /**
     * @brief 获取当前输出值（已限幅）
     */
    float RampGenerator_GetCurrent(const RampGenerator *ramp);

    /**
     * @brief 更新发生器状态（需定期调用）
     * @param current_time_ms 单调递增的当前时间戳
     */
    void RampGenerator_Update(RampGenerator *ramp, unsigned long current_time_ms);

    /**
     * @brief 修改控制间隔（立即生效）
     * @param interval_ms 新间隔（1-1000ms）
     */
    void RampGenerator_SetInterval(RampGenerator *ramp, unsigned int interval_ms);

    /**
     * @brief 修改加速度（下次更新生效）
     * @param accel 新加速度（≥0）
     */
    void RampGenerator_SetAccel(RampGenerator *ramp, float accel);

    /**
     * @brief 修改减速度（下次更新生效）
     * @param decel 新减速度（≥0）
     */
    void RampGenerator_SetDecel(RampGenerator *ramp, float decel);

    /**
     * @brief 修改最大值限制（立即生效并钳位当前值）
     */
    void RampGenerator_SetMaxLimit(RampGenerator *ramp, float max_limit);

    /**
     * @brief 强制设置当前值
     * 
     */
    void RampGenerator_SetCurrent(RampGenerator *ramp, float current_value);

#ifdef __cplusplus
}
#endif

#endif // RAMP_GENERATOR_H