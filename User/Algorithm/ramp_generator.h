#ifndef RAMP_GENERATOR_H
#define RAMP_GENERATOR_H

#include <stdint.h>

#ifdef __cplusplus
extern "C"
{
#endif

    /**
     * @brief 斜坡发生器模块：支持 S 曲线和线性斜坡
     *
     * 提供带 jerk(加加速度) 限制的平滑速度规划，实现真正的"缓加速/缓减速"。
     * 速度曲线为 S 形而非梯形，加速度连续渐变，消除机械冲击。
     * 
     * 默认使用 S 曲线；线性模式按固定加减速度更新，不使用 jerk。
     *
     * 典型应用：舵轮底盘速度规划、云台平滑转动
     */

    typedef enum
    {
        RAMP_MODE_S_CURVE = 0,
        RAMP_MODE_LINEAR
    } RampGeneratorMode;

    typedef struct
    {
        /* 输出状态 */
        float current_value;     // 当前输出值 (位置/速度，用户定义单位)
        float current_accel;     // 当前实际加速度（单位/s²），由 jerk 渐变控制
        
        /* 目标与参数 */
        float target_value;      // 目标值
        float max_accel;         // 最大加速度（单位/s²）
        float max_decel;         // 最大减速度（单位/s²）
        float jerk;              // 加加速度（单位/s³），控制加速度变化快慢
        float max_limit;         // 绝对值上限
        RampGeneratorMode mode;  // 斜坡模式，初始化默认为 S 曲线

        /* 时间管理 */
        unsigned int interval_ms;       // 控制间隔 (ms)，用于 dt 计算
        uint32_t last_update_time;       // 上次更新时间戳 (ms)，支持 32 位计时回绕
    } RampGenerator;

    /**
     * @brief 初始化斜坡发生器
     * @param ramp        实例指针
     * @param interval_ms 控制间隔 (≥1ms)
     * @param accel       最大加速度（≥0，单位/s²）
     * @param decel       最大减速度（≥0，单位/s²）
     * @param max_limit   最大绝对值限制
     * @note  默认 S 曲线，jerk = accel * 20；线性模式需调用 RampGenerator_SetMode
     */
    void RampGenerator_Init(RampGenerator *ramp,
                            unsigned int interval_ms,
                            float accel,
                            float decel,
                            float max_limit);

    /**
     * @brief 设置目标值
     */
    void RampGenerator_SetTarget(RampGenerator *ramp, float target);

    /**
     * @brief 获取当前输出值（已限幅）
     */
    float RampGenerator_GetCurrent(const RampGenerator *ramp);

    /**
     * @brief 更新发生器状态（需定期调用，建议 1ms 周期）
     * @note  内部使用 interval_ms 计算 dt
     */
    void RampGenerator_Update(RampGenerator *ramp);

    /**
     * @brief 按毫秒时间戳更新，未到周期不更新，延迟时补足完整周期
     * @note  首次调用建立时间基准；同一实例不要混用 Update 和 UpdateTimed
     */
    void RampGenerator_UpdateTimed(RampGenerator *ramp, uint32_t now_ms);

    /**
     * @brief 切换斜坡模式，保留当前值和目标值，清零当前加速度
     */
    void RampGenerator_SetMode(RampGenerator *ramp, RampGeneratorMode mode);

    /**
     * @brief 修改控制间隔（立即生效）
     * @param interval_ms 新间隔（1-1000ms）
     */
    void RampGenerator_SetInterval(RampGenerator *ramp, unsigned int interval_ms);

    /**
     * @brief 修改最大加速度（下次更新生效）
     */
    void RampGenerator_SetAccel(RampGenerator *ramp, float accel);

    /**
     * @brief 修改最大减速度（下次更新生效）
     */
    void RampGenerator_SetDecel(RampGenerator *ramp, float decel);

    /**
     * @brief 修改加加速度 jerk（下次更新生效）
     * @param jerk 加加速度（≥0，单位/s³）
     */
    void RampGenerator_SetJerk(RampGenerator *ramp, float jerk);

    /**
     * @brief 修改最大值限制（立即生效并钳位当前值）
     */
    void RampGenerator_SetMaxLimit(RampGenerator *ramp, float max_limit);

    /**
     * @brief 强制设置当前值（同时将当前加速度清零）
     */
    void RampGenerator_SetCurrent(RampGenerator *ramp, float current_value);

    /**
     * @brief 重置斜坡发生器（当前值、目标值、加速度全部归零）
     */
    void RampGenerator_Reset(RampGenerator *ramp);

    /**
     * @brief 检查是否已达到目标值
     * @retval 1: 已达到, 0: 未达到
     */
    uint8_t RampGenerator_IsFinished(const RampGenerator *ramp);

#ifdef __cplusplus
}
#endif

#endif // RAMP_GENERATOR_H
