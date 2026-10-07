/*
 * @Author: Nas(1319621819@qq.com)
 * @Date: 2026-01-03 07:11:44
 * @LastEditors: Nas(1319621819@qq.com)
 * @LastEditTime: 2026-01-03 07:22:40
 * @FilePath: \Regular_Sentry_Chassis\User\Algorithm\acceleration_models.c
 */

#include "acceleration_models.h"
#include <math.h>

/**
 * @brief 指数逼近 (一阶滞后)
 *        公式: y_new = y_old + k * (target - y_old)
 */
float Model_ExponentialUpdate(float current, float target, float k) {
    if (k < 0.0f) k = 0.0f;
    if (k > 1.0f) k = 1.0f;
    return current + k * (target - current);
}

/**
 * @brief 幂函数映射
 *        公式: y = x^p
 */
float Model_PowerMap(float input, float power) {
    if (input < 0.0f) input = 0.0f;
    if (input > 1.0f) input = 1.0f;
    return powf(input, power);
}

/**
 * @brief Sigmoid (S形) 函数映射
 *        公式: y = 1 / (1 + e^(-k * (x - 0.5)))
 *        将输入 [0,1] 映射到输出 [0,1] (近似).
 */
float Model_SigmoidMap(float t, float k) {
    // 将 t 平移到以 0 为中心 (-0.5 到 0.5) 以获得对称性
    float x = t - 0.5f;
    // 计算 sigmoid
    float sigmoid = 1.0f / (1.0f + expf(-k * x));
    
    // 归一化，使得 t=0 => 0 且 t=1 => 1
    // S(0) = 1 / (1 + e^(0.5k))
    // S(1) = 1 / (1 + e^(-0.5k))
    float min_val = 1.0f / (1.0f + expf(0.5f * k));
    float max_val = 1.0f / (1.0f + expf(-0.5f * k));
    
    return (sigmoid - min_val) / (max_val - min_val);
}

/**
 * @brief Gompertz (冈珀茨) 函数映射
 *        公式: y = e^(-b * e^(-c * t))
 *        适用于非对称增长.
 */
float Model_GompertzMap(float t, float B, float v) {
    // 简化的类 Gompertz 曲线，用于 0..1
    // y = exp(-B * exp(-v * t))
    // 这通常需要调整 B 和 v 以精确适应 0..1 范围。
    // 这里是一个简化版本：y = x * e^(k(1-x)) ? 不。
    
    // 让我们使用映射到 0..1 的标准形式
    // f(t) = a * exp(-b * exp(-c * t))
    // 假设我们要将 t=[0,1] 映射到 y=[0,1]
    
    // 一个更简单的“快速启动”模型：
    // y = 1 - (1-x)^k
    // 如果 k=2: 1 - (1-x)^2 = 2x - x^2 (反抛物线)
    // 启动快，随后减慢。
    
    if (t < 0.0f) t = 0.0f;
    if (t > 1.0f) t = 1.0f;
    
    // 使用“反向幂”实现“快速启动”
    // 这使得变量在短时间内（周期的早期）变大
    return 1.0f - powf(1.0f - t, B);
}

/**
 * @brief 灵敏度增强
 *        公式: y = x^(1/strength)
 */
float Model_SensitivityBoost(float input, float strength) {
    if (input < 0.0f) input = 0.0f;
    if (input > 1.0f) input = 1.0f;
    if (strength < 1.0f) strength = 1.0f;
    
    return powf(input, 1.0f / strength);
}
