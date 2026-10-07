/*
 * @Author: Nas(1319621819@qq.com)
 * @Date: 2026-01-03 07:11:44
 * @LastEditors: Nas(1319621819@qq.com)
 * @LastEditTime: 2026-01-03 07:22:28
 * @FilePath: \Regular_Sentry_Chassis\User\Algorithm\acceleration_models.h
 */

#ifndef ACCELERATION_MODELS_H
#define ACCELERATION_MODELS_H

#include "stm32h7xx_hal.h"

/**
 * @brief 指数逼近函数 (一阶滞后)
 *        最快缩小误差的方式，随着接近目标而减速。
 * @param current 当前值
 * @param target 目标值
 * @param k 响应系数 (0.0 < k <= 1.0). k越大响应越快.
 *          例如 k=0.1 意味着每次调用修正 10% 的误差.
 * @return 新值
 */
float Model_ExponentialUpdate(float current, float target, float k);

/**
 * @brief 幂函数加速
 *        基于幂律加速数值。
 * @param input 输入值 (归一化 0.0 到 1.0)
 * @param power 幂因子 (例如 2.0 为平方，3.0 为立方)
 *              power > 1.0: 启动慢，加速快。
 *              power < 1.0: 启动快，减速。
 * @return 输出值 (0.0 到 1.0)
 */
float Model_PowerMap(float input, float power);

/**
 * @brief Sigmoid (S形) 函数
 *        平滑的加速和减速。
 * @param t 当前时间/进度 (归一化 0.0 到 1.0)
 * @param k 陡峭度 (例如 10.0 为标准 S 形曲线)
 * @return 输出值 (0.0 到 1.0)
 */
float Model_SigmoidMap(float t, float k);

/**
 * @brief 广义 Logistic 函数 (Richards 曲线)
 *        高度可定制的 S 形曲线。
 * @param t 当前时间 (归一化 0.0 到 1.0)
 * @param B 增长率
 * @param v 不对称因子 (v=1 为标准 sigmoid)
 * @return 输出值
 */
float Model_GompertzMap(float t, float B, float v);

/**
 * @brief 灵敏度增强 (下凹曲线)
 *        使小数值迅速变大。
 *        公式: y = x^(1/k)
 * @param input 输入值 (0.0 到 1.0)
 * @param strength 增强强度 (>= 1.0). 
 *                 1.0 = 线性. 
 *                 2.0 = 平方根 (强增强).
 *                 3.0 = 立方根 (极强增强).
 * @return 输出值
 */
float Model_SensitivityBoost(float input, float strength);

#endif // ACCELERATION_MODELS_H
