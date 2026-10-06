#ifndef POWER_CONTROLLER_H
#define POWER_CONTROLLER_H

#ifdef __cplusplus
extern "C"
{
#endif

    typedef enum
    {
        MOTOR_6020 = 0,
        MOTOR_3508 = 1
    } MotorType;

    // 初始化
    void Power_Control_Init(void);

    // 预测
    float Power_Control_PredictCurrent(float target, float speed_rpm, MotorType type);
    float Power_Control_GetPredPower(float target, float speed_rpm, MotorType type);

    // 限制
    float Power_Control_Limit(float *current, float speed_rpm, float max_power, MotorType type);

    // 底盘策略
    void Chassis_PowerControl(float *steer_curr, float *steer_spd,
                              float *wheel_curr, float *wheel_spd,
                              float set_max_power);

    // 电容
    float Cap_GetEnergy(float voltage);
    float Cap_GetEnergyPercent(float voltage);

#ifdef __cplusplus
}
#endif

#endif