/**
 * @file VMC.h
 * @author Siri (lixirui2017@outlook.com)
 * @brief VMC计算
 * @version 0.1
 * @date 2025-07-20
 *
 * @copyright Copyright (c) 2025
 *
 */
#ifndef __VMC_H__
#define __VMC_H__

// 变量定义，参考(./.doc/.assets/image-202507201333.png)
typedef struct
{
    // 关节角度
    float Phi1;
    float Phi2;
    float Phi3;
    float Phi4;
    // 角速度
    float Phi0_dot;
    float Phi1_dot;
    float Phi4_dot;
    // 角加速度
    float Phi1_ddot;
    float Phi4_ddot;
    // 坐标位置
    float x_D;
    float y_D;
    float x_B;
    float y_B;
    float x_C;
    float y_C;
    //坐标速度
    float x_C_dot;
    float y_C_dot;
    // 距离
    float BD;
    //杆长度变量
    float L1;
    float L2;
    float L3;
    float L4;
    float L5;
    // 中间变量
    float A0;
    float B0;
    float C0;
    // 腿长与角度
    float Phi0;
    float L0;
    float L0_dot;
    //电机力矩
    float T1;
    float T2;
    //腿部力矩
    float F;
    float Tp;
} VMC_s;

extern void VMC_Init(VMC_s *VMC, float L1, float L2, float L3, float L4, float L5);
extern void VMC_Updata(VMC_s *VMC, float Phi1, float Phi4, float Phi1_dot, float Phi4_dot, float Phi1_ddot, float Phi4_ddot);
extern void VMC_Cal(VMC_s *VMC);


#endif // !__VMC_H__
