/*
 * @Date: 2025-08-31 21:36:57
 * @LastEditors: hao && (hao@qlu.edu.cn)
 * @LastEditTime: 2025-10-30 20:04:44
 * @FilePath: \Season-26-Code\User\Algorithm\VMC.c
 */
#include "VMC.h"
#include "math.h"

#define DOUBLE_L2(a) ((a->L2) * 2)
#define HALF_L5(a) ((a->L5) / 2)
#define SQUARE_L2(a) ((a->L2) * (VMC->L2))
#define SQUARE_L3(a) ((a->L3) * (VMC->L3))

void VMC_Init(VMC_s *VMC, float L1, float L2, float L3, float L4, float L5)
{
    VMC->L1 = L1;
    VMC->L2 = L2;
    VMC->L3 = L3;
    VMC->L4 = L4;
    VMC->L5 = L5;
}
/**
 * @brief VMC数据更新
 *
 * @param VMC VMC指针
 * @param Phi1
 * @param Phi4
 * @param Phi1_dot
 * @param Phi4_dot
 * @param Phi1_ddot
 * @param Phi4_ddot
 */
void VMC_Updata(VMC_s *VMC, float Phi1, float Phi4, float Phi1_dot, float Phi4_dot, float Phi1_ddot, float Phi4_ddot)
{
    VMC->Phi1 = Phi1;
    VMC->Phi4 = Phi4;
    VMC->Phi1_dot = Phi1_dot;
    VMC->Phi4_dot = Phi4_dot;
    VMC->Phi1_ddot = Phi1_ddot;
    VMC->Phi4_ddot = Phi4_ddot;
}

/**
 * @brief VMC计算，更新计算数据
 *
 * @param VMC VMC指针
 */
void VMC_Cal(VMC_s *VMC)
{
    float sin_Phi3_Phi2, sin_Phi1_Phi2, sin_Phi3_Phi4; // 储存临时数据

    // B,D坐标解算 BD长度计算
    VMC->x_B = VMC->L1 * cosf(VMC->Phi1);
    VMC->y_B = VMC->L1 * sinf(VMC->Phi1);
    VMC->x_D = VMC->L5 + VMC->L4 * cosf(VMC->Phi4);
    VMC->y_D = VMC->L4 * sinf(VMC->Phi4);

    VMC->BD = powf(((VMC->x_D - VMC->x_B) * (VMC->x_D - VMC->x_B) + (VMC->y_D - VMC->y_B) * (VMC->y_D - VMC->y_B)), 0.5f);

    // A0,B0,C0和Phi2计算
    VMC->A0 = DOUBLE_L2(VMC) * (VMC->x_D - VMC->x_B);
    VMC->B0 = DOUBLE_L2(VMC) * (VMC->y_D - VMC->y_B);
    VMC->C0 = SQUARE_L2(VMC) + VMC->BD * VMC->BD - SQUARE_L3(VMC);

    VMC->Phi2 = 2 * atan2f((VMC->B0 + powf((VMC->A0 * VMC->A0 + VMC->B0 * VMC->B0 - VMC->C0 * VMC->C0), 0.5)), (VMC->A0 + VMC->C0));

    // C点直角和极坐标计算
    VMC->x_C = VMC->x_B + VMC->L2 * cosf(VMC->Phi2);
    VMC->y_C = VMC->y_B + VMC->L2 * sinf(VMC->Phi2);

    VMC->L0 = powf(((VMC->x_C - HALF_L5(VMC)) * (VMC->x_C - HALF_L5(VMC)) + VMC->y_C * VMC->y_C), 0.5);
    VMC->Phi0 = atan2(VMC->y_C, (VMC->x_C - HALF_L5(VMC)));

    // Phi3计算
    VMC->Phi3 = atan2(VMC->y_C - VMC->y_D, VMC->x_C - VMC->x_D);

    // 临时记录数据计算
    sin_Phi3_Phi2 = sinf(VMC->Phi3 - VMC->Phi2);
    sin_Phi1_Phi2 = sinf(VMC->Phi1 - VMC->Phi2);
    sin_Phi3_Phi4 = sinf(VMC->Phi3 - VMC->Phi4);

    // 记录x_C_dot和y_C_dot，计算Phi0_dot
    VMC->x_C_dot = VMC->L1 * sin_Phi1_Phi2 * sinf(VMC->Phi3) / -sin_Phi3_Phi2 * VMC->Phi1_dot +
                   VMC->L4 * sin_Phi3_Phi4 * sinf(VMC->Phi2) / -sin_Phi3_Phi2 * VMC->Phi4_dot;

    VMC->y_C_dot = VMC->L1 * sin_Phi1_Phi2 * cosf(VMC->Phi3) / sin_Phi3_Phi2 * VMC->Phi1_dot +
                   VMC->L4 * sin_Phi3_Phi4 * cosf(VMC->Phi2) / sin_Phi3_Phi2 * VMC->Phi4_dot;

    VMC->Phi0_dot = (VMC->y_C_dot * cosf(VMC->Phi0) - VMC->x_C_dot * sinf(VMC->Phi0)) / VMC->L0;
    VMC->L0_dot = VMC->x_C_dot * cosf(VMC->Phi0) + VMC->y_C_dot * sinf(VMC->Phi0);

    // T1,T2解算
    VMC->T1 = VMC->F * VMC->L1 * sinf(VMC->Phi0 - VMC->Phi3) * sin_Phi1_Phi2 / sin_Phi3_Phi2 +
              VMC->Tp * VMC->L1 * cosf(VMC->Phi0 - VMC->Phi3) * sin_Phi1_Phi2 / (VMC->L0 * sin_Phi3_Phi2);

    VMC->T2 = VMC->F * VMC->L4 * sinf(VMC->Phi0 - VMC->Phi2) * sin_Phi3_Phi4 / sin_Phi3_Phi2 +
              VMC->Tp * VMC->L4 * cosf(VMC->Phi0 - VMC->Phi2) * sin_Phi3_Phi4 / (VMC->L0 * sin_Phi3_Phi2);
}