/*
 * @Author: Nas(1319621819@qq.com)
 * @Date: 2026-03-06 09:12:26
 * @LastEditors: Nas(1319621819@qq.com)
 * @LastEditTime: 2026-03-06 09:14:05
 * @FilePath: \Regular_Sentry_Chassis\User\Algorithm\RLS.c
 */
#include "RLS.h"

void RLS_Init(RLS_t *rls, float lambda, float delta) {
    rls->lambda = lambda;
    rls->delta = delta;
    
    // 绑定数据缓冲区到矩阵结构
    Matrix_Init(&rls->P, RLS_DIM, RLS_DIM, rls->P_data);
    Matrix_Init(&rls->theta, RLS_DIM, 1, rls->theta_data);
    Matrix_Init(&rls->K, RLS_DIM, 1, rls->K_data);
    
    // 赋初值: P = eye * delta, theta = zeros
    Matrix_Eye(&rls->P, delta);
    Matrix_Scale(&rls->theta, 0.0f, &rls->theta); // 置零
}

float* RLS_Update(RLS_t *rls, float *phi_data, float y) {
    Matrix_t phi, phi_T;
    float phi_T_data[RLS_DIM];
    Matrix_Init(&phi, RLS_DIM, 1, phi_data);
    Matrix_Init(&phi_T, 1, RLS_DIM, phi_T_data);
    Matrix_Transpose(&phi, &phi_T);

    /* --- 中间变量分配 (使用局部栈内存以保证线程安全且无malloc) --- */
    float temp_vec_data[RLS_DIM];
    float temp_scalar_data[1];
    float temp_mat_data[RLS_DIM * RLS_DIM];
    float temp_mat2_data[RLS_DIM * RLS_DIM];
    
    Matrix_t temp_vec, temp_scalar, temp_mat, temp_mat2;
    Matrix_Init(&temp_vec, RLS_DIM, 1, temp_vec_data);
    Matrix_Init(&temp_scalar, 1, 1, temp_scalar_data);
    Matrix_Init(&temp_mat, RLS_DIM, RLS_DIM, temp_mat_data);
    Matrix_Init(&temp_mat2, RLS_DIM, RLS_DIM, temp_mat2_data);

    /* 1. 计算增益矩阵 K = (P * phi) / (lambda + phi^T * P * phi) */
    // temp_vec = P * phi
    Matrix_Mult(&rls->P, &phi, &temp_vec);
    // temp_scalar = phi^T * temp_vec  (即 phi^T * P * phi)
    Matrix_Mult(&phi_T, &temp_vec, &temp_scalar);
    // K = temp_vec / (lambda + temp_scalar)
    float denom = rls->lambda + temp_scalar.data[0];
    Matrix_Scale(&temp_vec, 1.0f / denom, &rls->K);

    /* 2. 计算参数估计 theta = theta + K * (y - phi^T * theta) */
    // temp_scalar = phi^T * theta
    Matrix_Mult(&phi_T, &rls->theta, &temp_scalar);
    // error = y - phi^T * theta
    float error = y - temp_scalar.data[0];
    // temp_vec = K * error
    Matrix_Scale(&rls->K, error, &temp_vec);
    // theta = theta + temp_vec
    Matrix_Add(&rls->theta, &temp_vec, &rls->theta);

    /* 3. 更新协方差矩阵 P = (P - K * phi^T * P) / lambda */
    // temp_mat = K * phi^T
    Matrix_Mult(&rls->K, &phi_T, &temp_mat);
    // temp_mat2 = temp_mat * P
    Matrix_Mult(&temp_mat, &rls->P, &temp_mat2);
    // temp_mat = P - temp_mat2
    Matrix_Sub(&rls->P, &temp_mat2, &temp_mat);
    // P = temp_mat / lambda
    Matrix_Scale(&temp_mat, 1.0f / rls->lambda, &rls->P);

    return rls->theta.data;
}