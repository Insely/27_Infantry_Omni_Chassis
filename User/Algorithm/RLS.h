#ifndef RLS_H
#define RLS_H

#include "Matrix.h"

/* 定义待辨识参数的维度，例如 k1, k2 就是 2 */
#define RLS_DIM 2

typedef struct {
    float lambda;      // 遗忘因子
    float delta;       // 初始协方差对角线元素大小
    
    // 内存数据池（替代 C++ 模板类的数组成员）
    float P_data[RLS_DIM * RLS_DIM];
    float theta_data[RLS_DIM];
    float K_data[RLS_DIM];
    
    // 矩阵对象
    Matrix_t P;        // 转移矩阵 (协方差矩阵)
    Matrix_t theta;    // 参数估计向量
    Matrix_t K;        // 增益向量
} RLS_t;

/* 初始化 RLS 结构体 */
void RLS_Init(RLS_t *rls, float lambda, float delta);

/* RLS 单步更新算法 
 * 参数: phi_data 必须是长度为 RLS_DIM 的数组指针，y 为当前时刻实测值
 * 返回: 辨识出的参数数组指针 (长度为 RLS_DIM)
 */
float* RLS_Update(RLS_t *rls, float *phi_data, float y);

#endif // RLS_H
