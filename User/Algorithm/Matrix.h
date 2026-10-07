#ifndef MATRIX_H
#define MATRIX_H

#include <stdint.h>

/* 矩阵结构体，数据由外部提供以避免 malloc */
typedef struct {
    uint16_t rows;
    uint16_t cols;
    float *data;
} Matrix_t;

/* 矩阵初始化 */
void Matrix_Init(Matrix_t *mat, uint16_t rows, uint16_t cols, float *data);

/* 矩阵基本运算 */
void Matrix_Add(const Matrix_t *A, const Matrix_t *B, Matrix_t *out);
void Matrix_Sub(const Matrix_t *A, const Matrix_t *B, Matrix_t *out);
void Matrix_Mult(const Matrix_t *A, const Matrix_t *B, Matrix_t *out);
void Matrix_Scale(const Matrix_t *A, float scalar, Matrix_t *out);

/* 特殊矩阵生成 */
void Matrix_Transpose(const Matrix_t *A, Matrix_t *out);
void Matrix_Eye(Matrix_t *mat, float scale);

#endif // MATRIX_H