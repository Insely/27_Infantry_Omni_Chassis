#include "Matrix.h"

void Matrix_Init(Matrix_t *mat, uint16_t rows, uint16_t cols, float *data) {
    mat->rows = rows;
    mat->cols = cols;
    mat->data = data;
}

void Matrix_Add(const Matrix_t *A, const Matrix_t *B, Matrix_t *out) {
    uint16_t size = A->rows * A->cols;
    for (uint16_t i = 0; i < size; i++) {
        out->data[i] = A->data[i] + B->data[i];
    }
}

void Matrix_Sub(const Matrix_t *A, const Matrix_t *B, Matrix_t *out) {
    uint16_t size = A->rows * A->cols;
    for (uint16_t i = 0; i < size; i++) {
        out->data[i] = A->data[i] - B->data[i];
    }
}

void Matrix_Mult(const Matrix_t *A, const Matrix_t *B, Matrix_t *out) {
    for (uint16_t i = 0; i < A->rows; i++) {
        for (uint16_t j = 0; j < B->cols; j++) {
            float sum = 0.0f;
            for (uint16_t k = 0; k < A->cols; k++) {
                sum += A->data[i * A->cols + k] * B->data[k * B->cols + j];
            }
            out->data[i * out->cols + j] = sum;
        }
    }
}

void Matrix_Scale(const Matrix_t *A, float scalar, Matrix_t *out) {
    uint16_t size = A->rows * A->cols;
    for (uint16_t i = 0; i < size; i++) {
        out->data[i] = A->data[i] * scalar;
    }
}

void Matrix_Transpose(const Matrix_t *A, Matrix_t *out) {
    for (uint16_t i = 0; i < A->rows; i++) {
        for (uint16_t j = 0; j < A->cols; j++) {
            out->data[j * A->rows + i] = A->data[i * A->cols + j];
        }
    }
}

void Matrix_Eye(Matrix_t *mat, float scale) {
    uint16_t size = mat->rows * mat->cols;
    for (uint16_t i = 0; i < size; i++) {
        mat->data[i] = 0.0f;
    }
    uint16_t min_dim = (mat->rows < mat->cols) ? mat->rows : mat->cols;
    for (uint16_t i = 0; i < min_dim; i++) {
        mat->data[i * mat->cols + i] = scale;
    }
}