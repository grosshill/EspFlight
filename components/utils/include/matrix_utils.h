#ifndef MATRIX_UTILS_H
#define MATRIX_UTILS_H
#include "esp_dsp.h"
#define __cplusplus
void func(void);
typedef struct {
    int rows;
    int cols;
    float *data;
} Matrix;

Matrix* create_matrix(int rows, int cols);
void delete_matrix(Matrix *matrix);
void print_matrix(const Matrix *matrix);
#endif