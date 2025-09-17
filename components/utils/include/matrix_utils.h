#pragma once
#include <stdbool.h>
#include <stdint.h>
#include <stddef.h>

/* opaque handle of matrices*/
typedef struct mat_handle* mat_t;

// /* matrix data type */
// enum mat_data_type {
//     mat_float,
//     mat_double,
//     mat_int32
// };

/* create a matrix */
mat_t mat(size_t rows, size_t cols);

/* create a matrix with all elements 0 */
mat_t mat_zeros(size_t rows, size_t cols);

/* create an identity matirx */
mat_t mat_eye(size_t dim);

/* create a matrix from given array */
mat_t mat_array(const void* data, size_t rows, size_t cols);

/* release memory */
void mat_free(mat_t mat);

/* get the size of rows */
size_t mat_rows(const mat_t mat);

/* get the size of columns */
size_t mat_cols(const mat_t mat);

/* get/set value of matrix with certain data type */
bool mat_get_float(const mat_t mat, size_t row, size_t col, float* value);
bool mat_set_float(mat_t mat, size_t row, size_t col, float value);
// bool mat_get_double(const mat_t mat, size_t row, size_t col, double* value);
// bool mat_set_double(mat_t mat, size_t row, size_t col, double value);
// bool mat_get_int32(const mat_t mat, size_t row, size_t col, int32_t* value);
// bool mat_set_int32(mat_t mat, size_t row, size_t col, int32_t* value);

/* copy a matrix to another one */
bool mat_copy(const mat_t src, mat_t dest);

/* transpose */
bool mat_trans(mat_t result, const mat_t mat);

/* inverse */
bool mat_inv(mat_t result, const mat_t mat);

/* determinant */
bool mat_det(const mat_t mat, float* det);

/* basic operation */
bool mat_add(mat_t ret, const mat_t mat_l, const mat_t mat_r);
bool mat_sub(mat_t ret, const mat_t mat_l, const mat_t mat_r);
bool mat_mul(mat_t ret, const mat_t mat_l, const mat_t mat_r);
bool mat_sca_mul(mat_t ret, const mat_t mat, float scalar);
bool mat_sca_div(mat_t ret, const mat_t mat, float scalar);

/* block get/set operation */
bool mat_get_block(mat_t block, const mat_t mat, size_t start_row, size_t start_col, size_t rows, size_t cols);
bool mat_set_block(const mat_t src, mat_t dest, size_t start_row, size_t start_col);

/* compute eigen values */
bool mat_eigen(const mat_t mat, mat_t eigenvalues);

/* implement Singular Value Decomposition*/
bool mat_svd(const mat_t mat, mat_t U, mat_t S, mat_t V);

/* print a matrix */
bool mat_print(const mat_t mat);