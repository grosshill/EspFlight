#pragma once
#include "quaternion_utils.h"

#ifdef __cplusplus
extern "C" {
#endif

/* ========== 不透明指针类型定义 (Opaque Handle Types) ========== */
typedef void* mat3f_t;  // 单精度 3x3 矩阵句柄
typedef void* mat4f_t;  // 单精度 4x4 矩阵句柄
typedef void* vec3f_t;  // 单精度 3维向量句柄
typedef void* vec4f_t;  // 单精度 4维向量句柄

/* ========== 通用销毁函数 (必须配对使用) ========== */
void mat3f_free(mat3f_t mat);
void mat4f_free(mat4f_t mat);
void vec3f_free(vec3f_t vec);
void vec4f_free(vec4f_t vec);

/* ========== 3x3 矩阵操作 ========== */
// 构造函数
mat3f_t mat3f_eye(void);                                 // 创建单位矩阵
mat3f_t mat3f_from_array(const float data[9]);           // 从列优先数组创建
mat3f_t mat3f_from_quaternion(const quatf_t q);         // 从四元数创建旋转矩阵 (注意：quat为void*以降低耦合)

// 运算函数 (返回新对象)
mat3f_t mat3f_add(const mat3f_t a, const mat3f_t b);
mat3f_t mat3f_sub(const mat3f_t a, const mat3f_t b);
mat3f_t mat3f_mul_mat(const mat3f_t a, const mat3f_t b);  // 矩阵乘法
vec3f_t mat3f_mul_vec(const mat3f_t mat, const vec3f_t vec); // 矩阵乘以向量
mat3f_t mat3f_mul_scalar(const mat3f_t mat, const float scalar);

// 原地运算函数 (明确以 `_inplace` 后缀标识)
void mat3f_add_inplace(mat3f_t target, const mat3f_t a, const mat3f_t b);
void mat3f_mul_mat_inplace(mat3f_t target, const mat3f_t a, const mat3f_t b);

// 属性与访问
void mat3f_to_array(const mat3f_t mat, float output[9]); // 复制数据到列优先数组
float mat3f_get(const mat3f_t mat, const int row, const int col);     // 获取元素 (0-based索引)
void mat3f_set(mat3f_t mat, const int row, const int col, const float value); // 设置元素

// 特殊操作
mat3f_t mat3f_transpose(const mat3f_t mat);               // 转置 (返回新矩阵)
void mat3f_transpose_inplace(mat3f_t mat);                // 原地转置
mat3f_t mat3f_inverse(const mat3f_t mat);                 // 逆矩阵 (假设可逆)
vec3f_t mat3f_solve(const mat3f_t A, const vec3f_t b);

/* ========== 4x4 矩阵操作 (接口与3x3类似，保持一致性) ========== */
mat4f_t mat4f_eye(void);
mat4f_t mat4f_from_array(const float data[16]);
mat4f_t mat4f_mul_mat(const mat4f_t a, const mat4f_t b);
vec4f_t mat4f_mul_vec(const mat4f_t mat, const vec4f_t vec);
void mat4f_to_array(const mat4f_t mat, float output[16]);
// ... 其他类似mat3f的函数，此处省略以保持简洁
// 运算函数 (返回新对象)
mat4f_t mat4f_add(const mat4f_t a, const mat4f_t b);
mat4f_t mat4f_sub(const mat4f_t a, const mat4f_t b);
mat4f_t mat4f_mul_mat(const mat4f_t a, const mat4f_t b);  // 矩阵乘法
vec4f_t mat4f_mul_vec(const mat4f_t mat, const vec4f_t vec); // 矩阵乘以向量
mat4f_t mat4f_mul_scalar(const mat4f_t mat, const float scalar);

// 原地运算函数 (明确以 `_inplace` 后缀标识)
void mat4f_add_inplace(mat4f_t target, const mat4f_t a, const mat4f_t b);
void mat4f_mul_mat_inplace(mat4f_t target, const mat4f_t a, const mat4f_t b);
// 属性与访问
void mat4f_to_array(const mat4f_t mat, float output[16]); // 复制数据到列优先数组
float mat4f_get(const mat4f_t mat, const int row, const int col);     // 获取元素 (0-based索引)
void mat4f_set(mat4f_t mat, const int row, const int col, const float value); // 设置元素

// 特殊操作
mat4f_t mat4f_transpose(const mat4f_t mat);               // 转置 (返回新矩阵)
void mat4f_transpose_inplace(mat4f_t mat);                // 原地转置
mat4f_t mat4f_inverse(const mat4f_t mat);                 // 逆矩阵 (假设可逆)
vec4f_t mat4f_solve(const mat4f_t A, const vec4f_t b);
mat4f_t mat4f_homo(const mat3f_t R, const vec3f_t t);
mat3f_t mat4f_homo_get_R(const mat4f_t H);
vec3f_t mat4f_homo_get_t(const mat4f_t H);
mat4f_t mat4f_homo_inv(const mat4f_t H);

/* ========== 3维向量操作 ========== */
// 构造函数
vec3f_t vec3f_zeros(void);                               // 零向量
vec3f_t vec3f_from_array(const float data[3]);           // 从数组创建
vec3f_t vec3f_from_xyz(const float x, const float y, const float z);

// 运算
vec3f_t vec3f_add(const vec3f_t a, const vec3f_t b);
vec3f_t vec3f_sub(const vec3f_t a, const vec3f_t b);
vec3f_t vec3f_cross(const vec3f_t a, const vec3f_t b);    // 叉积
float   vec3f_dot(const vec3f_t a, const vec3f_t b);      // 点积 (返回标量)

// 标量运算
vec3f_t vec3f_scale(const vec3f_t vec, const float scalar);     // 数乘
void    vec3f_scale_inplace(vec3f_t vec, const float scalar);

// 属性与访问
void    vec3f_to_array(const vec3f_t vec, float output[3]);
void    vec3f_get(const vec3f_t vec, float* x, float* y, float* z);
void    vec3f_set(vec3f_t vec, const float x, const float y, const float z);
float   vec3f_norm(const vec3f_t vec);                    // 向量模长
void    vec3f_normalize_inplace(vec3f_t vec);             // 原地归一化
vec3f_t vec3f_normalized(const vec3f_t vec);              // 返回归一化副本

/* ========== 4维向量操作 ========== */
// (接口与vec3f类似，可根据需要添加齐次坐标操作)
vec4f_t vec4f_zeros(void);
vec4f_t vec4f_from_array(const float data[4]);
// ... 其他函数
vec4f_t vec4f_from_xyzt(const float x, const float y, const float z, const float t);

// 运算
vec4f_t vec4f_add(const vec4f_t a, const vec4f_t b);
vec4f_t vec4f_sub(const vec4f_t a, const vec4f_t b);
float   vec4f_dot(const vec4f_t a, const vec4f_t b);      // 点积 (返回标量)

// 标量运算
vec4f_t vec4f_scale(const vec4f_t vec, const float scalar);     // 数乘
void    vec4f_scale_inplace(vec4f_t vec, const float scalar);

// 属性与访问
void    vec4f_to_array(const vec4f_t vec, float output[4]);
void    vec4f_get(const vec4f_t vec, float* x, float* y, float* z, float* t);
void    vec4f_set(vec4f_t vec, const float x, const float y, const float z, const float t);
float   vec4f_norm(const vec4f_t vec);                    // 向量模长
void    vec4f_normalize_inplace(vec4f_t vec);             // 原地归一化
vec4f_t vec4f_normalized(const vec4f_t vec);              // 返回归一化副本

#ifdef __cplusplus
}
#endif