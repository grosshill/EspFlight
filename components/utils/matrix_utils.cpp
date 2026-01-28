#include "eigen_types.hpp"
#include "src/Core/Matrix.h"
#include <cstddef>
#include "matrix_utils.h"

extern "C" {
    
void mat3f_free(mat3f_t mat)
{
    if (mat)
    {
        delete static_cast<mat3f_handle*>(mat);
    }
}

void mat4f_free(mat4f_t mat)
{
    if (mat)
    {
        delete static_cast<mat4f_handle*>(mat);
    }
}

void vec3f_free(vec3f_t vec)
{
    if (vec)
    {
        delete static_cast<vec3f_handle*>(vec);
    }
}

void vec4f_free(vec4f_t vec)
{
    if (vec)
    {
        delete static_cast<vec4f_handle*>(vec);
    }
}

mat3f_t mat3f_cpy(mat3f_t mat)
{
    if (mat)
    {
        mat3f_t ret = new mat3f_handle(static_cast<mat3f_handle*>(mat)->mat);
        return ret;
    }
    else
    {
        return nullptr;
    }
}

mat3f_t mat3f_eye(void)
{
    mat3f_t ret = new mat3f_handle();
    return ret;
}

mat3f_t mat3f_from_array(const float data[9])
{
    mat3f_t ret = new mat3f_handle(data);
    return ret; 
}

mat3f_t mat3f_from_quaternion(const quatf_t q)
{
    if (q)
    {
        quatf_handle* qh = static_cast<quatf_handle*>(q);
        mat3f_t ret = new mat3f_handle(qh->quat.toRotationMatrix());
        return ret;
    }
    else
    {
        return nullptr;
    }
}

mat3f_t mat3f_add(const mat3f_t a, const mat3f_t b)
{
    if (a && b)
    {   
        mat3f_handle* ah = static_cast<mat3f_handle*>(a);
        mat3f_handle* bh = static_cast<mat3f_handle*>(b);
        mat3f_t ret = new mat3f_handle(ah->mat + bh->mat);
        return ret;
    }
    else
    {
        return nullptr;
    }
}

mat3f_t mat3f_sub(const mat3f_t a, const mat3f_t b)
{
    if (a && b)
    {   
        mat3f_handle* ah = static_cast<mat3f_handle*>(a);
        mat3f_handle* bh = static_cast<mat3f_handle*>(b);
        mat3f_t ret = new mat3f_handle(ah->mat - bh->mat);
        return ret;
    }
    else
    {
        return nullptr;
    }
}

mat3f_t mat3f_mul_mat(const mat3f_t a, const mat3f_t b)
{
    if (a && b)
    {   
        mat3f_handle* ah = static_cast<mat3f_handle*>(a);
        mat3f_handle* bh = static_cast<mat3f_handle*>(b);
        mat3f_t ret = new mat3f_handle(ah->mat * bh->mat);
        return ret;
    }
    else
    {
        return nullptr;
    }
}

vec3f_t mat3f_mul_vec(const mat3f_t mat, const vec3f_t vec)
{
    if (mat && vec)
    {
        mat3f_handle* mh = static_cast<mat3f_handle*>(mat);
        vec3f_handle* vh = static_cast<vec3f_handle*>(vec);
        vec3f_t ret = new vec3f_handle(mh->mat * vh->vec);
        return ret;
    }
    else 
    {
        return nullptr;
    }
}

mat3f_t mat3f_mul_scalar(const mat3f_t mat, const float scalar)
{
    if (mat)
    {
        mat3f_handle* mh = static_cast<mat3f_handle*>(mat);
        mat3f_t ret = new mat3f_handle(mh->mat * scalar);
        return ret;
    }
    else 
    {
        return nullptr;
    }
}

void mat3f_add_inplace(mat3f_t target, const mat3f_t a, const mat3f_t b)
{
    if (target && a && b)
    {
            mat3f_handle* ah = static_cast<mat3f_handle*>(a);
            mat3f_handle* bh = static_cast<mat3f_handle*>(b);

            static_cast<mat3f_handle*>(target)->mat = ah->mat + bh->mat;
    }
}

void mat3f_mul_mat_inplace(mat3f_t target, const mat3f_t a, const mat3f_t b)
{
    if (target)
    {
        mat3f_handle* ah = static_cast<mat3f_handle*>(a);
        mat3f_handle* bh = static_cast<mat3f_handle*>(b);

        static_cast<mat3f_handle*>(target)->mat = ah->mat * bh->mat;
    }
}

void mat3f_to_array(const mat3f_t mat, float output[9])
{
    if (mat && output)
    {
        mat3f_handle* mh = static_cast<mat3f_handle*>(mat);
        Eigen::Map<Eigen::Matrix3f> output_map(output);
        output_map = mh->mat;  
    }
}

float mat3f_get(const mat3f_t mat, const int row, const int col) 
{
    if (!mat || row < 0 || row > 2 || col < 0 || col > 2)
    {
        return 0.0f;
    }
    mat3f_handle* mh = static_cast<mat3f_handle*>(mat);
    return mh->mat(row, col);
}

void mat3f_set(mat3f_t mat, int row, int col, float value) 
{
    if (mat && row >= 0 && row <= 2 && col >= 0 && col <= 2)
    {
        mat3f_handle* mh = static_cast<mat3f_handle*>(mat);
        mh->mat(row, col) = value;
    }
}

mat3f_t mat3f_transpose(const mat3f_t mat) 
{
    if (!mat) return nullptr;
    mat3f_handle* mh = static_cast<mat3f_handle*>(mat);
    return new mat3f_handle(mh->mat.transpose());
}

void mat3f_transpose_inplace(mat3f_t mat)
{
    if (mat) 
    {
        static_cast<mat3f_handle*>(mat)->mat.transposeInPlace();
    }
}

// This should not be called at most time, it can be dangerous!
// For solving problems like Ax = b, do not use x = A.inv() * b
// Call A.lu().solve(b)
// Or rotation, R.inv() = R.T
// Or homogeneous, H.inv() = 
// | R.T, -R.T * t|
// | 0  ,        1|
mat3f_t mat3f_inverse(const mat3f_t mat)
{
    if (mat)
    {   
        mat3f_handle* mh = static_cast<mat3f_handle*>(mat);
        if(fabsf(mh->mat.determinant()) < 1e-6) return nullptr;
        mat3f_t ret = new mat3f_handle(mh->mat.inverse().eval());
        return ret;
    }
    else 
    {
        return nullptr;
    }
}

// Solve Ax = b, return x;
vec3f_t mat3f_solve(const mat3f_t A, const vec3f_t b)
{
    if (A && b)
    {   
        mat3f_handle* Ah = static_cast<mat3f_handle*>(A);
        if (fabsf(Ah->mat.determinant()) < 1e-6) return nullptr;
        vec3f_handle* bh = static_cast<vec3f_handle*>(b);
        vec3f_t ret = new vec3f_handle(Ah->mat.partialPivLu().solve(bh->vec));

        return ret;
    }
    else
    {
        return nullptr;
    }
}

vec3f_t vec3f_cpy(vec3f_t vec)
{
    if (vec)
    {
        vec3f_t ret = new vec3f_handle(static_cast<vec3f_handle*>(vec)->vec);
        return ret;
    }
    else 
    {
        return nullptr;
    }
}

vec3f_t vec3f_zeros(void)
{
    vec3f_t ret = new vec3f_handle();
    return ret;
}

vec3f_t vec3f_from_array(const float data[3])
{
    vec3f_t ret = new vec3f_handle(data);
    return ret;
}

vec3f_t vec3f_from_xyz(const float x, const float y, const float z)
{   
    const float data[3] = {x, y, z};
    return vec3f_from_array(data);
}

vec3f_t vec3f_add(const vec3f_t a, const vec3f_t b)
{
    if (a && b)
    {
        vec3f_handle* ah = static_cast<vec3f_handle*>(a);
        vec3f_handle* bh = static_cast<vec3f_handle*>(b);
        vec3f_t ret = new vec3f_handle(ah->vec + bh->vec);
        return ret;
    }
    else
    {
        return nullptr;
    }
}

vec3f_t vec3f_sub(const vec3f_t a, const vec3f_t b)
{
    if (a && b)
    {
        vec3f_handle* ah = static_cast<vec3f_handle*>(a);
        vec3f_handle* bh = static_cast<vec3f_handle*>(b);
        vec3f_t ret = new vec3f_handle(ah->vec - bh->vec);
        return ret;
    }
    else
    {
        return nullptr;
    }
}

vec3f_t vec3f_cross(const vec3f_t a, const vec3f_t b)
{
    if (a && b)
    {
        vec3f_handle* ah = static_cast<vec3f_handle*>(a);
        vec3f_handle* bh = static_cast<vec3f_handle*>(b);
        vec3f_t ret = new vec3f_handle(ah->vec.cross(bh->vec));
        return ret;
    }
    else
    {
        return nullptr;
    }
}

float vec3f_dot(const vec3f_t a, const vec3f_t b)
{
    if (a && b)
    {
        vec3f_handle* ah = static_cast<vec3f_handle*>(a);
        vec3f_handle* bh = static_cast<vec3f_handle*>(b);
        return ah->vec.dot(bh->vec);
    }
    else
    {
        return 0.f;
    }
}

vec3f_t vec3f_scale(const vec3f_t vec, const float scalar)
{
    if (vec)
    {   
        vec3f_handle* vh = static_cast<vec3f_handle*>(vec);
        vec3f_t ret = new vec3f_handle(vh->vec * scalar);
        return ret;
    }
    else 
    {
        return nullptr;
    }
}

void vec3f_scale_inplace(vec3f_t vec, const float scalar)
{
    if (vec)
    {
        static_cast<vec3f_handle*>(vec)->vec *= scalar;
    }
}

void vec3f_to_array(const vec3f_t vec, float output[3])
{
    if (vec)
    {
        vec3f_handle* vh = static_cast<vec3f_handle*>(vec);
        Eigen::Map<Eigen::Vector3f> output_map(output);
        output_map = vh->vec;
    }
}

void vec3f_get(const vec3f_t vec, float* x, float* y, float* z)
{
    if (vec)
    {
        vec3f_handle* vh = static_cast<vec3f_handle*>(vec);
        *x = vh->vec(0);
        *y = vh->vec(1);
        *z = vh->vec(2);
    }
}

void vec3f_set(vec3f_t vec, const float x, const float y, const float z)
{
    if (vec)
    {
        vec3f_handle* vh = static_cast<vec3f_handle*>(vec);
        vh->vec(0) = x;
        vh->vec(1) = y;
        vh->vec(2) = z;
    }
}

float vec3f_norm(const vec3f_t vec)
{
    if (vec)
    {
        return static_cast<vec3f_handle*>(vec)->vec.norm();
    }
    else 
    {
        return 0.f;
    }
}

void vec3f_normalize_inplace(vec3f_t vec)
{
    if (vec)
    {
        static_cast<vec3f_handle*>(vec)->vec.normalize();
    }
}

vec3f_t vec3f_normalized(const vec3f_t vec)
{
    if (vec)
    {
        vec3f_t ret = new vec3f_handle(static_cast<vec3f_handle*>(vec)->vec.normalized());
        return ret;
    }
    else
    {
        return nullptr;
    }
}

mat4f_t mat4f_cpy(mat3f_t mat)
{
    if (mat)
    {
        mat4f_t ret = new mat4f_handle(static_cast<mat4f_handle*>(mat)->mat);
        return ret;
    }
    else
    {
        return nullptr;
    }
}

mat4f_t mat4f_eye(void)
{
    mat4f_t ret = new mat4f_handle();
    return ret;
}

mat4f_t mat4f_from_array(const float data[16])
{
    mat4f_t ret = new mat4f_handle(data);
    return ret; 
}

mat4f_t mat4f_add(const mat4f_t a, const mat4f_t b)
{
    if (a && b)
    {   
        mat4f_handle* ah = static_cast<mat4f_handle*>(a);
        mat4f_handle* bh = static_cast<mat4f_handle*>(b);
        mat4f_t ret = new mat4f_handle(ah->mat + bh->mat);
        return ret;
    }
    else
    {
        return nullptr;
    }
}

mat4f_t mat4f_sub(const mat4f_t a, const mat4f_t b)
{
    if (a && b)
    {   
        mat4f_handle* ah = static_cast<mat4f_handle*>(a);
        mat4f_handle* bh = static_cast<mat4f_handle*>(b);
        mat4f_t ret = new mat4f_handle(ah->mat - bh->mat);
        return ret;
    }
    else
    {
        return nullptr;
    }
}

mat4f_t mat4f_mul_mat(const mat4f_t a, const mat4f_t b)
{
    if (a && b)
    {   
        mat4f_handle* ah = static_cast<mat4f_handle*>(a);
        mat4f_handle* bh = static_cast<mat4f_handle*>(b);
        mat4f_t ret = new mat4f_handle(ah->mat * bh->mat);
        return ret;
    }
    else
    {
        return nullptr;
    }
}

vec4f_t mat4f_mul_vec(const mat4f_t mat, const vec4f_t vec)
{
    if (mat && vec)
    {
        mat4f_handle* mh = static_cast<mat4f_handle*>(mat);
        vec4f_handle* vh = static_cast<vec4f_handle*>(vec);
        vec4f_t ret = new vec4f_handle(mh->mat * vh->vec);
        return ret;
    }
    else 
    {
        return nullptr;
    }
}

mat4f_t mat4f_mul_scalar(const mat4f_t mat, const float scalar)
{
    if (mat)
    {
        mat4f_handle* mh = static_cast<mat4f_handle*>(mat);
        mat4f_t ret = new mat4f_handle(mh->mat * scalar);
        return ret;
    }
    else 
    {
        return nullptr;
    }
}

void mat4f_add_inplace(mat4f_t target, const mat4f_t a, const mat4f_t b)
{
    if (target && a && b)
    {
            mat4f_handle* ah = static_cast<mat4f_handle*>(a);
            mat4f_handle* bh = static_cast<mat4f_handle*>(b);

            static_cast<mat4f_handle*>(target)->mat = ah->mat + bh->mat;
    }

}

void mat4f_mul_mat_inplace(mat4f_t target, const mat4f_t a, const mat4f_t b)
{
    if (target && a && b)
    {
        mat4f_handle* ah = static_cast<mat4f_handle*>(a);
        mat4f_handle* bh = static_cast<mat4f_handle*>(b);

        static_cast<mat4f_handle*>(target)->mat = ah->mat * bh->mat;
    }
}

void mat4f_to_array(const mat4f_t mat, float output[16])
{
    if (mat && output)
    {
        mat4f_handle* mh = static_cast<mat4f_handle*>(mat);
        Eigen::Map<Eigen::Matrix4f> output_map(output);
        output_map = mh->mat;  
    }
}

float mat4f_get(const mat4f_t mat, const int row, const int col) 
{
    if (!mat || row < 0 || row > 3 || col < 0 || col > 3)
    {
        return 0.0f;
    }
    mat4f_handle* mh = static_cast<mat4f_handle*>(mat);
    return mh->mat(row, col);
}

void mat4f_set(mat4f_t mat, int row, int col, float value) 
{
    if (mat && row >= 0 && row <= 3 && col >= 0 && col <= 3)
    {
        mat4f_handle* mh = static_cast<mat4f_handle*>(mat);
        mh->mat(row, col) = value;
    }
}

mat4f_t mat4f_transpose(const mat4f_t mat) 
{
    if (!mat) return nullptr;
    mat4f_handle* mh = static_cast<mat4f_handle*>(mat);
    return new mat4f_handle(mh->mat.transpose());
}

void mat4f_transpose_inplace(mat4f_t mat)
{
    if (mat) 
    {
        static_cast<mat4f_handle*>(mat)->mat.transposeInPlace();
    }
}

// This should not be called at most time, it can be dangerous!
// For solving problems like Ax = b, do not use x = A.inv() * b
// Call A.lu().solve(b)
// Or rotation, R.inv() = R.T
// Or homogeneous, H.inv() = 
// | R.T, -R.T * t|
// | 0  ,        1|
mat4f_t mat4f_inverse(const mat4f_t mat)
{
    if (mat)
    {   
        mat4f_handle* mh = static_cast<mat4f_handle*>(mat);
        if(fabsf(mh->mat.determinant()) < 1e-6) return nullptr;
        mat4f_t ret = new mat4f_handle(mh->mat.inverse().eval());
        return ret;
    }
    else 
    {
        return nullptr;
    }
}

// Solve Ax = b, return x;
vec4f_t mat4f_solve(const mat4f_t A, const vec4f_t b)
{
    if (A && b)
    {   
        mat4f_handle* Ah = static_cast<mat4f_handle*>(A);
        if (fabsf(Ah->mat.determinant()) < 1e-6) return nullptr;
        vec4f_handle* bh = static_cast<vec4f_handle*>(b);
        vec4f_t ret = new vec4f_handle(Ah->mat.partialPivLu().solve(bh->vec));

        return ret;
    }
    else
    {
        return nullptr;
    }
}

mat4f_t mat4f_homo(const mat3f_t R, const vec3f_t t)
{
    if (R && t)
    {   
        Eigen::Matrix4f H = Eigen::Matrix4f::Identity();
        H.block<3, 3>(0, 0) = static_cast<mat3f_handle*>(R)->mat;
        H.block<3, 1>(0, 3) = static_cast<vec3f_handle*>(t)->vec;
        mat4f_t ret = new mat4f_handle(H);
        return ret;
    }
    else 
    {
        return nullptr;
    }
}

mat3f_t mat4f_homo_get_R(const mat4f_t H)
{
    if (H)
    {   
        mat3f_t ret = new mat3f_handle(static_cast<mat4f_handle*>(H)->mat.block<3, 3>(0, 0));
        return ret;
    }
    else 
    {
        return nullptr;
    }
}

vec3f_t mat4f_homo_get_t(const mat4f_t H)
{
    if (H)
    {
        vec3f_t ret = new vec3f_handle(static_cast<mat4f_handle*>(H)->mat.block<3, 1>(0, 3));
        return ret;
    }
    else 
    {
        return nullptr;
    }
}

mat4f_t mat4f_homo_inv(const mat4f_t H)
{
    if (H)
    {
        mat4f_handle* ret = new mat4f_handle();
        Eigen::Matrix3f inv_R = static_cast<mat4f_handle*>(H)->mat.block<3, 3>(0, 0).transpose();
        ret->mat.block<3, 3>(0, 0) = inv_R;
        Eigen::Vector3f trs_t = -inv_R * static_cast<mat4f_handle*>(H)->mat.block<3, 1>(0, 3);
        ret->mat.block<3, 1>(0, 3) = trs_t;
        return ret;
    }
    else
    {
        return nullptr;
    }
}

vec4f_t vec4f_cpy(vec4f_t vec)
{
    if (vec)
    {
        vec4f_t ret = new vec4f_handle(static_cast<vec4f_handle*>(vec)->vec);
        return ret;
    }
    else
    {
        return nullptr;
    }
}

vec4f_t vec4f_zeros(void)
{
    vec4f_t ret = new vec4f_handle();
    return ret;
}

vec4f_t vec4f_from_array(const float data[4])
{
    vec4f_t ret = new vec4f_handle(data);
    return ret;
}

vec4f_t vec4f_from_xyz(const float x, const float y, const float z, const float t)
{   
    const float data[4] = {x, y, z, t};
    return vec4f_from_array(data);
}

vec4f_t vec4f_add(const vec4f_t a, const vec4f_t b)
{
    if (a && b)
    {
        vec4f_handle* ah = static_cast<vec4f_handle*>(a);
        vec4f_handle* bh = static_cast<vec4f_handle*>(b);
        vec4f_t ret = new vec4f_handle(ah->vec + bh->vec);
        return ret;
    }
    else
    {
        return nullptr;
    }
}

vec4f_t vec4f_sub(const vec4f_t a, const vec4f_t b)
{
    if (a && b)
    {
        vec4f_handle* ah = static_cast<vec4f_handle*>(a);
        vec4f_handle* bh = static_cast<vec4f_handle*>(b);
        vec4f_t ret = new vec4f_handle(ah->vec - bh->vec);
        return ret;
    }
    else
    {
        return nullptr;
    }
}

float vec4f_dot(const vec4f_t a, const vec4f_t b)
{
    if (a && b)
    {
        vec4f_handle* ah = static_cast<vec4f_handle*>(a);
        vec4f_handle* bh = static_cast<vec4f_handle*>(b);
        return ah->vec.dot(bh->vec);
    }
    else
    {
        return 0.f;
    }
}

vec4f_t vec4f_scale(const vec4f_t vec, const float scalar)
{
    if (vec)
    {   
        vec4f_handle* vh = static_cast<vec4f_handle*>(vec);
        vec4f_t ret = new vec4f_handle(vh->vec * scalar);
        return ret;
    }
    else 
    {
        return nullptr;
    }
}

void vec4f_scale_inplace(vec4f_t vec, const float scalar)
{
    if (vec)
    {
        static_cast<vec4f_handle*>(vec)->vec *= scalar;
    }
}

void vec4f_to_array(const vec4f_t vec, float output[4])
{
    if (vec)
    {
        vec4f_handle* vh = static_cast<vec4f_handle*>(vec);
        Eigen::Map<Eigen::Vector4f> output_map(output);
        output_map = vh->vec;
    }
}

void vec4f_get(const vec4f_t vec, float* x, float* y, float* z, float* t)
{
    if (vec)
    {
        vec4f_handle* vh = static_cast<vec4f_handle*>(vec);
        *x = vh->vec(0);
        *y = vh->vec(1);
        *z = vh->vec(2);
        *t = vh->vec(3);
    }
}

void vec4f_set(vec4f_t vec, const float x, const float y, const float z, const float t)
{
    if (vec)
    {
        vec4f_handle* vh = static_cast<vec4f_handle*>(vec);
        vh->vec(0) = x;
        vh->vec(1) = y;
        vh->vec(2) = z;
        vh->vec(3) = t;
    }
}

float vec4f_norm(const vec4f_t vec)
{
    if (vec)
    {
        return static_cast<vec4f_handle*>(vec)->vec.norm();
    }
    else 
    {
        return 0.f;
    }
}

void vec4f_normalize_inplace(vec4f_t vec)
{
    if (vec)
    {
        static_cast<vec4f_handle*>(vec)->vec.normalize();
    }
}

vec4f_t vec4f_normalized(const vec4f_t vec)
{
    if (vec)
    {
        vec4f_t ret = new vec4f_handle(static_cast<vec4f_handle*>(vec)->vec.normalized());
        return ret;
    }
    else
    {
        return nullptr;
    }
}

}