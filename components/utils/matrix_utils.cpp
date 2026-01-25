#include "eigen_types.hpp"
#include "matrix_utils.h"

extern "C" {

// mat3_t mat3(void)
// {
//     mat3_t ret = new mat3_handle();
//     return ret;
// }


// mat3_t mat3_arr(const float* arr)
// {
//     mat3_t ret = new mat3_handle();
//     ret->mat3 << arr[0], arr[1], arr[2],
//                  arr[3], arr[4], arr[5],
//                  arr[6], arr[7], arr[8];
//     return ret;
// }

// mat3_t eye3(void)
// {
//     const float arr[] = {1, 0, 0, 0, 1, 0, 0, 0, 1};
//     return mat3_arr(arr);
// }

// void mat3_free(mat3_t mat3)
// {
//     if (mat3 != nullptr)
//     {
//         delete mat3;
//     }
// }

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
        delete static_cast<vec3f_handle*>(vec);
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
        return mat3f_eye();
    }
}

}