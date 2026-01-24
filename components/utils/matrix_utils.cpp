#include "eigen_types.hpp"
#include "matrix_utils.h"

extern "C" {

mat3_t mat3(void)
{
    mat3_t ret = new mat3_handle();
    return ret;
}


mat3_t mat3_arr(const float* arr)
{
    mat3_t ret = new mat3_handle();
    ret->mat3 << arr[0], arr[1], arr[2],
                 arr[3], arr[4], arr[5],
                 arr[6], arr[7], arr[8];
    return ret;
}

mat3_t eye3(void)
{
    const float arr[] = {1, 0, 0, 0, 1, 0, 0, 0, 1};
    return mat3_arr(arr);
}

void mat3_free(mat3_t mat3)
{
    if (mat3 != nullptr)
    {
        delete mat3;
    }
}

}