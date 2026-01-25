#include "eigen_types.hpp"
#include "quaternion_utils.h"
#include "matrix_utils.h"

extern "C" {

quatf_t quatf_arr(const float arr[4])
{
    quatf_t ret = new quatf_handle(arr);
    return ret;
}

quatf_t quatf_wxyz(const float w, const float x, const float y, const float z)
{
    quatf_t ret = new quatf_handle(w, x, y, z);
    return ret;
}

void quatf_set(quatf_t q, const float w, const float x, const float y, const float z)
{
    if (q)
    {
        quatf_handle* qh = static_cast<quatf_handle*>(q);
        qh->quat.w() = w;
        qh->quat.x() = x;
        qh->quat.y() = y;
        qh->quat.z() = z;
        qh->quat.normalize();
    }
    else 
    {
        q = quatf_wxyz(w, x, y, z);
    }
}

void quatf_free(quatf_t q)
{
    if (q != nullptr)
    {
        delete static_cast<quatf_handle*>(q);
    }
}

void quatf_get(quatf_t q, float *w, float *x, float *y, float *z)
{
    if (q && w && x && y && z)
    {
        quatf_handle* qh = static_cast<quatf_handle*>(q);
        *w = qh->quat.w();
        *x = qh->quat.x();
        *y = qh->quat.y();
        *z = qh->quat.z();
    }
}

quatf_t quatf_mul(const quatf_t ql, const quatf_t qr)
{
    if (ql && qr)
    {   
        quatf_handle* qhl = static_cast<quatf_handle*>(ql);
        quatf_handle* qhr = static_cast<quatf_handle*>(qr);
        quatf_t ret = new quatf_handle(qhl->quat * qhr->quat);
        return ret;
    }
    return nullptr;
}

void quatf_conj(quatf_t q)
{
    if (q)
    {   
        quatf_handle* qh = static_cast<quatf_handle*>(q);
        qh->quat = qh->quat.conjugate();
    }
}

void quatf_norm(quatf_t q)
{
    if (q)
    {   
        quatf_handle* qh = static_cast<quatf_handle*>(q);
        qh->quat.normalize();
    }
}

quatf_t axis_angle2quatf(float rad, float x, float y, float z)
{
    quatf_handle* ret = new quatf_handle;
    ret->quat = Eigen::AngleAxisf(rad, Eigen::Vector3f(x, y, z));
    return static_cast<quatf_t>(ret);
}

void quatf2axis_angle(const quatf_t q, float *rad, float *x, float *y, float *z)
{
    if (q && rad && x && y && z)
    {   
        quatf_handle* qh = static_cast<quatf_handle*>(q);
        Eigen::AngleAxisf axis_angle(qh->quat);
        *rad = axis_angle.angle();
        *x = axis_angle.axis().x();
        *y = axis_angle.axis().y();
        *z = axis_angle.axis().z();
    }
}

int quatf_is_valid(const quatf_t q)
{
    if (!q) return 0;
    quatf_handle* qh = static_cast<quatf_handle*>(q);
    return qh->quat.coeffs().allFinite();
}

// mat3_t to_rot_mat(const quatf_t q)
// {   
//     if (q)
//     {     
//         quatf_handle* qh = static_cast<quatf_handle*>(q);
//         mat3_t ret = new mat_handle(qh->quat.toRotationMatrix());
//         return ret;
//     }
//     else
//     {
//         mat3_t ret = eye3();
//         return ret;
//     }
// }

}