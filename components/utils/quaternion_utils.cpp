#include "eigen_types.hpp"
#include "quaternion_utils.h"
#include "matrix_utils.h"

extern "C" {

quat_t quat_arr(const float arr[4])
{
    quat_t ret = new quat_handle(arr);
    ret->quat.normalize();
    return ret;
}

quat_t quat_wxyz(float w, float x, float y, float z)
{
    quat_t ret = new quat_handle(w, x, y, z);
    return ret;
}

void quat_free(quat_t q)
{
    if (q != nullptr)
    {
        delete q;
    }
}

void quat_set(quat_t q, float w, float x, float y, float z)
{
    if (q)
    {
        q->quat.w() = w;
        q->quat.x() = x;
        q->quat.y() = y;
        q->quat.z() = z;
        q->quat.normalize();
    }
}

void quat_get(quat_t q, float *w, float *x, float *y, float *z)
{
    if (q && w && x && y && z)
    {
        *w = q->quat.w();
        *x = q->quat.x();
        *y = q->quat.y();
        *z = q->quat.z();
    }
}

void quat_mul(quat_t ret, const quat_t ql, const quat_t qr)
{
    if (ret && ql && qr)
    {
        ret->quat = ql->quat * qr->quat;
    }
}

void quat_conj(quat_t q)
{
    if (q)
    {
        q->quat = q->quat.conjugate();
    }
}

void quat_norm(quat_t q)
{
    if (q)
    {
        q->quat.normalize();
    }
}

void axis_angle2quat(const quat_t q, float rad, float x, float y, float z)
{
    if (q)
    {
        q->quat = Eigen::AngleAxisf(rad, Eigen::Vector3f(x, y, z));
    }
}

void quat2axis_angle(const quat_t q, float *rad, float *x, float *y, float *z)
{
    if (q && rad && x && y && z)
    {
        Eigen::AngleAxisf axis_angle(q->quat);
        *rad = axis_angle.angle();
        *x = axis_angle.axis().x();
        *y = axis_angle.axis().y();
        *z = axis_angle.axis().z();
    }
}

int quat_is_valid(const quat_t q)
{
    if (!q) return 0;
    return q->quat.coeffs().allFinite();
}

mat3_t to_rot_mat(const quat_t q)
{   
    if (q)
    {   
        mat3_t ret = new mat3_handle(q->quat.toRotationMatrix());
        return ret;
    }
    else
    {
        mat3_t ret = eye3();
        return ret;
    }
}

}