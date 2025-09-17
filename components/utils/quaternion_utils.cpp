#include "quaternion_utils.h"
#include <Eigen/Geometry>

struct quat_handle {
    Eigen::Quaternionf quat;
};

extern "C" {

quat_t quat(void)
{
    quat_t ret = new quat_handle();

    ret->quat.w() = 1.f;
    ret->quat.x() = .0f;
    ret->quat.y() = .0f;
    ret->quat.z() = .0f;
    return ret;
}

quat_t quat_wxyz(float w, float x, float y, float z)
{
    quat_t ret = new quat_handle();

    ret->quat.w() = w;
    ret->quat.x() = x;
    ret->quat.y() = y;
    ret->quat.z() = z;
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


}