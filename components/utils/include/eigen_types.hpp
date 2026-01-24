#pragma once
#include <Eigen/Dense>

struct quat_handle {
#ifndef EIGEN_DOUBLE_DTYPE
    Eigen::Quaternionf quat;
    quat_handle(const float arr[4])
    {
        quat.w() = arr[0];
        quat.x() = arr[1];
        quat.y() = arr[2];
        quat.z() = arr[3];
    }
    explicit quat_handle(const float w, const float x, const float y, const float z)
    {
        quat.w() = w;
        quat.x() = x;
        quat.y() = y;
        quat.z() = z;
    }
#else
    Eigen::Quaterniond quat;
    quat_handle(const double arr[4])
    {
        quat.w() = arr[0];
        quat.x() = arr[1];
        quat.y() = arr[2];
        quat.z() = arr[3];
    }
    explicit quat_handle(const double w, const double x, const double y, const double z)
    {
        quat.w() = w;
        quat.x() = x;
        quat.y() = y;
        quat.z() = z;
    }
#endif
};

struct mat3_handle {
#ifndef EIGEN_DOUBLE_DTYPE
    Eigen::Matrix3f mat3;
    mat3_handle() : mat3(Eigen::Matrix3f::Identity()) {}
    explicit mat3_handle(const Eigen::Matrix3f& m) : mat3(m) {}
    
    explicit mat3_handle(const float arr[9])
    {
        mat3 << arr[0], arr[1], arr[2],
                arr[3], arr[4], arr[5],
                arr[6], arr[7], arr[8];
    }
#else
    Eigen::Matrix3d mat3;
    mat3_handle() : mat3(Eigen::Matrix3d::Identity()){}
    explicit mat3_handle(const Eigen::Matrix3d& m) : mat3(m) {}
    
    explicit mat3_handle(const double arr[9]) {
        mat3 << arr[0], arr[1], arr[2],
                arr[3], arr[4], arr[5],
                arr[6], arr[7], arr[8];
    }
#endif
};

struct vec3_handle {
#ifndef EIGEN_DOUBLE_DTYPE
    Eigen::Vector3f vec3;
    vec3_handle() : vec3(Eigen::Vector3f::Zero()) {}
#else
    Eigen::Vector3d vec3;
#endif
};