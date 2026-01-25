#pragma once
#include <Eigen/Dense> 

template<typename Scalar>
struct quat_handle {
    Eigen::Quaternion<Scalar> quat;
    
    quat_handle() : quat(Eigen::Quaternion<Scalar>::Identity()) {}
    
    explicit quat_handle(const Scalar arr[4])
        : quat(arr[0], arr[1], arr[2], arr[3]) {quat.normalize();} // Eigen: w, x, y, z
    
    explicit quat_handle(Scalar w, Scalar x, Scalar y, Scalar z)
        : quat(w, x, y, z) {quat.normalize();}
    
    explicit quat_handle(const Eigen::Quaternion<Scalar>& q) : quat(q) {quat.normalize();}
};

template<typename Scalar, int Dim>
struct mat_handle {
    Eigen::Matrix<Scalar, Dim, Dim> mat;
    
    mat_handle() : mat(Eigen::Matrix<Scalar, Dim, Dim>::Identity()) {}
    
    explicit mat_handle(const Eigen::Matrix<Scalar, Dim, Dim>& m) : mat(m) {}
    
    // Eigen: column first
    explicit mat_handle(const Scalar* data)
        : mat(Eigen::Map<const Eigen::Matrix<Scalar, Dim, Dim>>(data)) {}
};

template<typename Scalar, int Dim>
struct vec_handle {
    Eigen::Matrix<Scalar, Dim, 1> vec;
    
    vec_handle() : vec(Eigen::Matrix<Scalar, Dim, 1>::Zero()) {}
    
    explicit vec_handle(const Scalar* data)
        : vec(Eigen::Map<const Eigen::Matrix<Scalar, Dim, 1>>(data)) {}
    
    explicit vec_handle(const Eigen::Matrix<Scalar, Dim, 1>& v) : vec(v) {}
    
    explicit vec_handle(Scalar value) : vec(Eigen::Matrix<Scalar, Dim, 1>::Constant(value)) {}
};

using quatf_handle = quat_handle<float>;
using quatd_handle = quat_handle<double>;

using mat3f_handle = mat_handle<float, 3>;
using mat3d_handle = mat_handle<double, 3>;
using mat4f_handle = mat_handle<float, 4>;
using mat4d_handle = mat_handle<double, 4>;

using vec3f_handle = vec_handle<float, 3>;
using vec3d_handle = vec_handle<double, 3>;
using vec4f_handle = vec_handle<float, 4>;
using vec4d_handle = vec_handle<double, 4>;