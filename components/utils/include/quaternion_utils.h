#pragma once
#include <stddef.h>
#ifdef __cplusplus
extern "C" {
#endif

/* opaque handle of quaternions */
typedef void* quatf_t;

/* create a quaternion from array in wxyz */
quatf_t quatf_arr(const float arr[4]);

/* create a quaternion from given parameters */
quatf_t quatf_wxyz(const float w, const float x, const float y, const float z);

/* set value of a existing quaternion */
void quatf_set(quatf_t q, const float w, const float x, const float y, const float z);

/* release memory*/
void quatf_free(quatf_t q);

/* get value of a quaternion */
void quatf_get(quatf_t q, float* w, float* x, float* y, float* z);

/* implement quaternion multiplication */
quatf_t quatf_mul(const quatf_t ql, const quatf_t qr);

/* conjugate a quaternion */
void quatf_conj(quatf_t q);

/* normalize a quaternion */
void quatf_norm(quatf_t q);

/* axis angle to quaternion */
quatf_t axis_angle2quat(float rad, float x, float y, float z);

/* quaternion to axis angle */
void quat2axis_angle(const quatf_t q, float* rad, float* x , float* y, float* z);

/* check if a quaternion is singular */
int quatf_is_valid(const quatf_t q);


// void mat2quat(quatf_t q);

// void quat2mat(quatf_t q);

// void euler2quat(mat_t q);

// void quat2euler(mat_t q);

// void mat2euler(void);

// void euler2mat(void);

#ifdef __cplusplus
}
#endif