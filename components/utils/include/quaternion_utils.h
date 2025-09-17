#pragma once
#include <stdint.h>
#include <stddef.h>
#ifdef __cplusplus
extern "C" {
#endif

/* opaque handle of quaternions */
typedef struct quat_handle* quat_t;

/* create a unit quaternion, i.e. {1, 0, 0, 0} */
quat_t quat(void);

/* create a quaternion from given parameters */
quat_t quat_wxyz(float w, float x, float y, float z);

/* release memory*/
void quat_free(quat_t q);

/* set value of a quaternion */
void quat_set(quat_t q, float w, float x, float y, float z);

/* get value of a quaternion */
void quat_get(quat_t q, float* w, float* x, float* y, float* z);

/* implement quaternion multiplication */
void quat_mul(quat_t ret, const quat_t ql, const quat_t qr);

/* conjugate a quaternion */
void quat_conj(quat_t q);

/* normalize a quaternion */
void quat_norm(quat_t q);

/* axis angle to quaternion */
void axis_angle2quat(const quat_t q, float rad, float x, float y, float z);

/* quaternion to axis angle */
void quat2axis_angle(const quat_t q, float* rad, float* x , float* y, float* z);

/* check if a quaternion is singular */
int quat_is_valid(const quat_t q);


// void mat2quat(quat_t q);

// void quat2mat(quat_t q);

// void euler2quat(mat_t q);

// void quat2euler(mat_t q);

// void mat2euler(void);

// void euler2mat(void);

#ifdef __cplusplus
}
#endif