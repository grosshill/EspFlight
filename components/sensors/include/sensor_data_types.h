#pragma once
#include "stdint.h"
#include "matrix_utils.h"

#ifdef __cplusplus
extern "C" {
#endif
/* ax, ay, az is the row data of accelerometer in m/sec^2 */
typedef struct {
    float ax;
    float ay;
    float az;
} acc_pack_t;

/* gx, gy, gz is the row data of gyroscope in deg/sec */
typedef struct {
    float gx;
    float gy;
    float gz;
} gyro_pack_t;

/* mx, my, mz is the row data of magnetometer in deg */
typedef struct {
    float mx;
    float my;
    float mz;
} mag_pack_t;

/* roll, pitch, yaw is used to represent attitude in deg */
typedef struct {
    vec3f_t ang;
    vec3f_t omg;
    vec3f_t pos;
    vec3f_t vel;
    vec3f_t acc;
} state_t;

/* temperature in ℃ */
typedef struct {
    float temp;
} temp_pack_t;

/**
 * @param temp is temperature in (℃ / 100)
 * @param t_fine is the mid variable used to compute final pressure
 * @param press is the mid variable used to compute final pressure
 * @param pressure is the final pressure in (Pa / 256)
 * @param height is the final height in m
 *  */ 
typedef struct {
    int32_t temp;
    int32_t t_fine;
    int32_t press;
    uint32_t pressure;
    float height;   
} baro_pack_t;

state_t* state(void);
void _state(state_t* s);

#ifdef __cplusplus
}
#endif