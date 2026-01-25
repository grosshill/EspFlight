#pragma once
#include "generic_utils.h"
#include "sensor_data_types.h"
#include "quaternion_utils.h"
#include <math.h>


typedef struct {
    float Kp;
    float Ki;
    quatf_t rotation; /* quaternion of sensor frame relative to auxiliary frame */
    float inte_err_x, inte_err_y, inte_err_z;
    float roll, pitch, yaw;
    float grax, gray, graz;
} mahony_params_t;

void mahony_update(const acc_pack_t acc, const gyro_pack_t gyro, mahony_params_t* params, float dt);

void mahony_get_rad(const acc_pack_t acc, const gyro_pack_t gyro, mahony_params_t* params, float dt);

void mahony_get_deg(const acc_pack_t acc, const gyro_pack_t gyro, mahony_params_t* params, atti_pack_t* attitude, float dt);