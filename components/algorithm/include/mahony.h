#pragma once
#include "generic_utils.h"
#include "quaternion_utils.h"
#include <math.h>


typedef struct {
    float Kp;
    float Ki;
    quat_t rotation; /* quaternion of sensor frame relative to auxiliary frame */
    float inte_err_x, inte_err_y, inte_err_z;
    float roll, pitch, yaw;
    float ax, ay, az;
} mahony_params_t;

void update()