#pragma once
#include "quaternion_utils.h"
#include "freertos/FreeRTOS.h"

typedef struct {
    float Kp;
    float Ki;
    quat_t rotation; /* quaternion of sensor frame relative to auxiliary frame */

} mahony_params_t;