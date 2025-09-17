#pragma once

#include "freertos/FreeRTOS.h"
// #include "esp_timer.h"


typedef struct
{
    double Q_angle;
    double Q_bias;
    double R_measure;
    double angle;
    double bias;
    double P_00;
    double P_11;
    double P_01;
} kalman_t;

typedef struct
{
    double est_gx;
    double est_gy;
} kalman_est_angle_t;