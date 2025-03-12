#ifndef KALMAN_H
#define KALMAN_H

#include "esp_dsp.h"
#include "freertos/FreeRTOS.h"

typedef struct
{
    double Q_angle;
    double Q_bias;
    double R_measure;
    double angle;
    double bias;
    dspm::Mat P(2, 2);
} kalman_t;


#endif