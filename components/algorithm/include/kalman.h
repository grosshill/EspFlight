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
} kalman_t;


#endif