#include "kalman.h"
#include <math.h>

double kalman_iter(kalman_t* kalman, double new_angle, double new_rate, double dt)
{
    double rate = new_rate - kalman->bias;
    kalman->angle += dt * rate;

    kalman->P_00 += dt * (dt * kalman->P_11 - kalman->P_01 * 2 + kalman->Q_angle);
    kalman->P_01 -= dt * kalman->P_11;
    kalman->P_11 += kalman->Q_bias * dt;

    double s = kalman->P_00 + kalman->R_measure;
    double k1, k2;
    k1 = kalman->P_00 / s;
    k2 = kalman->P_01 / s;

    double y = new_angle - kalman->angle;
    kalman->angle += k1 * y;
    kalman->bias += k2 * y;

    double temp_P00 = kalman->P_00;
    double temp_P01 = kalman->P_01;

    kalman->P_00 -= k1 * temp_P00;
    kalman->P_01 -= k1 * temp_P01;
    kalman->P_11 -= k2 * temp_P01;

    return kalman->angle;
}