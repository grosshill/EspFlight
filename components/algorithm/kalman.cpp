#include "kalman.h"

void Kalman::Kalman_init(Kalman_t config_x, Kalman_t config_y)
{
    kx = config_x;
    ky = config_y;

    /*
      Setup hardware timer for Kalman filter
    */
    
    gptimer_new_timer(&k_timer_config, &k_timer);
    gptimer_enable(k_timer);
    gptimer_start(k_timer);
}

void Kalman::Kalman_cal(float ax, float ay, float az, float gx, float gy, float gz)
{
    gptimer_get_raw_count(k_timer, &count);
    gptimer_set_raw_count(k_timer, 0);
    dt = static_cast<float>(count * 1e-6);

    roll_sqrt = q_sqrt(ax * ax + ay);

    if (roll_sqrt != 0.0)
    {
        roll = atan(ay / roll_sqrt) * RAD_TO_DEG;
    }
    else
    {
        roll = 0.0;
    }
    pitch = atan2(-ax, az) * RAD_TO_DEG;

    if ((pitch < -90 && out_y> 90) || (pitch > 90 && out_y < -90))
    {
        ky.angle = pitch;
        out_y = pitch;
    }
    else
    {
        out_y = Kalman_get_angle(&ky, pitch, gy, dt);
    }

    if (fabs(out_y) > 90) gx = -gx;
    out_x = Kalman_get_angle(&kx, roll, gx, dt);
}

float Kalman::Kalman_get_angle(Kalman_t* k, float new_rate, float new_angle, float dt)
{
    float rate = new_rate - k->bias;
    k->angle += dt * rate;

    k->P[0][0] += dt * (dt * k->P[1][1] - k->P[0][1] - k->P[1][0] + k->Q_angle);
    k->P[0][1] -= dt * k->P[1][1];
    k->P[1][0] -= dt * k->P[1][1];
    k->P[1][1] += dt * k->Q_bias;

    float denom = k->P[0][0] + k->R_meas;
    float k0, k1;
    k0 = k->P[0][0] / denom;
    k1 = k->P[1][0] / denom;

    float factr = new_angle - k->angle;
    k->angle += k0 * factr;
    k->bias += k1 * factr;
    
    float p00 = k->P[0][0];
    float p01 = k->P[0][1];

    k->P[0][0] -= k0 * p00;
    k->P[0][1] -= k0 * p01; 
    k->P[1][0] -= k1 * p00;
    k->P[1][1] -= k1 * p01;

    return k->angle;
}
