#include "controller.h"
#include <math.h>

float track(pid_ctrl_t *pid, const float target, const float current, const float dt)
{
    float err = target - current;
    float p_out = pid->kp * err;
    float i_out = 0.f;

    if (pid->ki != 0)
    {
        pid->inte_err += dt * err;
        if (pid->max_inte > 0.f)
        {
            pid->inte_err = fmax(fmin(pid->inte_err, pid->max_inte), -pid->max_inte);
        }
        i_out = pid->ki * pid->inte_err;
    }
  
    float drv = (err - pid->prev_err) / dt;
    drv = pid->prev_drv + (dt / (pid->tau + dt)) * (drv - pid->prev_drv);
    float d_out = pid->kd * drv;
    pid->prev_err = err;
    pid->prev_drv = drv;

    float out = p_out + i_out + d_out + pid->ff;

    if (pid->max_out > 0.f)
    {
        out = fmax(fmin(out, pid->max_out), -pid->max_out);
    }

    return out;
}

void set_i_dyn(pid_ctrl_t *pid, const float ki)
{
    pid->ki = ki;
    /*
        Should reset integral error after dynamically activate ki.
    */
    pid->inte_err = 0;
}