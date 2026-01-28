#include "controller.h"
#include "matrix_utils.h"
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
            pid->inte_err = fmaxf(fminf(pid->inte_err, pid->max_inte), -pid->max_inte);
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
        out = fmaxf(fminf(out, pid->max_out), -pid->max_out);
    }

    return out;
}

float z_pos_ff_track(pid_ctrl_t *pid, const float target, const float current, const float dt)
{
    return 0;
}

void set_i_dyn(pid_ctrl_t *pid, const float ki)
{
    pid->ki = ki;
    /*
        Should reset integral error after dynamically activate ki.
    */
    pid->inte_err = 0;
}

vec4f_t acro_ctrl(state_t* state, vec4f_t target, acro_ctrl_t *acro_c, const float dt)
{
    float r = 0, p = 0, y = 0;
    vec3f_get(state->omg, &r, &p, &y);
    float tt = 0, tr = 0, tp = 0, ty = 0;
    vec4f_get(target, &tt, &tr, &tp, &ty);
    float cr = track(&acro_c->roll_rate, tr, r,  dt);
    float cp = track(&acro_c->pitch_rate, tp, p, dt);
    float cy = track(&acro_c->yaw_rate, ty, y, dt);
    vec4f_set(target, tt, cr, cp, cy);
    return target;
}

vec4f_t angle_ctrl(state_t* state, vec4f_t target, angle_ctrl_t* angle_c, const float dt)
{
    float r = 0, p = 0, y = 0;
    vec3f_get(state->ang, &r, &p, &y);
    float tt = 0, tr = 0, tp = 0, ty = 0;
    vec4f_get(target, &tt, &tr, &tp, &ty);
    float cr = track(&angle_c->roll_angle, tr, r, dt);
    float cp = track(&angle_c->pitch_angle,  tp, p, dt);
    float cy = ty;
    vec4f_set(target, tt, cr, cp, cy);
    target = acro_ctrl(state, target, &angle_c->acro_ctrl, dt);
    return target;
}

vec4f_t vel_ctrl(state_t* state, vec4f_t target, vel_ctrl_t* vel_c, const float dt)
{
    float x = 0, y = 0, z = 0;
    vec3f_get(state->vel, &x, &y, &z);
    float tt = 0, tr = 0, tp = 0, ty = 0;
    vec4f_get(target, &tt, &tr, &tp, &ty);
    float cr = track(&vel_c->y_vel, tr, y, dt);
    float cp = track(&vel_c->x_vel, tp, x, dt);
    float cy = ty;
    float ct = z_pos_ff_track(&vel_c->z_pos, tt, z, dt);
    vec4f_set(target, ct, cr, cp, cy);
    target = angle_ctrl(state, target, &vel_c->angle_ctrl, dt);
    return target;
}