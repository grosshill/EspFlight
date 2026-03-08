#include "controller.h"
#include "esp_log.h"
#include "matrix_utils.h"
#include <stdio.h>
#include <math.h>

// const float format_1[16] = {1, 1, 1, 1, 1, 1, -1, -1, -1, 1, -1, 1, 1, -1, -1, 1};
const float format_1[16] = {1, 1, 1, 1, 1, 1, -1, -1, -1, 1, 1, -1, 1, -1, 1, -1};
/* 0(cw)  3(cc2)
      \  /
      /  \
  1(ccw) 2(cw)
*/
float track(pid_ctrl_t *pid, const float target, const float current, const float dt)
{
    float err = target - current;
    // ESP_LOGI("PID", "target: %.2f, current: %.2f, err: %.2f", target, current, err);
    float p_out = pid->kp * err;
    // ESP_LOGI("PID", "p_out: %.2f", p_out);
    float i_out = 0.f;

    if (pid->ki != 0)
    {
        pid->inte_err += dt * err;
        // ESP_LOGI("PID", "inte_err: %.2f", pid->inte_err);
        if (pid->max_inte > 0.f)
        {
            pid->inte_err = fmaxf(fminf(pid->inte_err, pid->max_inte), -pid->max_inte);
        }
        i_out = pid->ki * pid->inte_err;
        // ESP_LOGI("PID", "i_out: %.2f", i_out);
    }
  
    float drv = (err - pid->prev_err) / dt;
    // ESP_LOGI("PID", "drv: %.2f", drv);
    drv = pid->prev_drv + (dt / (pid->tau + dt)) * (drv - pid->prev_drv);
    float d_out = pid->kd * drv;
    // ESP_LOGI("PID", "d_out: %.2f", d_out);
    pid->prev_err = err;
    pid->prev_drv = drv;

    float out = p_out + i_out + d_out + pid->ff;
    // ESP_LOGI("PID", "ff: %.2f, out: %.2f", pid->ff, out);
    if (pid->max_out > 0.f)
    {
        out = fmaxf(fminf(out, pid->max_out), -pid->max_out);
    }

    return out;
}

mat4f_t mixer_init(void)
{   
    /* 
        For 4-dimensional matrix, it can only represent drones with less than 4 motors.
        Or you have to write mixer without matrix-vector multiplication.
    */
    return mat4f_from_array(format_1);
}

acro_ctrl_t acro_ctrl_init(void)
{
    acro_ctrl_t acro_ctrl = {
        /* roll rate */
        .roll_rate.kp = 8e-4f,
        .roll_rate.ki = 0.f,
        .roll_rate.kd = 3e-4f,
        .roll_rate.ff = 0.f,
        .roll_rate.tau = 1e-3f,
        .roll_rate.prev_err = 0.f,
        .roll_rate.prev_drv = 0.f,
        .roll_rate.inte_err = 0.f,
        .roll_rate.max_out = 1.f,
        .roll_rate.max_inte = 500.f,

        /* pitch rate */
        .pitch_rate.kp = 8e-4f,
        .pitch_rate.ki = 0.f,
        .pitch_rate.kd = 3e-4f,
        .pitch_rate.ff = 0.f,
        .pitch_rate.tau = 1e-3f,
        .pitch_rate.prev_err = 0.f,
        .pitch_rate.prev_drv = 0.f,
        .pitch_rate.inte_err = 0.f,
        .pitch_rate.max_out = 1.f,
        .pitch_rate.max_inte = 500.f,

        /* yaw rate */
        .yaw_rate.kp = 8e-4f,
        .yaw_rate.ki = 0.000001f,
        .yaw_rate.kd = 3e-4f,
        .yaw_rate.ff = 0.f,
        .yaw_rate.tau = 1e-3f,
        .yaw_rate.prev_err = 0.f,
        .yaw_rate.prev_drv = 0.f,
        .yaw_rate.inte_err = 0.f,
        .yaw_rate.max_out = 1.f,
        .yaw_rate.max_inte = 500.f,
    };
    return acro_ctrl;
}

angle_ctrl_t angle_ctrl_init(void)
{
    angle_ctrl_t angle_ctrl = {
        /* roll angle */
        .roll_angle.kp = 0.03f,
        .roll_angle.ki = 0.f,
        .roll_angle.kd = 0.01f,
        .roll_angle.ff = 0.f,
        .roll_angle.tau = 0.f,
        .roll_angle.prev_err = 0.f,
        .roll_angle.prev_drv = 0.f,
        .roll_angle.inte_err = 0.f,
        .roll_angle.max_out = 100.f,
        .roll_angle.max_inte = 500.f,

        /* pitch angle */
        .pitch_angle.kp = 0.03f,
        .pitch_angle.ki = 0.f,
        .pitch_angle.kd = 0.01f,
        .pitch_angle.ff = 0.f,
        .pitch_angle.tau = 0.f,
        .pitch_angle.prev_err = 0.f,
        .pitch_angle.prev_drv = 0.f,
        .pitch_angle.inte_err = 0.f,
        .pitch_angle.max_out = 100.f,
        .pitch_angle.max_inte = 500.f,

        /* roll rate */
        .acro_ctrl.roll_rate.kp = 0.03f,
        .acro_ctrl.roll_rate.ki = 0.f,
        .acro_ctrl.roll_rate.kd = 0.01f,
        .acro_ctrl.roll_rate.ff = 0.f,
        .acro_ctrl.roll_rate.tau = 0.f,
        .acro_ctrl.roll_rate.prev_err = 0.f,
        .acro_ctrl.roll_rate.prev_drv = 0.f,
        .acro_ctrl.roll_rate.inte_err = 0.f,
        .acro_ctrl.roll_rate.max_out = 100.f,
        .acro_ctrl.roll_rate.max_inte = 500.f,

        /* pitch rate */
        .acro_ctrl.pitch_rate.kp = 0.03f,
        .acro_ctrl.pitch_rate.ki = 0.f,
        .acro_ctrl.pitch_rate.kd = 0.01f,
        .acro_ctrl.pitch_rate.ff = 0.f,
        .acro_ctrl.pitch_rate.tau = 0.f,
        .acro_ctrl.pitch_rate.prev_err = 0.f,
        .acro_ctrl.pitch_rate.prev_drv = 0.f,
        .acro_ctrl.pitch_rate.inte_err = 0.f,
        .acro_ctrl.pitch_rate.max_out = 100.f,
        .acro_ctrl.pitch_rate.max_inte = 500.f,

        /* yaw rate */
        .acro_ctrl.yaw_rate.kp = 0.02f,
        .acro_ctrl.yaw_rate.ki = 0.00001f,
        .acro_ctrl.yaw_rate.kd = 0.01f,
        .acro_ctrl.yaw_rate.ff = 0.f,
        .acro_ctrl.yaw_rate.tau = 0.f,
        .acro_ctrl.yaw_rate.prev_err = 0.f,
        .acro_ctrl.yaw_rate.prev_drv = 0.f,
        .acro_ctrl.yaw_rate.inte_err = 0.f,
        .acro_ctrl.yaw_rate.max_out = 100.f,
        .acro_ctrl.yaw_rate.max_inte = 500.f,
    };
    return angle_ctrl;
}

// vel_ctrl_t vel_ctrl_init(void)

float z_pos_ff_track(pid_ctrl_t *pid, const float target, const float current, const float dt)
{
    return 0;
}

vec4f_t CTBR2throttle(const mat4f_t mixer, vec4f_t cmd)
{   
    /* 
        CTBR is short for "Collective Thrust Body Rate", 
        which is the lowest control object in our system.
        We represent CTBR as 4 values, or a vec4x_t vector,
        that is t, r, p, y. t is thrust, that is the thrust from 
        all the motors despites rigid-body rotation. And r, p, y
        represents angular velocity in roll, pitch, and yaw axis.
        Here the all the values are normalized to fabsf(max(x)) = 1:
        That is:
        t: 0 ~ 1
        r, p, y: -1 ~ 1
        m0, m1, m2, m3: 0 ~ 1
    */
    mat4f_mul_vec_inplace(mixer, cmd);

    float m0 = 0.f, m1 = 0.f, m2 = 0.f, m3 = 0.f;
    vec4f_get(cmd, &m0, &m1, &m2, &m3);

    float max_m = fmaxf(fmaxf(m0, m1), fmaxf(m2, m3));
    float min_m = fminf(fminf(m0, m1), fminf(m2, m3));

    // First, reduce collective if any motor exceeds upper bound.
    if (max_m > 1.f)
    {
        const float down_shift = max_m - 1.f;
        m0 -= down_shift;
        m1 -= down_shift;
        m2 -= down_shift;
        m3 -= down_shift;
        max_m -= down_shift;
        min_m -= down_shift;
    }

    // Then recover from lower saturation if there is headroom on top side.
    if (min_m < 0.f && max_m < 1.f)
    {
        const float up_shift = fminf(-min_m, 1.f - max_m);
        m0 += up_shift;
        m1 += up_shift;
        m2 += up_shift;
        m3 += up_shift;
        max_m += up_shift;
        min_m += up_shift;
    }

    // If span still cannot fit [0, 1], compress around midpoint to avoid hard clipping.
    if (min_m < 0.f || max_m > 1.f)
    {
        const float span = max_m - min_m;
        if (span > 1e-6f)
        {
            const float scale = fminf(1.f, 1.f / span);
            const float mid = 0.5f * (max_m + min_m);
            m0 = (m0 - mid) * scale + 0.5f;
            m1 = (m1 - mid) * scale + 0.5f;
            m2 = (m2 - mid) * scale + 0.5f;
            m3 = (m3 - mid) * scale + 0.5f;
        }
    }

    m0 = fminf(fmaxf(m0, 0.f), 1.f);
    m1 = fminf(fmaxf(m1, 0.f), 1.f);
    m2 = fminf(fmaxf(m2, 0.f), 1.f);
    m3 = fminf(fmaxf(m3, 0.f), 1.f);
    vec4f_set(cmd, m0, m1, m2, m3);
    return cmd;
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
    // printf("\n");
    // ESP_LOGI("ACRO", "State: r: %.2f, p: %.2f, y: %.2f", r, p, y);
    float tt = 0, tr = 0, tp = 0, ty = 0;
    vec4f_get(target, &tt, &tr, &tp, &ty);
    // ESP_LOGI("ACRO", "Target: tt: %.2f, tr: %.2f, tp: %.2f, ty: %.2f", tt, tr, tp, ty);
    float cr = track(&acro_c->roll_rate, tr, r,  dt);
    float cp = track(&acro_c->pitch_rate, tp, p, dt);
    float cy = track(&acro_c->yaw_rate, ty, y, dt);
    // float cp = 0;
    // float cy = 0;
    // printf("\n");
    vec4f_set(target, tt, cr, cp, cy);
    // ESP_LOGI("CTRL", "Target: tt: %.2f, cr: %.2f, cp: %.2f, cy: %.2f", tt, cr, cp, cy);
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
        
    /*
        Should add this inclination compensate when the direction of thrust
        is not matched up with the gravity.
        This is a sort of feed forward, and it is not added in the pid controller.
    */

    float incline_comp = fabsf(acosf(cos(r) * cos(p)));
    tt *= (1 + incline_comp);

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