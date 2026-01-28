#pragma once
#include "matrix_utils.h"
#include "quaternion_utils.h"
#include "sensor_data_types.h"

#define PID_FREQ (1000U)
typedef struct {
    float kp;               // Propotional term
    float ki;               // Integral term
    float kd;               // Derivative term
    float ff;               // Feed Forward
    float tau;              // Derivative 1-order LPF coefficient
    float prev_err;         // Error from last stamp
    float prev_drv;         // Derivative from last stamp
    float inte_err;         // Integral from last stamp
    float max_out;          // Maximum absolute value of output
    float max_inte;         // Maximum absolute value of Intergral
} pid_ctrl_t;

typedef struct {
    float ff;
    float tau;
    float max_out;
} feed_forward_t;

typedef struct
{
    pid_ctrl_t roll_rate;
    pid_ctrl_t pitch_rate;
    pid_ctrl_t yaw_rate;
} acro_ctrl_t;

typedef struct {
    pid_ctrl_t roll_angle;
    pid_ctrl_t pitch_angle;
    feed_forward_t thrust_ff;
    acro_ctrl_t acro_ctrl;
} angle_ctrl_t;

typedef struct {
    pid_ctrl_t x_vel;
    pid_ctrl_t y_vel;
    pid_ctrl_t z_pos;
    angle_ctrl_t angle_ctrl;    
} vel_ctrl_t;

float track(pid_ctrl_t* pid, const float target, const float current, const float dt);

void set_i_dyn(pid_ctrl_t* pid, const float ki);

vec4f_t acro_ctrl(state_t* state, vec4f_t target, acro_ctrl_t* acro_c, const float dt);

vec4f_t angle_ctrl(state_t* state, vec4f_t target, angle_ctrl_t* angle_c, const float dt);

vec4f_t vel_ctrl(state_t* state, vec4f_t target, vel_ctrl_t* vel_c, const float dt);
