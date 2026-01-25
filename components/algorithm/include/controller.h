#pragma once
#include "matrix_utils.h"
#include "quaternion_utils.h"

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
    pid_ctrl_t roll_rate;
    pid_ctrl_t roll_angle;
    pid_ctrl_t pitch_rate;
    pid_ctrl_t pitch_angle;
    pid_ctrl_t yaw_rate;
} stable_ctrl_t;


float track(pid_ctrl_t* pid, const float target, const float current, const float dt);

void set_i_dyn(pid_ctrl_t* pid, const float ki);

