#ifndef KALMAN_H
#define KALMAN_H

#include <iostream>
#include "esp_dsp.h"
#include "driver/gptimer_types.h"
#include "driver/gptimer.h"
#include "EF_math.h"
#ifdef __cplusplus
extern "C" {
#endif

#ifdef __cplusplus
}
#endif

typedef struct {
    float Q_angle;
    float Q_bias;
    float R_meas;
    float angle = 0;
    float bias = 0;
    float P[2][2] = {{1, 0}, {0, 1}};
} Kalman_t;

class Kalman
{
    public:
        Kalman(){};
        ~Kalman(){};

        void Kalman_init(Kalman_t config_x, Kalman_t config_y);
        void Kalman_cal(float ax, float ay, float az, float gx, float gy, float gz);
        float out_x, out_y;

    private:
        float Kalman_get_angle(Kalman_t* k, float new_rate, float new_angle, float dt);
        Kalman_t kx, ky;
        
        /*
          hardware timer with 1MHz frequency, 1us resolution.
        */

        gptimer_handle_t k_timer = NULL;
        gptimer_config_t k_timer_config = {
            .clk_src = GPTIMER_CLK_SRC_DEFAULT,
            .direction = GPTIMER_COUNT_UP,
            .resolution_hz = 1 * 1000 * 1000,
        };
        
        /*
          Timer period for Kalman filter
        */
        float dt = 0;
        uint64_t count = 0;

        /*
          pose
        */
        float roll, roll_sqrt, pitch;
};

#endif