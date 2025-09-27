#include "motor_pwm.h"

const uint8_t pwm_motor_gpio[8] = {
    GPIO_NUM_4,
    GPIO_NUM_5,
    GPIO_NUM_6,
    GPIO_NUM_7,
    GPIO_NUM_8,
    GPIO_NUM_9,
    GPIO_NUM_10,
    GPIO_NUM_11,
};

const uint8_t pwm_motor_channel[8] = {
    LEDC_CHANNEL_0,
    LEDC_CHANNEL_1,
    LEDC_CHANNEL_2,
    LEDC_CHANNEL_3,
    LEDC_CHANNEL_4,
    LEDC_CHANNEL_5,
    LEDC_CHANNEL_6,
    LEDC_CHANNEL_7,
};

esp_err_t pwm_motor_setup(const ledc_timer_config_t* timer_cfg, enum motor_num num)
{
    EF_ERR_CHECK(ledc_timer_config(timer_cfg), PWM_MOTOR_TAG);

    ledc_channel_config_t ch_cfg = {
        .speed_mode = PWM_SPEED_MODE,
        .timer_sel = PWM_TIMER,
        .intr_type = PWM_INTR_TYPE,
        .duty = 0,
        .hpoint = 0,
    };

    for(int i = 0; i < (uint8_t)num; i++)
    {
        ch_cfg.gpio_num = pwm_motor_gpio[i];
        ch_cfg.channel = pwm_motor_channel[i];
        EF_ERR_CHECK(ledc_channel_config(&ch_cfg), PWM_MOTOR_TAG);
    }

    return ESP_OK;
}

esp_err_t pwm_throttle_set(uint8_t id, uint16_t value)
{
    /*
        pwm signal 1000  ->  0.5ms / 20ms  ->  25 / 1024 %2.5
        pwm signal 2000  ->  2.5ms / 20ms  -> 128 / 1024 %12.5
    */
    uint16_t duty = (uint16_t)((value - 1000) * 103.f / 1000.f + 25);
    EF_ERR_CHECK(ledc_set_duty(PWM_SPEED_MODE, pwm_motor_channel[id], duty), PWM_MOTOR_TAG);
    EF_ERR_CHECK(ledc_update_duty(PWM_SPEED_MODE, pwm_motor_channel[id]), PWM_MOTOR_TAG);

    return ESP_OK;
}