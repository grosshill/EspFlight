#pragma once
#include "driver/ledc.h"
#include "motor_data_types.h"
#include "debug_utils.h"

#define PWM_MOTOR_TAG "PWM_MOTOR"

#define PWM_TIMER LEDC_TIMER_0
#define PWM_SPEED_MODE LEDC_LOW_SPEED_MODE
#define PWM_DUTY_RES LEDC_TIMER_10_BIT
#define PWM_INTR_TYPE LEDC_INTR_DISABLE
#define PWM_AUTO_CLK LEDC_AUTO_CLK
#define PWM_FREQUENCY (50) /* 50 Hz for standard RC PWM frequency */

#define MOTOR_ID_0 (0)
#define MOTOR_ID_1 (1)
#define MOTOR_ID_2 (2)
#define MOTOR_ID_3 (3)
#define MOTOR_ID_4 (4)
#define MOTOR_ID_5 (5)
#define MOTOR_ID_6 (6)
#define MOTOR_ID_7 (7)

typedef struct {
    ledc_channel_config_t m1_cfg;
    ledc_channel_config_t m2_cfg;
    ledc_channel_config_t m3_cfg;
    ledc_channel_config_t m4_cfg;
    ledc_channel_config_t m5_cfg;
    ledc_channel_config_t m6_cfg;
    ledc_channel_config_t m7_cfg;
    ledc_channel_config_t m8_cfg;
} pwm_8_motors_ch_config_t;

esp_err_t pwm_motor_setup(const ledc_timer_config_t* timer_cfg, enum motor_num num);

esp_err_t pwm_throttle_set(uint8_t id, uint16_t value);