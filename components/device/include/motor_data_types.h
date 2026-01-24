#pragma once
#include <stdint.h>

#define MOTOR_THRO_L_LIM 800
#define MOTOR_THRO_H_LIM 2000

enum motor_num {
    MOTOR_1 = 1,
    MOTOR_2,
    MOTOR_3,
    MOTOR_4,
    MOTOR_5,
    MOTOR_6,
    MOTOR_7,
    MOTOR_8
};


/**
 * @brief Defines few motor driving method.
 * @param BLDC Brushless DC motor.
 * @param FOC Field Orientation Control motor.
 */
enum motor_type {
    BLDC,
    FOC
};

/**
 * @brief Defines the spin orientation of motors.
 * @param CW Clockwise.
 * @param CCW Counter-Clockwise.
 */
enum motor_direction {
    CW,
    CCW
};

/**
 * @brief Abstruct object of a motor.
 * @param id Id is the unique number of each motor.
 * @param motor_type Defines how is this motor driven.
 * @param throttle Defines the throttle signal given to this motor, from 0 to 1000, with a constant bias 1000.
 * @note This means that the actual throttle signal ranges from 1000 to 2000.
 * @param throttle_bias Defines the customized throttle signal bias.
 * @param temp Esc feedback signal of motor temperature, only available for configured Dshot protocol.
 */
typedef struct {
    uint8_t id;
    enum motor_type type;
    enum motor_direction direction;
    uint16_t throttle;
    int16_t throttle_bias;
    float temp;
} motor_t;
