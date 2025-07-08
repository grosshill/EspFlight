#ifndef BMP280_H
#define BMP280_H

#include "drivers.h"
#define BMP280_DEVICE 0x76
// #define BMP280_DEVICE 0x68
#define BMP280_CTRL_MEAS 0xF4
#define BMP280_CONFIG 0xF5
#define BMP280_COLAB_START_REG 0x88
#define BMP280_PRESS_REG 0xF7
#define BMP280_TEMP_REG 0xFA

#define BMP280_PWR_ON_NORMAL 0b11

enum bmp280_iir_coeff {
    iir_disable,
    iir_coeff_2,
    iir_coeff_4,
    iir_coeff_8,
    iir_coeff_16,
};

enum bmp280_temp_oversamp {
    t_overs_1 = 1,
    t_overs_2,
    t_overs_4,
    t_overs_8,
    t_overs_16,
};

enum bmp280_press_oversamp {
    p_overs_1 = 1,
    p_overs_2,
    p_overs_4,
    p_overs_8,
    p_overs_16,
};

enum bmp280_time_standby {
    t_standby_0_5ms,
    t_standby_62_5ms,
    t_standby_125ms,
    t_standby_250ms,
    t_standby_500ms,
    t_standby_1s,
    t_standby_2s,
    t_standby_4s,
};

typedef struct {
    unsigned short T1;
    short T2;
    short T3;
    unsigned short P1;
    short P2;
    short P3;
    short P4;
    short P5;
    short P6;
    short P7;
    short P8;
    short P9;
} bmp_colab_params_t;

class BMP280 : public EF_I2C::EF_I2C_device
{
    public:
        BMP280(const uint8_t dev_addr);
        ~BMP280(){};

        float bmp280_get_height();

    private:
        bmp_colab_params_t colab_params;
        int32_t t_fine;
        void bmp280_get_colab_params();
        int32_t bmp280_read_temp();
        uint32_t bmp280_read_press();
};
#endif