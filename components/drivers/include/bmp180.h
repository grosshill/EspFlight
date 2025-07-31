#ifndef BMP180_H
#define BMP180_H

#define BMP180_DEVICE 0x77
#define BMP180_CTRL_ADDR 0xF4
#define BMP180_CTRL_TEMP 0x2E
// #define BMP180_CTRL_OSS 0x34
#define BMP180_CTRL_OSS 0xF4
#define BMP180_COLAB_START 0xAA
#define BMP180_TEMP_START 0xF6

#include "i2c_manager.h"

typedef struct {
    short AC1;
    short AC2;
    short AC3;
    unsigned short AC4;
    unsigned short AC5;
    unsigned short AC6;
    short B1;
    short B2;
    short MB;
    short MC;
    short MD;
} bmp_colab_params_t;

bmp_colab_params_t bmp180_init(i2c_master_dev_handle_t dev_handle);

int32_t bmp180_read(i2c_master_dev_handle_t dev_handle, bmp_colab_params_t colab);

#endif
