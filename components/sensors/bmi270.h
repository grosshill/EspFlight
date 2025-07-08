#ifndef BMI270_H
#define BMI270_H
#include "drivers.h"
#include "kalman.h"

#define BMI270_DEVICE 0x68
#define BMI270_PWR_CONF 0x7C
#define BMI270_PWR_CTRL 0x7D
#define BMI270_INIT_CTRL 0x59
#define BMI270_INIT_DATA 0x5E
#define BMI270_ACC_CONF 0x40
#define BMI270_ACC_RANGE 0x41
#define BMI270_GYRO_CONF 0x42
#define BMI270_GYRO_RANGE 0x43
#define BMI270_GYRO_START 0x12
#define BMI270_ACC_START 0x0C

enum bmi270_gyro_range {
    bmi270_gyro_range_2000,
    bmi270_gyro_range_1000,
    bmi270_gyro_range_500,
    bmi270_gyro_range_250,
    bmi270_gyro_range_125
};

enum bmi270_acc_range {
    bmi270_acc_range_2g,
    bmi270_acc_range_4g,
    bmi270_acc_range_8g,
    bmi270_acc_range_16g
};

typedef struct {
    int16_t rax;
    int16_t ray;
    int16_t raz;
} bmi270_acc_pack_t;

typedef struct {
    int16_t rgx;
    int16_t rgy;
    int16_t rgz;
} bmi270_gyro_pack_t;

class BMI270 : public EF_I2C::EF_I2C_device
{   
    public:
        BMI270(const uint8_t dev_addr);
        ~BMI270(){};

        void bmi270_get_pose(float* roll, float* pitch);
        // void Kalman_filter_init(Kalman_t kx, Kalman_t ky);
    private:
        bmi270_acc_pack_t acc_pack = {0, 0, 0};
        bmi270_gyro_pack_t gyro_pack = {0, 0, 0};

        bmi270_acc_pack_t bmi270_read_acc();
        bmi270_gyro_pack_t bmi270_read_gyro();

        void row_2_standard();

        uint8_t acc_range = bmi270_acc_range_2g;
        uint8_t gyro_range = bmi270_gyro_range_500;

        // const float gyro_trans[5] = {16.4, 32.8, 65.6, 131.2, 262.4};
        // const float acc_trans[4] = {16383.5, 8191.8, 4095.9, 2047.9};
        
        // float real_data[6] = {0};
        // float ax, ay, az, gx, gy, gz;

        // gyro: 500dps <-> (+-)2^15 - 1
        // +-dps <-> LSB/dps
        // 2000  <-> 16.4
        // 1000  <-> 32.8
        //  500  <-> 65.6
        //  250  <-> 131.2
        //  125  <-> 262.4

        // acc:
        // +-g <-> LSB/g
        // 16  <-> 2047.9
        //  8  <-> 4095.9
        //  4  <-> 8191.8
        //  2  <-> 16383.5
        
        /*
          Kalman filter
        // */  
        // Kalman k_filter;
        // Kalman_t kx, ky;
};
#endif