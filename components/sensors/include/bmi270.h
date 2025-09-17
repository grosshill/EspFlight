#ifndef BMI270_H
#define BMI270_H
#include "i2c_manager.h"
#include "sensor_data_type.h"

#define BMI270_CHIP_ID 0x00
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
#define BMI270_INTERNAL_STATUS 0x21
#define BMI270_TEMP_START 0x22

#define BMI270_TAG "BMI270"

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

esp_err_t bmi270_init(i2c_master_dev_handle_t dev_handle, enum bmi270_acc_range acc_range, enum bmi270_gyro_range gyro_range);

esp_err_t bmi270_read_gyro(i2c_master_dev_handle_t dev_handle, gyro_pack_t* gyro_pack, enum bmi270_gyro_range gyro_range);

esp_err_t bmi270_read_acc(i2c_master_dev_handle_t dev_handle, acc_pack_t* acc_pack, enum bmi270_acc_range acc_range);

esp_err_t bmi270_read_temp(i2c_master_dev_handle_t dev_handle, temp_pack_t* temp_pack);

// esp_err_t bmi270_get_pose(bmi270_gyro_pack_t gyro_pack, bmi270_acc_pack_t acc_pack, kalman_t* kalman, kalman_est_angle_t* est_angle);
#endif