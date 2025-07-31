#ifndef MPU6050_H
#define MPU6050_H

#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "esp_err.h"
#include "esp_log.h"
#include "i2c_manager.h"
#include "kalman.h"

#define MPU6050_DEVICE_L 0x68
#define MPU6050_DEVICE_H 0x69

#define MPU6050_SELF_TEST_X 0x0D
#define MPU6050_SELF_TEST_Y 0x0E
#define MPU6050_SELF_TEST_Z 0x0F
#define MPU6050_SELF_TEST_A 0x10

#define MPU6050_SMPLRT_DIV 0x19

#define MPU6050_CONFIG 0x1A
#define MPU6050_GYRO_CONFIG 0x1B
#define MPU6050_ACCEL_CONFIG 0x1C

#define MPU6050_ZRMOT_DUR 0x22


#define MPU6050_FIFO_EN 0x23

#define MPU6050_I2C_MST_CTRL 0x24

#define MPU6050_I2C_SLV0_ADDR 0x25
#define MPU6050_I2C_SLV0_REG 0x26
#define MPU6050_I2C_SLV0_CTRL 0x27

#define MPU6050_I2C_SLV1_ADDR 0x28
#define MPU6050_I2C_SLV1_REG 0x29
#define MPU6050_I2C_SLV1_CTRL 0x2A

#define MPU6050_I2C_SLV2_ADDR 0x2B
#define MPU6050_I2C_SLV2_REG 0x2C
#define MPU6050_I2C_SLV2_CTRL 0x2D

#define MPU6050_I2C_SLV3_ADDR 0x2E
#define MPU6050_I2C_SLV3_REG 0x2F
#define MPU6050_I2C_SLV3_CTRL 0x30

#define MPU6050_I2C_SLV4_ADDR 0x31
#define MPU6050_I2C_SLV4_REG 0x32
#define MPU6050_I2C_SLV4_DO 0x33
#define MPU6050_I2C_SLV4_CTRL 0x34
#define MPU6050_I2C_SLV4_DI 0x35

#define MPU6050_I2C_MST_STATUS 0x36

#define MPU6050_INT_PIN_CFG 0x37
#define MPU6050_INT_ENABLE 0x38
#define MPU6050_INT_STATUS 0x3A

#define MPU6050_ACCEL_XOUT_H 0x3B
#define MPU6050_ACCEL_XOUT_L 0x3C
#define MPU6050_ACCEL_YOUT_H 0x3D
#define MPU6050_ACCEL_YOUT_L 0x3E
#define MPU6050_ACCEL_ZOUT_H 0x3F
#define MPU6050_ACCEL_ZOUT_L 0x40

#define MPU6050_TEMP_OUT_H 0x41
#define MPU6050_TEMP_OUT_L 0x42

#define MPU6050_GYRO_XOUT_H 0x43
#define MPU6050_GYRO_XOUT_L 0x44
#define MPU6050_GYRO_YOUT_H 0x45
#define MPU6050_GYRO_YOUT_L 0x46
#define MPU6050_GYRO_ZOUT_H 0x47
#define MPU6050_GYRO_ZOUT_L 0x48

#define MPU6050_EXT_SENS_DATA_00 0x49
#define MPU6050_EXT_SENS_DATA_01 0x4A
#define MPU6050_EXT_SENS_DATA_02 0x4B
#define MPU6050_EXT_SENS_DATA_03 0x4C
#define MPU6050_EXT_SENS_DATA_04 0x4D
#define MPU6050_EXT_SENS_DATA_05 0x4E
#define MPU6050_EXT_SENS_DATA_06 0x4F
#define MPU6050_EXT_SENS_DATA_07 0x50
#define MPU6050_EXT_SENS_DATA_08 0x51
#define MPU6050_EXT_SENS_DATA_09 0x52
#define MPU6050_EXT_SENS_DATA_10 0x53
#define MPU6050_EXT_SENS_DATA_11 0x54
#define MPU6050_EXT_SENS_DATA_12 0x55
#define MPU6050_EXT_SENS_DATA_13 0x56
#define MPU6050_EXT_SENS_DATA_14 0x57
#define MPU6050_EXT_SENS_DATA_15 0x58
#define MPU6050_EXT_SENS_DATA_16 0x59
#define MPU6050_EXT_SENS_DATA_17 0x5A
#define MPU6050_EXT_SENS_DATA_18 0x5B
#define MPU6050_EXT_SENS_DATA_19 0x5C
#define MPU6050_EXT_SENS_DATA_20 0x5D
#define MPU6050_EXT_SENS_DATA_21 0x5E
#define MPU6050_EXT_SENS_DATA_22 0x5F
#define MPU6050_EXT_SENS_DATA_23 0x60

#define MPU6050_I2C_SLV0_DO 0x63
#define MPU6050_I2C_SLV1_DO 0x64
#define MPU6050_I2C_SLV2_DO 0x65
#define MPU6050_I2C_SLV3_DO 0x66
#define MPU6050_I2C_MST_DELAY_CTRL 0x67

#define MPU6050_SIGNAL_PATH_RESET 0x68

#define MPU6050_MOT_DETECT_CTRL 0x69

#define MPU6050_USER_CTRL 0x6A

#define MPU6050_PWR_MGMT_1 0x6B
#define MPU6050_PWR_MGMT_2 0x6C

#define MPU6050_FIFO_COUNTH 0x72
#define MPU6050_FIFO_COUNTL 0x73
#define MPU6050_FIFO_R_W 0x74

#define MPU6050_WHO_AM_I 0x75

#define MPU6050_TEMP_DIS 0x01
#define MPU6050_TEMP_EN 0x00
#define MPU6050_TEMP_SEL 3

typedef struct {
    int16_t rax;
    int16_t ray;
    int16_t raz;
} mpu6050_accel_pack_t;

typedef struct {
    int16_t rgx;
    int16_t rgy;
    int16_t rgz;
} mpu6050_gyro_pack_t;

typedef struct {
    mpu6050_accel_pack_t accl_r;
    mpu6050_gyro_pack_t gyro_r;
    kalman_t kalman;
} mpu6050_output_t;


typedef struct {
    int16_t temp;
} mpu6050_temp_pack_t;


enum mpu6050_device_setup {
    BW_260_D_0000_BW_256_D_0098_F_8 = 0,
    BW_184_D_0020_BW_188_D_0005_F_1,
    BW_094_D_0050_BW_098_D_0013_F_1,
    BW_044_D_0098_BW_042_D_0048_F_1,
    BW_021_D_0099_BW_020_D_0099_F_1,
    BW_010_D_0099_BW_010_D_0099_F_1,
    BW_005_D_0099_BW_005_D_0099_F_1
};

enum mpu6050_gyro_setup {
    GYRO_250_DPS = 0,
    GYRO_500_DPS,
    GYRO_1000_DPS,
    GYRO_2000_DPS
};

enum mpu6050_accel_setup {
    ACCEL_2G = 0,
    ACCEL_4G,
    ACCEL_8G,
    ACCEL_16G
};

enum mpu6050_accel_hpf_setup {
    ALL_PASS = 0,
    HPF_5HZ,
    HPF_2_5HZ,
    HPF_1_25HZ,
    HPF_0_63HZ,
    DIFF_REG = 7
};

enum mpu6050_clk_setup {
    INTERNAL_8MHZ = 0,
    PLL_XGYRO,
    PLL_YGYRO,
    PLL_ZGYRO,
    PLL_EXT_32_768KHZ,
    PLL_EXT_19_2MHZ,
    STOP_CLOCK = 7
};

enum mpu6050_temp_setup {
    TEMP_DIS = 0,
    TEMP_EN = 1,
};

esp_err_t mpu6050_init(i2c_master_dev_handle_t dev_handle);

esp_err_t mpu6050_set_freq(i2c_master_dev_handle_t dev_handle, uint16_t freq);

esp_err_t mpu6050_device_setup(i2c_master_dev_handle_t dev_handle, enum mpu6050_device_setup setup);

esp_err_t mpu6050_gyro_range_setup(i2c_master_dev_handle_t dev_handle, enum mpu6050_gyro_setup setup);

esp_err_t mpu6050_gyro_read(i2c_master_dev_handle_t dev_handle, mpu6050_gyro_pack_t* gyro_pack);

esp_err_t mpu6050_accel_range_setup(i2c_master_dev_handle_t dev_handle, enum mpu6050_accel_setup setup);

esp_err_t mpu6050_accel_hpf_setup(i2c_master_dev_handle_t dev_handle, enum mpu6050_accel_hpf_setup setup);

esp_err_t mpu6050_accel_read(i2c_master_dev_handle_t dev_handle, mpu6050_accel_pack_t* accel_pack);

esp_err_t mpu6050_clk_setup(i2c_master_dev_handle_t dev_handle, enum mpu6050_clk_setup setup);

esp_err_t mpu6050_temp_setup(i2c_master_dev_handle_t dev_handle, enum mpu6050_temp_setup setup);

esp_err_t mpu6050_temp_read(i2c_master_dev_handle_t dev_handle, mpu6050_temp_pack_t* temp_pack);

esp_err_t mpu6050_get_pos(i2c_master_dev_handle_t dev_handle, mpu6050_output_t* out_put);

#endif