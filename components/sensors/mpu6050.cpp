#include "mpu6050.h"

MPU6050::MPU6050(const uint8_t dev_addr, EF_I2C::EF_I2C_bus* i2c_bus): EF_I2C::EF_I2C_device(dev_addr, i2c_bus)
{
    uint8_t data;
    this->EF_I2C_read(MPU6050_PWR_MGMT_1, &data, 1);
    
    data &= 0x08;
    data |= (uint8_t)(INTERNAL_8MHZ & 0x07);
    this->EF_I2C_write(MPU6050_PWR_MGMT_1, &data, 1);

    // ESP_LOGI("MPU6050", "initialized.");
    
    this->MPU6050_set_freq(8000);

    mpu6050_device_setup(dev_handle, BW_260_D_0000_BW_256_D_0098_F_8);

    mpu6050_gyro_range_setup(dev_handle, GYRO_1000_DPS);

    mpu6050_accel_hpf_setup(dev_handle, HPF_0_63HZ);

    mpu6050_accel_range_setup(dev_handle, ACCEL_8G);

}

void MPU6050::MPU6050_set_freq(uint16_t freq)
{
    if (freq > 8000) return;

    uint8_t data = (8000 / freq) - 1;
    
    this->EF_I2C_write(MPU6050_SMPLRT_DIV, &data, 1);
}