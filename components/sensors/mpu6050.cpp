#include "mpu6050.h"
#include "esp_dsp.h"


MPU6050::MPU6050(const uint8_t dev_addr): EF_I2C::EF_I2C_device(dev_addr)
{
    uint8_t data;
    this->EF_I2C_read(MPU6050_PWR_MGMT_1, &data, 1);
    
    data &= 0x08;
    data |= static_cast<uint8_t>(INTERNAL_8MHZ & 0x07);
    this->EF_I2C_write(MPU6050_PWR_MGMT_1, &data, 1);

    // ESP_LOGI("MPU6050", "initialized.");
    
    this->MPU6050_set_freq(8000);

    this->MPU6050_device_setup(BW_260_D_0000_BW_256_D_0098_F_8);

    this->MPU6050_gyro_range_setup(GYRO_1000_DPS);

    this->MPU6050_accel_hpf_setup(HPF_0_63HZ);

    this->MPU6050_accel_range_setup(ACCEL_8G);

}

void MPU6050::MPU6050_set_freq(uint16_t freq)
{
    if (freq > 8000) return;

    uint8_t data = (8000 / freq) - 1;
    
    this->EF_I2C_write(MPU6050_SMPLRT_DIV, &data, 1);
}

void MPU6050::MPU6050_device_setup(enum mpu6050_device_setup setup)
{
    uint8_t data = static_cast<uint8_t>(setup & 0x07);

    this->EF_I2C_write(MPU6050_CONFIG, &data, 1);
}

void MPU6050::MPU6050_gyro_range_setup(enum mpu6050_gyro_setup setup)
{
    uint8_t data = static_cast<uint8_t>((setup & 0x03) << 3);

    this->EF_I2C_write(MPU6050_ACCEL_CONFIG, &data, 1);
}

void MPU6050::MPU6050_accel_hpf_setup(enum mpu6050_accel_hpf_setup setup)
{
    uint8_t data;

    this->EF_I2C_read(MPU6050_ACCEL_CONFIG, &data, 1);

    data &= 0x18;
    data |= static_cast<uint8_t>(setup & 0x07);

    this->EF_I2C_write(MPU6050_ACCEL_CONFIG, &data, 1);
}

void MPU6050::MPU6050_accel_range_setup(enum mpu6050_accel_setup setup)
{
    uint8_t data;

    this->EF_I2C_read(MPU6050_ACCEL_CONFIG, &data, 1);

    data &= 0x07;
    data |= static_cast<uint8_t>((setup & 0x03) << 3);

    this->EF_I2C_write(MPU6050_ACCEL_CONFIG, &data, 1);
}

mpu6050_accel_pack_t MPU6050::MPU6050_accel_read(void)
{
    uint8_t data_pack[6];
    this->EF_I2C_read(MPU6050_ACCEL_XOUT_H, data_pack, 6);

    this->accel_pack.rax = (data_pack[0] << 8) | data_pack[1];
    this->accel_pack.ray = (data_pack[2] << 8) | data_pack[3];
    this->accel_pack.raz = (data_pack[4] << 8) | data_pack[5];

    return this->accel_pack;
}

void MPU6050::MPU6050_clk_setup(enum mpu6050_clk_setup setup)
{
    uint8_t data;
    this->EF_I2C_read(MPU6050_PWR_MGMT_1, &data, 1);

    data &= 0x08;
    data |= static_cast<uint8_t>(setup & 0x07);

    this->EF_I2C_write(MPU6050_PWR_MGMT_1, &data, 1);
}

void MPU6050::MPU6050_temp_setup(enum mpu6050_temp_setup setup)
{
    uint8_t data;
    this->EF_I2C_read(MPU6050_PWR_MGMT_1, &data, 1);

    data &= 0x07;
    data |= static_cast<uint8_t>((setup & 0x01) << 3);

    this->EF_I2C_write(MPU6050_PWR_MGMT_1, &data, 1);
}

mpu6050_temp_pack_t MPU6050::MPU6050_temp_read(void)
{
    uint8_t data_pack[2];
    this->EF_I2C_read(MPU6050_TEMP_OUT_H, data_pack, 2);

    this->temp_pack.temp = (data_pack[0] << 8) | data_pack[1];

    return this->temp_pack;
}

mpu6050_gyro_pack_t MPU6050::MPU6050_gyro_read(void)
{
    uint8_t data_pack[6];
    this->EF_I2C_read(MPU6050_ACCEL_XOUT_H, data_pack, 6);

    this->gyro_pack.rgx = (data_pack[0] << 8) | data_pack[1];
    this->gyro_pack.rgy = (data_pack[2] << 8) | data_pack[3];
    this->gyro_pack.rgz = (data_pack[4] << 8) | data_pack[5];

    return this->gyro_pack;
}

void MPU6050::MPU6050_Kalman_Filter(void)
{
    
}
