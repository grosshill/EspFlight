#include "drivers.h"

EF_I2C::EF_I2C_device::EF_I2C_device(const uint8_t dev_addr, EF_I2C_bus* i2c_bus)
{
    EF_I2C_device_config.dev_addr_length = I2C_ADDR_BIT_LEN_7;
    EF_I2C_device_config.device_address = dev_addr;
    EF_I2C_device_config.scl_speed_hz = I2C_CLK_FREQ;
    EF_I2C_bus_dev = i2c_bus;
    i2c_master_bus_add_device(EF_I2C_bus_dev->get_bus_handle(), &EF_I2C_device_config, &EF_I2C_device_handle);
}

void EF_I2C::EF_I2C_device::EF_I2C_write(const uint8_t reg_addr, const uint8_t* data_word, const uint8_t data_length)
{    
    uint8_t write_buffer[data_length + 1];
    write_buffer[0] = reg_addr;
    memcpy(&write_buffer[1], data_word, data_length);

    i2c_master_transmit(this->EF_I2C_device_handle, write_buffer, sizeof(uint8_t) * (data_length + 1), I2C_TIMEOUT);            
}

void EF_I2C::EF_I2C_device::EF_I2C_read(const uint8_t reg_addr, uint8_t *data, const uint8_t data_length)
{
    i2c_master_transmit_receive(this->EF_I2C_device_handle, &reg_addr, sizeof(reg_addr), data, data_length * sizeof(uint8_t), I2C_TIMEOUT);
}