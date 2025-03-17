#ifndef DRIVERS_H
#define DRIVERS_H

#include <cstdio>
#include "driver/i2c_master.h"
#include "driver/gpio.h"
#include "string.h"
#include "freertos/FreeRTOS.h"

namespace EF_I2C
{

#define I2C_INTERNAL_SCL GPIO_NUM_21
#define I2C_INTERNAL_SDA GPIO_NUM_22
#define I2C_TIMEOUT pdMS_TO_TICKS(100)
#define I2C_CLK_FREQ 400000

#ifdef __cplusplus
extern "C"{
#endif

#ifdef __cplusplus
}
#endif

enum i2c_port_id {
    i2c_internal_port = 0,
    i2c_external_port
};

class EF_I2C_bus
{
    public:
        static EF_I2C_bus& instance()
        {
            static EF_I2C_bus instance;
            return instance;
        }


        EF_I2C_bus(const EF_I2C_bus&) = delete;
        // EF_I2C_bus& operator=(const EF_I2C_bus&) = delete;
        
        i2c_master_bus_handle_t get_bus_handle()
        {
            return EF_I2C_bus_handle;
        }
        
        static void initialize() {
            instance();
        }

    private:
        i2c_master_bus_config_t EF_I2C_bus_config;
        i2c_master_bus_handle_t EF_I2C_bus_handle;

        EF_I2C_bus()
        {
            EF_I2C_bus_config.clk_source = I2C_CLK_SRC_DEFAULT;
            EF_I2C_bus_config.i2c_port = i2c_internal_port;
            EF_I2C_bus_config.scl_io_num = I2C_INTERNAL_SCL;
            EF_I2C_bus_config.sda_io_num = I2C_INTERNAL_SDA;
            EF_I2C_bus_config.glitch_ignore_cnt = 7;
            EF_I2C_bus_config.flags.enable_internal_pullup = true;

            i2c_new_master_bus(&EF_I2C_bus_config, &EF_I2C_bus_handle);
        }

        ~EF_I2C_bus() {
            if (EF_I2C_bus_handle) {
                i2c_del_master_bus(EF_I2C_bus_handle);
                EF_I2C_bus_handle = NULL;
            }
        }
};

class EF_I2C_device
{
    public:
        EF_I2C_device(const uint8_t dev_addr);
        
        void EF_I2C_write(const uint8_t reg_addr, const uint8_t* data_word, const uint8_t data_length);

        void EF_I2C_read(const uint8_t reg_addr, uint8_t *data, const uint8_t data_length);

        i2c_master_dev_handle_t get_dev_handle() const {
            return EF_I2C_device_handle;
        }

    private:
        i2c_device_config_t EF_I2C_device_config;
        i2c_master_dev_handle_t EF_I2C_device_handle;
        EF_I2C_bus* EF_I2C_bus_dev;
};

}

namespace EF_SDIO
{

class EF_SDIO
{
    public:
        const char* name = "SDIO";
};

}

#endif