#include <iostream>
#include "drivers.h"
#include "freertos/FreeRTOS.h"

extern "C" void app_main(void)
{
    EF_I2C::EF_I2C_bus* i2c_bus = EF_I2C::EF_I2C_bus::get_instance();
}