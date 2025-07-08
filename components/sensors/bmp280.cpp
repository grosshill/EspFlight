#include "bmp280.h"
#include "math.h"
BMP280::BMP280(const uint8_t dev_addr): EF_I2C::EF_I2C_device(dev_addr)
{
    uint8_t data = 0;

    /*
      test
    */
    // EF_I2C_read(0x00, &data, 1);
    // printf("%d", data);
    // assert(data == 0x24);
    /*
      According to the datasheet of BMP280, Table 13, when implementing indoor navigation,
      we shall use:
      - Power mode: Normal
      - Oversampling setting: Ultra high resolution
      - Pressure oversampling coefficient: 16
      - Temperture oversampling coefficient: 2
      - IIR filter coefficient: 16
      - Timing standby: 0.5ms
    */

    data = static_cast<uint8_t>(0x00 | BMP280_PWR_ON_NORMAL | p_overs_16 << 2 | t_overs_2 << 5);
    EF_I2C_write(BMP280_CTRL_MEAS, &data, 1);

    data = static_cast<uint8_t>(0x00 | iir_coeff_16 << 2 | t_standby_0_5ms << 5);
    EF_I2C_write(BMP280_CONFIG, &data, 1);

    bmp280_get_colab_params();
}

void BMP280::bmp280_get_colab_params()
{
    uint8_t data[24];
    EF_I2C_read(BMP280_COLAB_START_REG, data, 24);

    /*
      Check to the datasheet of BMP280, Table 17
    */

    colab_params.T1 = static_cast<unsigned short>(data[0] | data[1] << 8);
    colab_params.T2 = static_cast<signed short>(data[2] | data[3] << 8);
    colab_params.T3 = static_cast<signed short>(data[4] | data[5] << 8);
    colab_params.P1 = static_cast<unsigned short>(data[6] | data[7] << 8);
    colab_params.P2 = static_cast<signed short>(data[8] | data[9] << 8);
    colab_params.P3 = static_cast<signed short>(data[10] | data[11] << 8);
    colab_params.P4 = static_cast<signed short>(data[12] | data[13] << 8);
    colab_params.P5 = static_cast<signed short>(data[14] | data[15] << 8);
    colab_params.P6 = static_cast<signed short>(data[16] | data[17] << 8);
    colab_params.P7 = static_cast<signed short>(data[18] | data[19] << 8);
    colab_params.P8 = static_cast<signed short>(data[20] | data[21] << 8);
    colab_params.P9 = static_cast<signed short>(data[22] | data[23] << 8);
}

int32_t BMP280::bmp280_read_temp()
{
    uint8_t data[3];
    int32_t temp;
    EF_I2C_read(BMP280_TEMP_REG, data, 3);
    temp = static_cast<int32_t>(data[0] << 12 | data[1] << 4 | (data[2] >> 4 & 0x0F));

    /*
      This colabration formula is from the datasheet of BMP280, section 3.11.3
    */

    int32_t var1, var2;
    var1 = (((temp >> 3) - (static_cast<int32_t>(colab_params.T1) << 1)) * static_cast<int32_t>(colab_params.T2)) >> 11;
    var2 = (((((temp >> 4) - static_cast<int32_t>(colab_params.T1)) * ((temp >> 4) - static_cast<int32_t>(colab_params.T1))) >> 12)
            * static_cast<int32_t>(colab_params.T3)) >> 14;
    t_fine = var1 + var2;
    temp = (t_fine * 5 + 128) >> 8;

    /*
      Here the return value is in ℃ / 100, which means we need to devide it by 100 to get the value in ℃.
    */

    return temp;
}

uint32_t BMP280::bmp280_read_press()
{
    uint8_t data[3];
    int32_t press;
    EF_I2C_read(BMP280_PRESS_REG, data, 3);
    press = static_cast<int32_t>(data[0] << 12 | data[1] << 4 | (data[2] >> 4 & 0x0F));

    /*
      This colabration formula is from the datasheet of BMP280, section 3.11.3
    */

    int64_t var1, var2, p;
    var1 = static_cast<int64_t>(t_fine) - 128000;
    var2 = var1 * var1 * static_cast<int64_t>(colab_params.P6);
    var2 = var2 + ((var1 * static_cast<int64_t>(colab_params.P5)) << 17);
    var2 = var2 + (static_cast<int64_t>(colab_params.P4) << 35);
    var1 = ((var1 * var1 * static_cast<int64_t>(colab_params.P3)) >> 8) + ((var1 * static_cast<int64_t>(colab_params.P2)) << 12);
    var1 = (((static_cast<int64_t>(1) << 47) + var1) * (static_cast<int64_t>(colab_params.P1))) >> 33;
    if(var1 == 0)
    {
        return 0;
    }
    p = 1048576 - press;
    p = (((p << 31) - var2) * 3125) / var1;
    var1 = (static_cast<int64_t>(colab_params.P9) * (p >> 13) * (p >> 13)) >> 25;
    var2 = (static_cast<int64_t>(colab_params.P8) * p) >> 19;
    p = ((p + var1 + var2) >> 8) + (static_cast<int64_t>(colab_params.P7) << 4);

    /*
      Here the return value is in Pa / 256, which means we need to devide it by 256 to get the value in Pa.
    */

    return static_cast<uint32_t>(p);
}

float BMP280::bmp280_get_height()
{
    float temp = bmp280_read_temp() / 100.0f;
    float press = bmp280_read_press() / 256.0f / 100.0f;
    // ESP_LOGI("BMP280", "Temp: %.2f, Press: %.2f", temp, press);
    return ((pow((1013.25 / press), 0.19026) - 1) * (temp + 273.15)) / 0.0065f;
}