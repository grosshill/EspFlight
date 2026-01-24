#include "bmp280.h"

esp_err_t bmp280_init(i2c_master_dev_handle_t dev_handle, bmp_colab_params_t* params)
{
    uint8_t data = 0;

    /*
      test
    */
    // i2c_read(dev_handle, 0x00, &data, 1);
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

    data = (uint8_t)(0x00 | BMP280_PWR_ON_NORMAL | p_overs_16 << 2 | t_overs_2 << 5);
    EF_ERR_CHECK(i2c_write(dev_handle, BMP280_CTRL_MEAS, &data, 1), BMP280_TAG);

    data = (uint8_t)(0x00 | iir_coeff_16 << 2 | t_standby_0_5ms << 5);
    EF_ERR_CHECK(i2c_write(dev_handle, BMP280_CONFIG, &data, 1), BMP280_TAG);

    bmp280_get_colab_params(dev_handle, params);

    return ESP_OK;
}

/* called automatically by function bmp280_init */
esp_err_t bmp280_get_colab_params(i2c_master_dev_handle_t dev_handle, bmp_colab_params_t* params)
{
    uint8_t data[24];
    EF_ERR_CHECK(i2c_read(dev_handle, BMP280_COLAB_START_REG, data, 24), BMP280_TAG);

    /*
      Check to the datasheet of BMP280, Table 17
    */

    params->T1 = (uint16_t)(data[0] | (uint16_t)data[1] << 8);
    params->T2 = (int16_t)(data[2]  | (uint16_t)data[3] << 8);
    params->T3 = (int16_t)(data[4]  | (uint16_t)data[5] << 8);
    params->P1 = (uint16_t)(data[6] | (uint16_t)data[7] << 8);
    params->P2 = (int16_t)(data[8]  | (uint16_t)data[9] << 8);
    params->P3 = (int16_t)(data[10] | (uint16_t)data[11] << 8);
    params->P4 = (int16_t)(data[12] | (uint16_t)data[13] << 8);
    params->P5 = (int16_t)(data[14] | (uint16_t)data[15] << 8);
    params->P6 = (int16_t)(data[16] | (uint16_t)data[17] << 8);
    params->P7 = (int16_t)(data[18] | (uint16_t)data[19] << 8);
    params->P8 = (int16_t)(data[20] | (uint16_t)data[21] << 8);
    params->P9 = (int16_t)(data[22] | (uint16_t)data[23] << 8);
    
    return ESP_OK;
}

esp_err_t bmp280_read_temp(i2c_master_dev_handle_t dev_handle, baro_pack_t* baro_pack, const bmp_colab_params_t* params)
{
    uint8_t data[3];
    EF_ERR_CHECK(i2c_read(dev_handle, BMP280_TEMP_REG, data, 3), BMP280_TAG);
    // ESP_LOGI("BMP280", "row temp %d, %d, %d", data[0], data[1], data[2]);
    baro_pack->temp = (int32_t)(data[0] << 12 | data[1] << 4 | (data[2] >> 4 & 0x0F));
    // ESP_LOGI("BMP280", "row temp %ld", baro_pack->temp);
    /*
      This colabration formula is from the datasheet of BMP280, section 3.11.3
    */

    int32_t var1, var2;
    var1 = (((baro_pack->temp >> 3) - ((int32_t)(params->T1) << 1)) * (int32_t)(params->T2)) >> 11;
    var2 = (((((baro_pack->temp >> 4) - (int32_t)(params->T1)) * ((baro_pack->temp >> 4) - (int32_t)(params->T1))) >> 12)
            * (int32_t)(params->T3)) >> 14;
    baro_pack->t_fine = var1 + var2;
    baro_pack->temp = (baro_pack->t_fine * 5 + 128) >> 8;
    // ESP_LOGI("BMP280", "calculated temp %ld", baro_pack->temp);
    // ESP_LOGI("BMP280", "t_fine %ld", baro_pack->t_fine);
    /*
      Here temp value is in ℃ / 100, which means we need to devide it by 100 to get the value in ℃.
    */

    return ESP_OK;
}

esp_err_t bmp280_read_press(i2c_master_dev_handle_t dev_handle, baro_pack_t* baro_pack, const bmp_colab_params_t* params)
{
    uint8_t data[3];
    EF_ERR_CHECK(i2c_read(dev_handle, BMP280_PRESS_REG, data, 3), BMP280_TAG);
    baro_pack->press = (int32_t)(data[0] << 12 | data[1] << 4 | (data[2] >> 4 & 0x0F));

    /*
      This colabration formula is from the datasheet of BMP280, section 3.11.3
    */

    int64_t var1, var2, p;
    var1 = (int64_t)(baro_pack->t_fine) - 128000;
    var2 = var1 * var1 * (int64_t)(params->P6);
    var2 = var2 + ((var1 * (int64_t)(params->P5)) << 17);
    var2 = var2 + ((int64_t)(params->P4) << 35);
    var1 = ((var1 * var1 * (int64_t)(params->P3)) >> 8) + ((var1 * (int64_t)(params->P2)) << 12);
    var1 = ((((int64_t)(1) << 47) + var1) * ((int64_t)(params->P1))) >> 33;
    if(var1 == 0)
    {
        return 0;
    }
    p = 1048576 - baro_pack->press;
    p = (((p << 31) - var2) * 3125) / var1;
    var1 = ((int64_t)(params->P9) * (p >> 13) * (p >> 13)) >> 25;
    var2 = ((int64_t)(params->P8) * p) >> 19;
    p = ((p + var1 + var2) >> 8) + ((int64_t)(params->P7) << 4);
    baro_pack->pressure = (uint32_t)p;
    /*
      Here the return value is in Pa / 256, which means we need to devide it by 256 to get the value in Pa.
    */

    return ESP_OK;
}

float bmp280_get_height(const baro_pack_t* baro_pack)
{
    // float temp = bmp280_read_temp() / 100.0f;
    // float press = bmp280_read_press() / 256.0f / 100.0f;
    // ESP_LOGI("BMP280", "Temp: %.2f, Press: %.2f", baro_pack->temp / 100.0f, baro_pack->pressure / 256.0f / 100.0f);
    // return ((pow((1013.25 / (baro_pack->pressure / 256.0f / 100.0f)), 0.19026) - 1) * (baro_pack->temp / 100.0f + 273.15)) / 0.0065f;
    return ((pow((1028.25 / (baro_pack->pressure / 256.0f / 100.0f)), 0.19026) - 1) * (baro_pack->temp / 100.0f + 273.15)) / 0.0065f;
}