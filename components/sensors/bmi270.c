#include "bmi270.h"
#include "bmi270_config.h"
#include "debug_utils.h"
#include "esp_log.h"
#include "esp_err.h"


const float bmi270_gyro_trans[5] = {16.4, 32.8, 65.6, 131.2, 262.4};
const float bmi270_acc_trans[4] = {16383.5, 8191.8, 4095.9, 2047.9};
uint64_t bmi270_us_timer = 0;

esp_err_t bmi270_init(i2c_master_dev_handle_t dev_handle, enum bmi270_acc_range acc_range, enum bmi270_gyro_range gyro_range)
{
    /*
      Implement initialization for BMI270, according to the datasheet.
    */
    vTaskDelay(pdMS_TO_TICKS(10));
    uint8_t data = 0x00;
    EF_ERR_CHECK(i2c_read(dev_handle, BMI270_CHIP_ID, &data, 1), BMI270_TAG);
    
    if(data != 0x24)
    {
      ESP_LOGI(BMI270_TAG, "BMI270 I2C communication error.");
    }

    // ESP_LOGI("BMI270", "1");
    data = 0x00;
    EF_ERR_CHECK(i2c_write(dev_handle, BMI270_PWR_CONF, &data, 1), BMI270_TAG);
    // ESP_LOGI("BMI270", "2");
    vTaskDelay(pdMS_TO_TICKS(10));
    EF_ERR_CHECK(i2c_write(dev_handle, BMI270_INIT_CTRL, &data, 1), BMI270_TAG);
    // ESP_LOGI("BMI270", "3");
    // vTaskDelay(pdMS_TO_TICKS(1));
    EF_ERR_CHECK(i2c_write(dev_handle, BMI270_INIT_DATA, bmi270_config_file, sizeof(bmi270_config_file)), BMI270_TAG);
    // ESP_LOGI("BMI270", "4");
    data = 0x01;
    EF_ERR_CHECK(i2c_write(dev_handle, BMI270_INIT_CTRL, &data, 1), BMI270_TAG);

    /*
      Check initialization state.
    */
    vTaskDelay(pdMS_TO_TICKS(10));
    EF_ERR_CHECK(i2c_read(dev_handle, BMI270_INTERNAL_STATUS, &data, 1), BMI270_TAG);
    if((data & 0x01) != 1)
    {
      ESP_LOGI(BMI270_TAG, "BMI270 initialization error.");
    }

    /*
      Select performance mode.
    */

    data = 0x0E;
    EF_ERR_CHECK(i2c_write(dev_handle, BMI270_PWR_CTRL, &data, 1), BMI270_TAG);
    data = 0xA8;
    EF_ERR_CHECK(i2c_write(dev_handle, BMI270_ACC_CONF, &data, 1), BMI270_TAG);
    data = 0xE9;
    EF_ERR_CHECK(i2c_write(dev_handle, BMI270_GYRO_CONF, &data, 1), BMI270_TAG);
    data = 0x02;
    EF_ERR_CHECK(i2c_write(dev_handle, BMI270_PWR_CONF, &data, 1), BMI270_TAG);

    /*
      Select the range.
    */

    data = 0x00 | acc_range;
    EF_ERR_CHECK(i2c_write(dev_handle, BMI270_ACC_RANGE, &data, 1), BMI270_TAG);
    data = 0x00 | gyro_range;
    EF_ERR_CHECK(i2c_write(dev_handle, BMI270_GYRO_RANGE, &data, 1), BMI270_TAG);

    return ESP_OK;
}

// void BMI270::Kalman_filter_init(Kalman_t kx, Kalman_t ky)
// {
//     k_filter.Kalman_init(kx, ky);
// }

esp_err_t bmi270_read_gyro(i2c_master_dev_handle_t dev_handle, bmi270_gyro_pack_t* gyro_pack, enum bmi270_gyro_range gyro_range)
{
    uint8_t data[6];
    EF_ERR_CHECK(i2c_read(dev_handle, BMI270_GYRO_START, data, 6), BMI270_TAG);
    
    int16_t rgx, rgy, rgz;
    double g_factor = bmi270_gyro_trans[gyro_range];

    rgx = (data[1] << 8) | data[0];
    rgy = (data[3] << 8) | data[2];
    rgz = (data[5] << 8) | data[4];

    gyro_pack->gx = rgx / g_factor;
    gyro_pack->gy = rgy / g_factor;
    gyro_pack->gz = rgz / g_factor;

    // ESP_LOGI("BMI270", "rgx: %d, rgy: %d, rgz: %d", gyro_pack->rgx, gyro_pack->rgx, gyro_pack->rgz);

    return ESP_OK;
}

esp_err_t bmi270_read_acc(i2c_master_dev_handle_t dev_handle, bmi270_acc_pack_t* acc_pack, enum bmi270_acc_range acc_range)
{
    uint8_t data[6];
    EF_ERR_CHECK(i2c_read(dev_handle, BMI270_ACC_START, data, 6), BMI270_TAG);

    int16_t rax, ray, raz;
    double a_factor = bmi270_acc_trans[acc_range];

    rax = (data[1] << 8) | data[0];
    ray = (data[3] << 8) | data[2];
    raz = (data[5] << 8) | data[4];

    acc_pack->ax = rax / a_factor;
    acc_pack->ay = ray / a_factor;
    acc_pack->az = raz / a_factor;

    // ESP_LOGI("BMI270", "rax: %d, ray: %d, raz: %d", acc_pack->rax, acc_pack->ray, acc_pack->raz);

    return ESP_OK;
}

esp_err_t bmi270_read_temp(i2c_master_dev_handle_t dev_handle, bmi270_temp_pack_t* temp_pack)
{
    uint8_t data[2];
    EF_ERR_CHECK(i2c_read(dev_handle, BMI270_TEMP_START, data, 2), BMI270_TAG);

    temp_pack->temp = 23.0 + (double)((data[1] << 8) | data[0]) / 512.0;

    // ESP_LOGI("BMI270", "temperature: %.2f", temp_pack->temp);

    return ESP_OK;
}

/*
bmi270_acc_pack_t BMI270::bmi270_read_acc()
{
    uint8_t data[6];
    EF_I2C_read(BMI270_ACC_START, data, 6);

    acc_pack.rax = (data[1] << 8) | data[0];
    acc_pack.ray = (data[3] << 8) | data[2];
    acc_pack.raz = (data[5] << 8) | data[4];

    // ESP_LOGI("BMI270", "rax: %d, ray: %d, raz: %d", acc_pack.rax, acc_pack.ray, acc_pack.raz);

    return acc_pack;
}

bmi270_gyro_pack_t BMI270::bmi270_read_gyro()
{
    uint8_t data[6];
    EF_I2C_read(BMI270_GYRO_START, data, 6);

    gyro_pack.rgx = (data[1] << 8) | data[0];
    gyro_pack.rgy = (data[3] << 8) | data[2];
    gyro_pack.rgz = (data[5] << 8) | data[4];

    // ESP_LOGI("BMI270", "rgx: %d, rgy: %d, rgz: %d", gyro_pack.rgx, gyro_pack.rgx, gyro_pack.rgz);

    return gyro_pack;
}

void BMI270::bmi270_get_pose(float* roll, float* pitch)
{
    bmi270_read_acc();
    bmi270_read_gyro();
    // row_2_standard();
    // k_filter.Kalman_cal(ax, ay, az, gx, gy, gz);

    // *roll = k_filter.out_x;
    // *pitch = k_filter.out_y;
}
*/
// void BMI270::row_2_standard()
// {
//     float a_factor = acc_trans[acc_range], g_factor = gyro_trans[gyro_range];

//     ax = acc_pack.rax / a_factor;
//     ay = acc_pack.ray / a_factor;
//     az = acc_pack.raz / a_factor;
//     gx = gyro_pack.rgx / g_factor;
//     gy = gyro_pack.rgy / g_factor;
//     gz = gyro_pack.rgz / g_factor;
//     ESP_LOGI("BMI270", "ax: %.4f, ay, %.4f, az, %.4f", ax, ay, az);
//     ESP_LOGI("BMI270", "gx: %.4f, gy, %.4f, gz, %.4f", gx, gy, gz);
// }