#include "driver/gpio.h"
#include "quaternion_utils.h"
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "i2c_manager.h"
#include "bmi270.h"
#include "bmp280.h"
#include "mahony.h"
#include "sensor_data_types.h"
#include "time_manager.h"

#define DEBUG

esp_err_t ret;

i2c_master_bus_config_t i2c_master_bus_config = {
    .clk_source = I2C_CLK_SRC_DEFAULT,
    .i2c_port = i2c_internal_port,
    .scl_io_num = I2C_INTERNAL_SCL,
    .sda_io_num = I2C_INTERNAL_SDA,
    .glitch_ignore_cnt = 7,
    .flags.enable_internal_pullup = true
};

i2c_master_bus_handle_t i2c_bus_handle;

void bmi270_task(void* param);
void bmp280_task(void* param);

void app_main(void)
{   
    i2c_device_config_t bmi270_cfg = {
        .dev_addr_length = I2C_ADDR_BIT_LEN_7,
        .device_address = BMI270_DEVICE,
        .scl_speed_hz = I2C_CLK_FREQ,
    };
    i2c_master_dev_handle_t bmi270_handle;

    i2c_device_config_t bmp280_cfg = {
        .dev_addr_length = I2C_ADDR_BIT_LEN_7,
        .device_address = BMI270_DEVICE,
        .scl_speed_hz = I2C_CLK_FREQ,
    };
    i2c_master_dev_handle_t bmp280_handle;

    EF_ERR_CHECK(i2c_new_master_bus(&i2c_master_bus_config, &i2c_bus_handle), "main");
    ESP_LOGI("EspFlight", "I2C bus created successfully");

    // i2c_master_bus_add_device(i2c_bus_handle, &bmp180_cfg, &bmp180_handle);
    EF_ERR_CHECK(i2c_master_bus_add_device(i2c_bus_handle, &bmi270_cfg, &bmi270_handle), "main");
    EF_ERR_CHECK(i2c_master_bus_add_device(i2c_bus_handle, &bmp280_cfg, &bmp280_handle), "main");
    // mpu6050_init(mpu6050_handle);

    vTaskDelay(pdMS_TO_TICKS(1000));
    
    xTaskCreatePinnedToCore(bmi270_task, "BMI270", 10240, (void*)bmi270_handle, 4, NULL, 0);
    xTaskCreatePinnedToCore(bmp280_task, "BMI280", 10240, (void*)bmp280_handle, 5, NULL, 0);
    vTaskDelay(pdMS_TO_TICKS(100));
    // xTaskCreatePinnedToCore(bmp180_task, "BMP180", 10240, (void*)bmp180_handle, 5, NULL, 0);

    ESP_LOGI("EspFlight", "Initialize I2C");
}


void bmi270_task(void* param)
{
    i2c_master_dev_handle_t bmi270_handle = (i2c_master_dev_handle_t) param;
    ESP_LOGI("LOG1" , "handle got!");
    acc_pack_t acc_pack;
    gyro_pack_t gyro_pack;
    temp_pack_t temp_pack;
    atti_pack_t atti_pack;
    int64_t timer;
    float dt;
    quat_t init_quat = quat_wxyz(1.f, .0f, .0f , .0f);
    mahony_params_t mahony_params = {
        .Kp = 1.f,
        .Ki = 1e-3f,
        .rotation = init_quat,
    };
    bmi270_init(bmi270_handle, bmi270_acc_range_8g, bmi270_gyro_range_2000);
    timer = esp_timer_get_time();
    while(1)
    {
        vTaskDelay(pdMS_TO_TICKS(2)); /* 500 hz*/
        bmi270_read_acc(bmi270_handle, &acc_pack, bmi270_acc_range_8g);
        bmi270_read_gyro(bmi270_handle, &gyro_pack, bmi270_gyro_range_2000);
        bmi270_read_temp(bmi270_handle, &temp_pack);
        // ESP_LOGI("BMI270", "ax: %.2f, ay: %.2f, az: %.2f", acc_pack.ax, acc_pack.ay, acc_pack.az);
        // ESP_LOGI("BMI270", "gx: %.2f, gy: %.2f, gz: %.2f", gyro_pack.gx, gyro_pack.gy, gyro_pack.gz);
        dt = dt_s(&timer);
        mahony_get_deg(acc_pack, gyro_pack, &mahony_params, &atti_pack, dt);
        // ESP_LOGI("BMI270", "roll: %.2f°, pitch: %.2f°, yaw: %.2f°", atti_pack.roll, atti_pack.pitch, atti_pack.yaw);
    }
}


void bmp280_task(void* param)
{
    i2c_master_dev_handle_t bmp280_handle = (i2c_master_dev_handle_t) param;
    bmp_colab_params_t colab_params;
    baro_pack_t baro_pack;
    bmp280_init(bmp280_handle, &colab_params);
    float height;
    while(1)
    {   
        // printf("asd"); 
        bmp280_read_temp(bmp280_handle, &baro_pack, &colab_params);
        vTaskDelay(pdMS_TO_TICKS(10));
        // printf("xcv");
        bmp280_read_press(bmp280_handle, &baro_pack, &colab_params);
        vTaskDelay(pdMS_TO_TICKS(10));
        // printf("vbn");
        height = bmp280_get_height(&baro_pack);
        ESP_LOGI("BMP280", "height: %.2fm, temperature: %.2f℃", height, baro_pack.temp / 100.0f);
        vTaskDelay(pdMS_TO_TICKS(10));
    }

}