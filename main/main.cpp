#include "mpu6050.h"
#include "bmi270.h"
#include "bmp280.h"

#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "esp_log.h"
#include "esp_task_wdt.h"

void task(void*);

mpu6050_accel_pack_t accel_pack = {0, 0, 0};
mpu6050_gyro_pack_t gyro_pack = {0, 0, 0};
mpu6050_temp_pack_t temp_pack = {0};

extern "C" void app_main(void)
{   
    xTaskCreatePinnedToCore(task, "task", 100 * 1024, NULL, 5, NULL, 1);
}

void task(void*)
{
    // MPU6050 mpu6050(MPU6050_DEVICE_L);
    
    // BMP280 bmp280(BMP280_DEVICE);
    printf("OK0");

    BMI270 bmi270(BMI270_DEVICE);
    printf("OK1");
    Kalman_t kx = {
        .Q_angle = 0.001f,
        .Q_bias = 0.003f,
        .R_meas = 0.03f,
        .angle = 0,
        .bias = 0,
        .P = {{1, 0}, {0, 1}},
    };
    Kalman_t ky = {
        .Q_angle = 0.001f,
        .Q_bias = 0.003f,
        .R_meas = 0.03f,
        .angle = 0,
        .bias = 0,
        .P = {{1, 0}, {0, 1}},
    };
    // printf("OK2");
    // bmi270.Kalman_filter_init(kx, ky);
    printf("OK3");
    float roll = 0, pitch = 0;
    float height;
    while(1)
    {
        
        vTaskDelay(pdMS_TO_TICKS(120));
        bmi270.bmi270_get_pose(&roll, &pitch);
        ESP_LOGI("BMI270", "roll: %.4f \n pitch: %.4f \n\n", roll, pitch);
        // accel_pack = mpu6050.MPU6050_accel_read();
        // vTaskDelay(pdMS_TO_TICKS(100));
        // height = bmp280.bmp280_get_height();
        // temp_pack = mpu6050.MPU6050_temp_read();
        
        // ESP_LOGI("MPU6050", "Gyro: x = %d, y = %d, z = %d", gyro_pack.rgx, gyro_pack.rgy, gyro_pack.rgz);
        // ESP_LOGI("MPU6050", "Accel: x = %d, y = %d, z = %d", accel_pack.rax, accel_pack.ray, accel_pack.raz);
        // ESP_LOGI("MPU6050", "Temp: %d", temp_pack.temp);
        // vTaskDelay(pdMS_TO_TICKS(100));
        
        // ESP_LOGI("BMP280", "Height: %.4f", height);

    }
}