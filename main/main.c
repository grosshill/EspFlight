#include "controller.h"
#include "driver/gpio.h"
#include "esp_log.h"
#include "esp_timer.h"
#include "quaternion_utils.h"
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "freertos/queue.h"
#include "i2c_manager.h"
#include "bmi270.h"
#include "bmp280.h"
#include "mahony.h"
#include "sensor_data_types.h"
#include "time_manager.h"
#include "motor_pwm.h"
#include <stdint.h>
#include <stdlib.h>
#include <math.h>
#include "matrix_utils.h"

#define DEBUG

esp_err_t ret;

typedef struct {
    float t;
    float r;
    float p;
    float y;
} ctrl_output_t;

static QueueHandle_t ctrl_output_q = NULL;

i2c_master_bus_config_t i2c_master_bus_config = {
    .clk_source = I2C_CLK_SRC_DEFAULT,
    .i2c_port = i2c_internal_port,
    // .scl_io_num = I2C_INTERNAL_SCL,
    // .sda_io_num = I2C_INTERNAL_SDA,
    .scl_io_num = I2C_ONBOARD_SCL,
    .sda_io_num = I2C_ONBOARD_SDA,
    .glitch_ignore_cnt = 7,
    .flags.enable_internal_pullup = true
};

i2c_master_bus_handle_t i2c_bus_handle;

void bmi270_task(void* param);
void bmp280_task(void* param);
void pwm_motor_test_task(void* param);
void mixer_debug_task(void* param);

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
        .device_address = BMP280_DEVICE,
        .scl_speed_hz = I2C_CLK_FREQ,
    };
    i2c_master_dev_handle_t bmp280_handle;

    ledc_timer_config_t pwm_motor_timer_cfg = {
        .speed_mode = PWM_SPEED_MODE,
        .duty_resolution = PWM_DUTY_RES,
        .timer_num = PWM_TIMER,
        .freq_hz = PWM_FREQUENCY,
        .clk_cfg = PWM_AUTO_CLK
    };

    pwm_motor_setup(&pwm_motor_timer_cfg, MOTOR_4);
    ctrl_output_q = xQueueCreate(1, sizeof(ctrl_output_t));
    if (ctrl_output_q == NULL)
    {
        ESP_LOGE("EvoFlight", "Create ctrl_output_q failed");
        return;
    }
    EF_ERR_CHECK(i2c_new_master_bus(&i2c_master_bus_config, &i2c_bus_handle), "main");
    ESP_LOGI("EvoFlight", "I2C bus created successfully");

    // i2c_master_bus_add_device(i2c_bus_handle, &bmp180_cfg, &bmp180_handle);
    EF_ERR_CHECK(i2c_master_bus_add_device(i2c_bus_handle, &bmi270_cfg, &bmi270_handle), "main");
    // EF_ERR_CHECK(i2c_master_bus_add_device(i2c_bus_handle, &bmp280_cfg, &bmp280_handle), "main");
    
    vTaskDelay(pdMS_TO_TICKS(1000));
    
    xTaskCreatePinnedToCore(bmi270_task, "BMI270", 10240, (void*)bmi270_handle, 4, NULL, 0);
    // xTaskCreatePinnedToCore(bmp280_task, "BMI280", 10240, (void*)bmp280_handle, 5, NULL, 0);
    xTaskCreatePinnedToCore(pwm_motor_test_task, "PWM", 4096, NULL, 5, NULL, 1);
    // xTaskCreatePinnedToCore(mixer_debug_task, "MIXER", 4096, NULL, 5, NULL, 1);
    vTaskDelay(pdMS_TO_TICKS(100));

    ESP_LOGI("EvoFlight", "Initialize I2C");
}


void bmi270_task(void* param)
{
    i2c_master_dev_handle_t bmi270_handle = (i2c_master_dev_handle_t) param;
    ESP_LOGI("LOG1" , "handle got!");
    acc_pack_t acc_pack;
    gyro_pack_t gyro_pack;
    temp_pack_t temp_pack;
    state_t* s = state();
    
    // vec4f_t test_cmd = vec4f_from_xyzt(1., 0., 0., 0.);
    // vec4f_t output = vec4f_from_xyzt(0., 0., 0., 0.);
    // mat4f_t mixer = mixer_init();
    acro_ctrl_t acro_ctrl_params = acro_ctrl_init();
    // state_t* s = malloc(sizeof(state_t));
    // s->acc = vec3f_zeros();
    // s->ang = vec3f_zeros();
    // s->omg = vec3f_zeros();
    // s->pos = vec3f_zeros();
    // s->vel = vec3f_zeros();
    float r = 0, p = 0, y = 0;
    float rr = 0, rp = 0, ry = 0;
    float tt = 0, tr = 0, tp = 0, ty = 0;
    int64_t timer;
    float dt;
    quatf_t init_quat = quatf_wxyz(1.f, .0f, .0f , .0f);
    quatf_t i2b = quatf_wxyz(0, 0, 1, 0);
    vec4f_t test_cmd = vec4f_from_xyzt(1., 0., 0., 0.);
    mahony_params_t mahony_params = {
        .Kp = 1.f,
        .Ki = 1e-3f,
        .rotation = init_quat,
        .imu_t_body = i2b,
    };
    bmi270_init(bmi270_handle, bmi270_acc_range_8g, bmi270_gyro_range_2000);
    timer = esp_timer_get_time();
    // int sample_num = 10000;
    // int i = 0;
    // int64_t test_timer = esp_timer_get_time();
    while(1)
    {
        // i++;
        // vTaskDelay(pdMS_TO_TICKS(2)); /* 500 hz*/
        // cmd: throttle, roll_rate, pitch_rate, yaw_rate
        //      0 - 1   , xxx deg/s,  xxx deg/s,  xxx deg/s
        vec4f_set(test_cmd, .5f, 0.f, 0., 300.);
        bmi270_read_acc(bmi270_handle, &acc_pack, bmi270_acc_range_8g);
        bmi270_read_gyro(bmi270_handle, &gyro_pack, bmi270_gyro_range_2000);
        bmi270_read_temp(bmi270_handle, &temp_pack);
        // ESP_LOGI("BMI270", "ax: %.2f, ay: %.2f, az: %.2f", acc_pack.ax, acc_pack.ay, acc_pack.az);
        // ESP_LOGI("BMI270", "gx: %.2f, gy: %.2f, gz: %.2f", gyro_pack.gx, gyro_pack.gy, gyro_pack.gz);
        dt = dt_s(&timer);
        // ESP_LOGI("BMI270", "%.4f", dt);
        mahony_get_deg(acc_pack, gyro_pack, &mahony_params, s, dt);
        vec3f_get(s->ang, &r, &p, &y);
        vec3f_get(s->omg, &rr, &rp, &ry);
        // ESP_LOGI("BMI270", "roll: %.2f°, pitch: %.2f°, yaw: %.2f°", r, p, y);
        acro_ctrl(s, test_cmd, &acro_ctrl_params, dt);
        vec4f_get(test_cmd, &tt, &tr, &tp, &ty);

        if (ctrl_output_q)
        {
            ctrl_output_t out = {
                .t = tt,
                .r = tr,
                .p = tp,
                .y = ty,
            };
            xQueueOverwrite(ctrl_output_q, &out);
        }

        // if (i > 100)
        // {   
            // i = 0;
            // ESP_LOGI("BMI270", "roll: %.2f°/s, pitch: %.2f°/s, yaw: %.2f°/s", rr, rp, ry);
            // printf("\n");
            // ESP_LOGI("BMI270", "T: %.4f, R: %.4f, P: %.4f, Y: %.4f", tt, tr, tp, ty);
        // }
        // mat4f_mul_vec_inplace(mixer, test_cmd);
        // vec4f_get(test_cmd, &tt, &tr, &tp, &ty);        
        // t0 = (uint16_t)(tt * 1000.0f + 1000.0f);
        // t1 = (uint16_t)(tr * 1000.0f + 1000.0f);
        // t2 = (uint16_t)(tp * 1000.0f + 1000.0f);
        // t3 = (uint16_t)(ty * 1000.0f + 1000.0f);

        // pwm_throttle_set(MOTOR_ID_0, t0);
        // pwm_throttle_set(MOTOR_ID_1, t1);
        // pwm_throttle_set(MOTOR_ID_2, t2);
        // pwm_throttle_set(MOTOR_ID_3, t3);
    }
    // ESP_LOGI("BMI270_timer", "%.4f", (float)(sample_num  / 1.f) / (float)(esp_timer_get_time() - test_timer) * 1e6);
    // vTaskDelete(NULL);
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

void pwm_motor_test_task(void* param)
{
    (void)param;
    int i = 0;
    mat4f_t mixer = mixer_init();
    vec4f_t motor_cmd = vec4f_from_xyzt(0.f, 0.f, 0.f, 0.f);
    ctrl_output_t out = {0};
    float m0 = 0.f, m1 = 0.f, m2 = 0.f, m3 = 0.f;

    pwm_throttle_set(MOTOR_ID_0, 1000);
    pwm_throttle_set(MOTOR_ID_1, 1000);
    pwm_throttle_set(MOTOR_ID_2, 1000);
    pwm_throttle_set(MOTOR_ID_3, 1000);
    ESP_LOGI("PWM", "PWM task started on core 1");

    while(1)
    {   
        i++;
        if (ctrl_output_q == NULL)
        {
            vTaskDelay(pdMS_TO_TICKS(10));
            continue;
        }

        if (xQueueReceive(ctrl_output_q, &out, portMAX_DELAY) == pdTRUE)
        {
            vec4f_set(motor_cmd, out.t, out.r, out.p, out.y);
            CTBR2throttle(mixer, motor_cmd);
            vec4f_get(motor_cmd, &m0, &m1, &m2, &m3);

            uint16_t pwm0 = (uint16_t)(fminf(fmaxf(m0, 0.f), 1.f) * 1000.f + 1000.f);
            uint16_t pwm1 = (uint16_t)(fminf(fmaxf(m1, 0.f), 1.f) * 1000.f + 1000.f);
            uint16_t pwm2 = (uint16_t)(fminf(fmaxf(m2, 0.f), 1.f) * 1000.f + 1000.f);
            uint16_t pwm3 = (uint16_t)(fminf(fmaxf(m3, 0.f), 1.f) * 1000.f + 1000.f);

            pwm_throttle_set(MOTOR_ID_0, pwm0);
            pwm_throttle_set(MOTOR_ID_1, pwm1);
            pwm_throttle_set(MOTOR_ID_2, pwm2);
            pwm_throttle_set(MOTOR_ID_3, pwm3);
            if (i > 100)
            {
                i = 0;
                printf("\n");
                ESP_LOGI("PWM", "T: %.4f, R: %.4f, P: %.4f, Y: %.4f", out.t, out.r, out.p, out.y);
                ESP_LOGI("PWM", "M0: %d, M1: %d, M2: %d, M3: %d", pwm0, pwm1, pwm2, pwm3);
            }
        }
    }
}

void mixer_debug_task(void* param)
{
    vec4f_t test_cmd = vec4f_from_xyzt(1., 0., 0., 0.);

    acro_ctrl_t acro_ctrl_ins = acro_ctrl_init();
    state_t* s = state();
    float t = 1, r = 0, p = 0, y = 0;
    while (1)
    {
        acro_ctrl(s, test_cmd, &acro_ctrl_ins, 0.001);
        vec4f_get(test_cmd, &t, &r, &p, &y);
        ESP_LOGI("MIXER", "T: %.4f, R: %.4f, P: %.4f, Y: %.4f", t, r, p, y);
        vec4f_set(test_cmd, 1, 0, 0, 0);
        vTaskDelay(pdMS_TO_TICKS(1));
    }

}