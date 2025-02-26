<<<<<<< HEAD

void app_main(void) {

    return ;
}
=======
#include "driver/i2c_master.h"
#include "driver/gpio.h"
#include "esp_err.h"
#include "esp_log.h"
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include <string.h>
#include <stdio.h>
#include <pid.h>
#define TAG "EspFlight"

void app_main(void)
{
    ESP_LOGI(TAG, "Initialize LVGL");
    func();
}
>>>>>>> 5f6b4e8 (NEW:espfilght:refined file structures)
