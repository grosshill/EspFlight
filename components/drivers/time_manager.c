#include "time_manager.h"

float dt_s(int64_t* last_time_us)
{
    int64_t now_time_us = esp_timer_get_time();
    float delta_time_s = (now_time_us - *last_time_us) / 1e6;
    *last_time_us = now_time_us;
    return delta_time_s;
}